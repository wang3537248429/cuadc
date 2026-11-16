from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import socket
import struct
import time
from typing import BinaryIO

from pymavlink import mavutil
import serial

from .config import DEFAULT_CONFIG, OnboardConfig, TelemetryRouterConfig, load_config
from .service import SystemdWatchdog


def atomic_write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)


class TlogWriter:
    """Write timestamped, frame-aligned flight-controller MAVLink logs."""

    def __init__(self, directory: Path, max_mb: int, retained_files: int):
        self.directory = directory
        self.max_bytes = max_mb * 1024 * 1024
        self.retained_files = retained_files
        self.stream: BinaryIO | None = None
        self.path: Path | None = None
        self.parser = mavutil.mavlink.MAVLink(None)
        self.parser.robust_parsing = True
        self.sequence = 0

    def _remove_old_logs(self) -> None:
        paths = sorted(
            self.directory.glob("cuadc-*.tlog"),
            key=lambda item: item.stat().st_mtime,
        )
        for old in paths[: max(0, len(paths) - self.retained_files)]:
            old.unlink(missing_ok=True)

    def _open(self) -> None:
        self.directory.mkdir(parents=True, exist_ok=True)
        self._remove_old_logs()
        stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
        self.sequence += 1
        self.path = self.directory / (
            f"cuadc-{stamp}-{os.getpid()}-{self.sequence:02d}.tlog"
        )
        self.stream = self.path.open("ab", buffering=0)

    def _rotate_if_needed(self) -> None:
        if self.stream is None:
            self._open()
            return
        if self.stream.tell() < self.max_bytes:
            return
        self.stream.close()
        self.stream = None
        self._open()

    def append(self, raw_serial_data: bytes) -> int:
        written = 0
        try:
            messages = self.parser.parse_buffer(raw_serial_data) or []
        except Exception:
            return 0
        for message in messages:
            if message.get_type() == "BAD_DATA":
                continue
            frame = message.get_msgbuf()
            if not frame:
                continue
            self._rotate_if_needed()
            assert self.stream is not None
            # pymavlink/MAVProxy TLOG timestamps are big-endian microseconds;
            # low bits are reserved for link identifiers.
            timestamp = int(time.time() * 1_000_000) & ~3
            self.stream.write(struct.pack(">Q", timestamp))
            self.stream.write(frame)
            written += 1
        return written

    def close(self) -> None:
        if self.stream is not None:
            self.stream.close()
            self.stream = None
        if self.directory.exists():
            self._remove_old_logs()


class MavlinkRouter:
    """Own the FC USB serial port and route MAVLink to the local companion."""

    def __init__(self, config: OnboardConfig):
        self.config = config
        self.router: TelemetryRouterConfig = config.telemetry_router
        self.runtime_dir = config.runtime.path()
        self.state_path = self.runtime_dir / "router_state.json"
        self.serial_port = None
        self.local_socket: socket.socket | None = None
        self.local_parser = mavutil.mavlink.MAVLink(None)
        self.local_parser.robust_parsing = True
        self.companion_addresses: set[str] = set()
        self.next_serial_attempt = 0.0
        self.last_state_write = -float("inf")
        self.stop_requested = False
        self.watchdog = SystemdWatchdog()
        self.tlog = (
            TlogWriter(
                self.runtime_dir / "tlogs",
                self.router.tlog_max_mb,
                self.router.tlog_retained_files,
            )
            if self.router.log_tlog
            else None
        )
        self.state = {
            "schema": "cuadc-mavlink-router-state.v3",
            "status": "STARTING",
            "serial_device": self.router.serial_device,
            "serial_connected": False,
            "serial_error": None,
            "companion_endpoint": (
                f"{self.router.companion_host}:{self.router.companion_port}"
            ),
            "router_local_endpoint": (
                f"{self.router.router_bind_host}:{self.router.router_local_port}"
            ),
            "topology": "RASPBERRY_PI_USB_TO_FLIGHT_CONTROLLER",
            "onboard_reports_via": "ARDUPILOT_MAVLINK_ROUTING",
            "bytes_fc_to_local": 0,
            "bytes_local_to_fc": 0,
            "dropped_local_packets": 0,
            "rejected_local_packets": 0,
            "blocked_local_prearm_check_commands": 0,
            "last_blocked_local_prearm_check_at": None,
            "tlog_frames": 0,
            "last_fc_rx_at": None,
            "last_local_rx_at": None,
            "started_at": time.time(),
        }

    @staticmethod
    def _resolve_ipv4(host: str, port: int) -> list[tuple[str, int]]:
        records = socket.getaddrinfo(
            host,
            port,
            family=socket.AF_INET,
            type=socket.SOCK_DGRAM,
        )
        unique: list[tuple[str, int]] = []
        for record in records:
            peer = (str(record[4][0]), int(record[4][1]))
            if peer not in unique:
                unique.append(peer)
        if not unique:
            raise OSError(f"no IPv4 UDP address for {host}")
        return unique

    def _open_local_socket(self) -> None:
        peers = self._resolve_ipv4(
            self.router.companion_host,
            self.router.companion_port,
        )
        self.companion_addresses = {peer[0] for peer in peers}
        local = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        local.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        local.bind(
            (self.router.router_bind_host, self.router.router_local_port)
        )
        local.setblocking(False)
        self.local_socket = local

    def _open_serial(self) -> None:
        now = time.monotonic()
        if self.serial_port is not None or now < self.next_serial_attempt:
            return
        self.next_serial_attempt = now + self.router.serial_reconnect_s
        try:
            self.serial_port = serial.Serial(
                self.router.serial_device,
                self.router.baud,
                timeout=0.02,
                write_timeout=0.5,
                exclusive=True,
            )
            self.state["serial_connected"] = True
            self.state["serial_error"] = None
            self.state["status"] = "ROUTING"
            self.watchdog.status("Flight-controller serial connected")
        except (OSError, serial.SerialException) as exc:
            self.serial_port = None
            self.state["serial_connected"] = False
            self.state["serial_error"] = f"{type(exc).__name__}: {exc}"
            self.state["status"] = "WAITING_FOR_FLIGHT_CONTROLLER"

    def _close_serial(self, exc: Exception) -> None:
        if self.serial_port is not None:
            try:
                self.serial_port.close()
            except Exception:
                pass
        self.serial_port = None
        self.state["serial_connected"] = False
        self.state["serial_error"] = f"{type(exc).__name__}: {exc}"
        self.state["status"] = "WAITING_FOR_FLIGHT_CONTROLLER"
        self.next_serial_attempt = time.monotonic() + self.router.serial_reconnect_s

    def _write_serial(self, data: bytes, source: str) -> None:
        if self.serial_port is None:
            self.state[f"dropped_{source}_packets"] += 1
            return
        try:
            written = int(self.serial_port.write(data))
            self.state[f"bytes_{source}_to_fc"] += written
        except (OSError, serial.SerialException, serial.SerialTimeoutException) as exc:
            self.state[f"dropped_{source}_packets"] += 1
            self._close_serial(exc)

    def _contains_blocked_local_command(self, data: bytes) -> bool:
        """Reject command 401 from the trusted local companion link.

        ArduPlane 4.6.2 may dereference the GPS blended driver before GPS
        initialisation when an external RUN_PREARM_CHECKS command is received.
        Normal pre-arm enforcement remains enabled and runs inside ArduPilot;
        this guard only removes the unnecessary external trigger from the Pi
        loopback route.
        """
        if not self.router.block_run_prearm_checks:
            return False
        try:
            messages = self.local_parser.parse_buffer(data) or []
        except Exception:
            # Routing arbitrary non-MAVLink bytes is retained for diagnostics.
            # Reset parsing state so malformed input cannot poison later frames.
            self.local_parser = mavutil.mavlink.MAVLink(None)
            self.local_parser.robust_parsing = True
            return False
        blocked = sum(
            1
            for message in messages
            if message.get_type() in {"COMMAND_LONG", "COMMAND_INT"}
            and int(message.command)
            == mavutil.mavlink.MAV_CMD_RUN_PREARM_CHECKS
        )
        if not blocked:
            return False
        self.state["blocked_local_prearm_check_commands"] += blocked
        self.state["last_blocked_local_prearm_check_at"] = time.time()
        return True

    def _drain_local(self) -> None:
        assert self.local_socket is not None
        while True:
            try:
                data, address = self.local_socket.recvfrom(65535)
            except BlockingIOError:
                return
            except OSError as exc:
                raise RuntimeError(f"local UDP route failed: {exc}") from exc
            if (
                address[0] not in self.companion_addresses
                or int(address[1]) != self.router.companion_port
            ):
                self.state["rejected_local_packets"] += 1
                continue
            if data:
                self.state["last_local_rx_at"] = time.time()
                if self._contains_blocked_local_command(data):
                    continue
                self._write_serial(data, "local")

    def _read_serial(self) -> None:
        if self.serial_port is None:
            return
        try:
            waiting = int(getattr(self.serial_port, "in_waiting", 0))
            data = self.serial_port.read(max(1, min(waiting, 4096)))
        except (OSError, serial.SerialException) as exc:
            self._close_serial(exc)
            return
        if not data:
            return
        now = time.time()
        self.state["last_fc_rx_at"] = now
        assert self.local_socket is not None
        try:
            sent = self.local_socket.sendto(
                data,
                (self.router.companion_host, self.router.companion_port),
            )
            self.state["bytes_fc_to_local"] += sent
        except OSError as exc:
            self.state["serial_error"] = f"local UDP send failed: {exc}"
        if self.tlog is not None:
            self.state["tlog_frames"] += self.tlog.append(data)

    def _write_state(self, force: bool = False) -> None:
        now = time.monotonic()
        if (
            not force
            and now - self.last_state_write < self.router.status_interval_s
        ):
            return
        payload = dict(self.state)
        payload["updated_at"] = time.time()
        payload["tlog_path"] = (
            str(self.tlog.path) if self.tlog and self.tlog.path else None
        )
        atomic_write_json(self.state_path, payload)
        self.last_state_write = now

    def shutdown(self, *_args) -> None:
        self.stop_requested = True

    def run(self) -> int:
        self.runtime_dir.mkdir(parents=True, exist_ok=True)
        self._open_local_socket()
        self.watchdog.ready("MAVLink router started; waiting for devices")
        try:
            while not self.stop_requested:
                self._open_serial()
                self._drain_local()
                self._read_serial()
                self._write_state()
                self.watchdog.ping()
                if self.serial_port is None:
                    time.sleep(0.05)
            return 0
        finally:
            self.watchdog.stopping()
            if self.serial_port is not None:
                self.serial_port.close()
            if self.local_socket is not None:
                self.local_socket.close()
            if self.tlog is not None:
                self.tlog.close()
            self.state["status"] = "STOPPED"
            self.state["serial_connected"] = False
            self._write_state(force=True)


def main(config: OnboardConfig) -> int:
    runtime = MavlinkRouter(config)
    signal.signal(signal.SIGINT, runtime.shutdown)
    signal.signal(signal.SIGTERM, runtime.shutdown)
    try:
        return runtime.run()
    except KeyboardInterrupt:
        return 130
    except (OSError, RuntimeError, ValueError) as exc:
        runtime.state["status"] = "FATAL"
        runtime.state["fatal_error"] = f"{type(exc).__name__}: {exc}"
        runtime._write_state(force=True)
        print(f"CUADC MAVLink router fatal: {exc}", flush=True)
        return 2


def cli_main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="CUADC flight-controller USB/local companion MAVLink router"
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG,
    )
    args = parser.parse_args(argv)
    return main(load_config(args.config))


if __name__ == "__main__":
    raise SystemExit(cli_main())
