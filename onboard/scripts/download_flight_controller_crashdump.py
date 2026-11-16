#!/usr/bin/env python3
"""Preserve an ArduPilot crash dump without clearing or mutating the aircraft.

This maintenance tool only requests ``@SYS/crash_dump.bin`` over MAVFTP.  It
never arms, changes mode, changes parameters, uploads a mission, selects a
mission item, or commands an actuator.  The normal onboard service must be
stopped so that this process is the sole client of the router's companion UDP
endpoint; the serial-owning router must remain active.
"""

from __future__ import annotations

import argparse
from datetime import datetime
import fcntl
import hashlib
import json
import logging
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
from typing import Any, TextIO

from pymavlink import mavftp, mavutil


APP_ROOT = Path(__file__).resolve().parents[1]
ROOT = APP_ROOT if (APP_ROOT / "src/cuadc_onboard").is_dir() else APP_ROOT.parent
SOURCE_ROOT = ROOT / "src"
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))

from cuadc_onboard.config import DEFAULT_CONFIG, load_config  # noqa: E402
from cuadc_onboard.mavlink_io import MavlinkClient  # noqa: E402


AUTOPILOT_COMPONENT = mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
CRASH_DUMP_REMOTE_PATH = "@SYS/crash_dump.bin"
CRASH_DUMP_MAGIC = b"cC"
MAX_CRASH_DUMP_BYTES = 64 * 1024 * 1024


class CrashDumpDownloadError(RuntimeError):
    """A safety refusal, protocol failure, or invalid crash-dump response."""


def _service_state(unit: str) -> str:
    systemctl = shutil.which("systemctl")
    if systemctl is None:
        raise CrashDumpDownloadError("systemctl is required for exclusive access")
    result = subprocess.run(
        [systemctl, "is-active", unit],
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
    )
    return result.stdout.strip()


def require_exclusive_router_access() -> None:
    main_state = _service_state("cuadc-onboard.service")
    if main_state in {"active", "activating", "reloading", "deactivating"}:
        raise CrashDumpDownloadError(
            "cuadc-onboard.service must be stopped before MAVFTP diagnostics"
        )
    if main_state not in {"inactive", "failed", "unknown", ""}:
        raise CrashDumpDownloadError(
            f"cannot prove cuadc-onboard.service is stopped (state={main_state!r})"
        )

    router_state = _service_state("cuadc-onboard-router.service")
    if router_state != "active":
        raise CrashDumpDownloadError(
            "cuadc-onboard-router.service must remain active "
            f"(state={router_state!r})"
        )


def acquire_diagnostic_lock(runtime_dir: Path) -> TextIO:
    runtime_dir.mkdir(parents=True, exist_ok=True)
    path = runtime_dir / "diagnostic-mavftp.lock"
    stream = path.open("a+", encoding="utf-8")
    try:
        fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        stream.close()
        raise CrashDumpDownloadError(
            f"another diagnostic MAVFTP process holds {path}"
        ) from exc
    stream.seek(0)
    stream.truncate()
    stream.write(f"pid={os.getpid()} started_at={time.time():.6f}\n")
    stream.flush()
    os.fsync(stream.fileno())
    return stream


def _is_target_autopilot_message(client: MavlinkClient, message: Any) -> bool:
    if message is None:
        return False
    try:
        return (
            int(message.get_srcSystem()) == client.target_system
            and int(message.get_srcComponent()) == client.target_component
            and client.target_component == AUTOPILOT_COMPONENT
        )
    except (AttributeError, TypeError, ValueError):
        return False


def wait_for_disarmed_heartbeat(
    client: MavlinkClient, timeout: float
) -> dict[str, int | bool]:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        heartbeat = client.recv(
            "HEARTBEAT",
            timeout=min(0.5, max(0.0, deadline - time.monotonic())),
        )
        if not _is_target_autopilot_message(client, heartbeat):
            continue
        armed = MavlinkClient.is_armed(heartbeat)
        if armed:
            raise CrashDumpDownloadError(
                "flight controller is armed; crash-dump download refused"
            )
        return {
            "armed": False,
            "base_mode": int(heartbeat.base_mode),
            "custom_mode": int(heartbeat.custom_mode),
            "system_status": int(heartbeat.system_status),
            "vehicle_type": int(heartbeat.type),
            "autopilot_type": int(heartbeat.autopilot),
        }
    raise CrashDumpDownloadError("fresh target-autopilot heartbeat was not received")


def _bytes_hex(value: Any) -> str:
    try:
        return bytes(value).hex()
    except (TypeError, ValueError):
        return ""


def request_autopilot_version(
    client: MavlinkClient, timeout: float = 5.0
) -> dict[str, Any] | None:
    assert client.connection is not None
    client.connection.mav.autopilot_version_request_send(
        client.target_system, client.target_component
    )
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        message = client.recv(
            "AUTOPILOT_VERSION",
            timeout=min(0.5, max(0.0, deadline - time.monotonic())),
        )
        if not _is_target_autopilot_message(client, message):
            continue
        flight = int(message.flight_sw_version)
        return {
            "flight_sw_version": flight,
            "flight_version": {
                "major": (flight >> 24) & 0xFF,
                "minor": (flight >> 16) & 0xFF,
                "patch": (flight >> 8) & 0xFF,
                "type": flight & 0xFF,
            },
            "flight_custom_version_hex": _bytes_hex(
                message.flight_custom_version
            ),
            "middleware_sw_version": int(message.middleware_sw_version),
            "middleware_custom_version_hex": _bytes_hex(
                message.middleware_custom_version
            ),
            "os_sw_version": int(message.os_sw_version),
            "os_custom_version_hex": _bytes_hex(message.os_custom_version),
            "board_version": int(message.board_version),
            "vendor_id": int(message.vendor_id),
            "product_id": int(message.product_id),
            "uid": int(message.uid),
            "uid2_hex": _bytes_hex(getattr(message, "uid2", b"")),
            "capabilities": int(message.capabilities),
        }
    return None


def read_parameters(
    client: MavlinkClient, names: tuple[str, ...], timeout: float = 8.0
) -> dict[str, float]:
    """Read a small diagnostic parameter set without requesting all params."""

    assert client.connection is not None
    remaining = set(names)
    values: dict[str, float] = {}
    for name in remaining:
        client.connection.mav.param_request_read_send(
            client.target_system,
            client.target_component,
            name.encode("ascii"),
            -1,
        )
    deadline = time.monotonic() + timeout
    while remaining and time.monotonic() < deadline:
        message = client.recv(
            "PARAM_VALUE",
            timeout=min(0.5, max(0.0, deadline - time.monotonic())),
        )
        if not _is_target_autopilot_message(client, message):
            continue
        param_id = message.param_id
        if isinstance(param_id, bytes):
            param_id = param_id.decode("ascii", errors="ignore")
        name = str(param_id).rstrip("\x00")
        if name in remaining:
            values[name] = float(message.param_value)
            remaining.remove(name)
    if remaining:
        raise CrashDumpDownloadError(
            "diagnostic parameters not received: " + ", ".join(sorted(remaining))
        )
    return values


def _fsync_directory(path: Path) -> None:
    descriptor = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _atomic_write_json(path: Path, payload: dict[str, Any]) -> None:
    temporary = path.with_suffix(path.suffix + f".tmp-{os.getpid()}")
    try:
        with temporary.open("x", encoding="utf-8") as stream:
            json.dump(payload, stream, ensure_ascii=False, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        _fsync_directory(path.parent)
    finally:
        temporary.unlink(missing_ok=True)


def download_crash_dump(
    client: MavlinkClient, output_dir: Path, timeout: float
) -> tuple[Path, bytes]:
    assert client.connection is not None
    stamp = datetime.now().astimezone().strftime("%Y%m%d-%H%M%S-%f%z")
    final_path = output_dir / (
        f"crash_dump-{stamp}-sys{client.target_system}.bin"
    )
    staging_path = final_path.with_suffix(final_path.suffix + f".part-{os.getpid()}")
    mavftp_temp_path = final_path.with_suffix(
        final_path.suffix + f".mavftp-{os.getpid()}"
    )
    ftp = mavftp.MAVFTP(
        client.connection, client.target_system, client.target_component
    )
    ftp.ftp_settings.debug = 2
    ftp.temp_filename = str(mavftp_temp_path)
    try:
        start_result = ftp.cmd_get(
            [CRASH_DUMP_REMOTE_PATH, str(staging_path)]
        )
        if start_result.error_code != mavftp.FtpError.Success:
            raise CrashDumpDownloadError(
                "MAVFTP refused crash-dump request: "
                f"{start_result.error_code.name}"
            )
        result = ftp.process_ftp_reply("GetCrashDump", timeout=timeout)
        if result.error_code != mavftp.FtpError.Success or not ftp.done:
            raise CrashDumpDownloadError(
                "MAVFTP crash-dump download failed: "
                f"result={result.error_code.name}, done={ftp.done}, "
                f"remote_size={ftp.remote_file_size}, read_total={ftp.read_total}, "
                f"eof={ftp.reached_eof}, gaps={len(ftp.read_gaps)}"
            )
        payload = staging_path.read_bytes()
        if not payload:
            raise CrashDumpDownloadError("downloaded crash dump is empty")
        if len(payload) > MAX_CRASH_DUMP_BYTES:
            raise CrashDumpDownloadError(
                f"crash dump is implausibly large ({len(payload)} bytes)"
            )
        if not payload.startswith(CRASH_DUMP_MAGIC):
            raise CrashDumpDownloadError(
                "downloaded file lacks ArduPilot crash-dump magic 0x6343"
            )
        with staging_path.open("rb") as stream:
            os.fsync(stream.fileno())
        os.replace(staging_path, final_path)
        _fsync_directory(output_dir)
        return final_path, payload
    finally:
        staging_path.unlink(missing_ok=True)
        mavftp_temp_path.unlink(missing_ok=True)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=ROOT / ".runtime/onboard/flight_controller_diagnostics",
    )
    parser.add_argument("--timeout", type=float, default=180.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )
    if args.timeout <= 5.0:
        raise CrashDumpDownloadError("--timeout must be greater than 5 seconds")
    config = load_config(args.config)
    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    require_exclusive_router_access()
    lock = acquire_diagnostic_lock(ROOT / ".runtime/onboard")
    client = MavlinkClient(
        config.flight_controller.endpoint, config.flight_controller.baud
    )
    try:
        client.connect(timeout=config.flight_controller.heartbeat_timeout_s)
        before = wait_for_disarmed_heartbeat(client, timeout=5.0)
        version = request_autopilot_version(client)
        safety_parameters = read_parameters(
            client, ("ARMING_CHECK", "ARMING_CRSDP_IGN")
        )
        # Version discovery may have consumed queued heartbeats.  Re-prove the
        # aircraft is disarmed immediately before opening the MAVFTP session.
        before_ftp = wait_for_disarmed_heartbeat(client, timeout=5.0)
        dump_path, payload = download_crash_dump(client, output_dir, args.timeout)
        after = wait_for_disarmed_heartbeat(client, timeout=8.0)
        digest = hashlib.sha256(payload).hexdigest()
        metadata = {
            "captured_at": datetime.now().astimezone().isoformat(),
            "remote_path": CRASH_DUMP_REMOTE_PATH,
            "local_path": str(dump_path),
            "sha256": digest,
            "size_bytes": len(payload),
            "magic_hex": payload[:2].hex(),
            "target_system": client.target_system,
            "target_component": client.target_component,
            "endpoint": config.flight_controller.endpoint,
            "heartbeat_before": before,
            "heartbeat_before_ftp": before_ftp,
            "heartbeat_after": after,
            "autopilot_version": version,
            "safety_parameters": safety_parameters,
            "mutations_performed": [],
            "crash_dump_cleared": False,
        }
        metadata_path = dump_path.with_suffix(dump_path.suffix + ".json")
        _atomic_write_json(metadata_path, metadata)
        print(
            json.dumps(
                {
                    "ok": True,
                    "crash_dump": str(dump_path),
                    "metadata": str(metadata_path),
                    "size_bytes": len(payload),
                    "sha256": digest,
                    "target_system": client.target_system,
                    "target_component": client.target_component,
                    "armed_before": before["armed"],
                    "armed_after": after["armed"],
                    "crash_dump_cleared": False,
                },
                ensure_ascii=False,
                sort_keys=True,
            )
        )
        return 0
    finally:
        if client.connection is not None:
            client.connection.close()
        lock.close()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except CrashDumpDownloadError as exc:
        print(f"[CUADC] REFUSED/FAILED: {exc}", file=sys.stderr)
        raise SystemExit(2) from exc
