#!/usr/bin/env python3
"""Collect read-only ArduPilot evidence before firmware maintenance.

The normal onboard service must be stopped and the serial router left active.
This tool only requests parameters, mission items, version information and a
small allowlist of ``@SYS`` diagnostic files.  It never sets a parameter,
clears a log/dump, writes a mission, arms, changes mode, or commands outputs.
"""

from __future__ import annotations

import argparse
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time
from typing import Any

from pymavlink import mavftp, mavutil


APP_ROOT = Path(__file__).resolve().parents[1]
ROOT = APP_ROOT if (APP_ROOT / "src/cuadc_onboard").is_dir() else APP_ROOT.parent
SOURCE_ROOT = ROOT / "src"
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))
if str(Path(__file__).resolve().parent) not in sys.path:
    sys.path.insert(0, str(Path(__file__).resolve().parent))

from cuadc_onboard.config import DEFAULT_CONFIG, load_config  # noqa: E402
from cuadc_onboard.mavlink_io import MavlinkClient  # noqa: E402
from cuadc_onboard.mission import MissionItem  # noqa: E402
from download_flight_controller_crashdump import (  # noqa: E402
    CrashDumpDownloadError,
    acquire_diagnostic_lock,
    request_autopilot_version,
    require_exclusive_router_access,
    wait_for_disarmed_heartbeat,
)


SYS_FILES = (
    "@SYS/persistent.parm",
    "@SYS/tasks.txt",
    "@SYS/threads.txt",
    "@SYS/memory.txt",
    "@SYS/uarts.txt",
    "@SYS/dma.txt",
    "@SYS/timers.txt",
    "@SYS/storage.bin",
)
AUTOPILOT_COMPONENT = mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1


class EvidenceCollectionError(RuntimeError):
    """A safety refusal or incomplete read-only collection."""


def _is_target_autopilot(client: MavlinkClient, message: Any) -> bool:
    try:
        return bool(
            message is not None
            and int(message.get_srcSystem()) == client.target_system
            and int(message.get_srcComponent()) == client.target_component
            and client.target_component == AUTOPILOT_COMPONENT
        )
    except (AttributeError, TypeError, ValueError):
        return False


def _fsync_directory(path: Path) -> None:
    descriptor = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def atomic_bytes(path: Path, payload: bytes) -> None:
    temporary = path.with_suffix(path.suffix + f".tmp-{os.getpid()}")
    try:
        with temporary.open("x+b") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        _fsync_directory(path.parent)
    finally:
        temporary.unlink(missing_ok=True)


def atomic_json(path: Path, payload: dict) -> None:
    atomic_bytes(
        path,
        (json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n").encode(
            "utf-8"
        ),
    )


def read_all_parameters(
    client: MavlinkClient,
    timeout: float = 120.0,
) -> tuple[dict[str, float], dict[str, int], int]:
    assert client.connection is not None
    values: dict[str, float] = {}
    types: dict[str, int] = {}
    expected_count = 0
    deadline = time.monotonic() + timeout
    next_request = -math.inf
    last_new = time.monotonic()
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_request:
            client.connection.mav.param_request_list_send(
                client.target_system, client.target_component
            )
            next_request = now + 8.0
        message = client.recv("PARAM_VALUE", timeout=0.5)
        if not _is_target_autopilot(client, message):
            continue
        name = message.param_id
        if isinstance(name, bytes):
            name = name.decode("ascii", errors="ignore")
        name = str(name).rstrip("\x00")
        if not name:
            continue
        if name not in values:
            last_new = time.monotonic()
        values[name] = float(message.param_value)
        types[name] = int(message.param_type)
        expected_count = max(expected_count, int(message.param_count))
        if expected_count > 0 and len(values) >= expected_count:
            break
        # Some firmwares report a count that includes hidden/unavailable
        # entries.  A fully quiet stream after repeated list requests is the
        # only safe fallback; the metadata preserves both counts.
        if (
            expected_count > 0
            and len(values) >= expected_count - 2
            and time.monotonic() - last_new > 12.0
        ):
            break
    if not values:
        raise EvidenceCollectionError("no flight-controller parameters received")
    if expected_count and len(values) < expected_count - 2:
        raise EvidenceCollectionError(
            f"parameter download incomplete: received={len(values)}, "
            f"reported={expected_count}"
        )
    return values, types, expected_count


def parameter_text(values: dict[str, float]) -> str:
    return "".join(f"{name:<16.16s} {value:.9g}\n" for name, value in sorted(values.items()))


def _float_text(value: float) -> str:
    if not math.isfinite(value):
        raise EvidenceCollectionError(f"non-finite mission value {value!r}")
    return format(float(value), ".17g")


def mission_text(items: list[MissionItem]) -> str:
    lines = ["QGC WPL 110"]
    for expected_seq, item in enumerate(items):
        if item.seq != expected_seq:
            raise EvidenceCollectionError(
                f"mission sequence is not contiguous at {expected_seq}: {item.seq}"
            )
        lines.append(
            "\t".join(
                (
                    str(item.seq),
                    str(item.current),
                    str(item.frame),
                    str(item.command),
                    _float_text(item.param1),
                    _float_text(item.param2),
                    _float_text(item.param3),
                    _float_text(item.param4),
                    _float_text(item.x),
                    _float_text(item.y),
                    _float_text(item.z),
                    str(item.autocontinue),
                )
            )
        )
    return "\n".join(lines) + "\n"


def mavftp_download(
    client: MavlinkClient,
    remote_path: str,
    local_path: Path,
    timeout: float,
) -> bytes:
    assert client.connection is not None
    staging = local_path.with_suffix(local_path.suffix + f".part-{os.getpid()}")
    internal = local_path.with_suffix(local_path.suffix + f".mavftp-{os.getpid()}")
    ftp = mavftp.MAVFTP(
        client.connection, client.target_system, client.target_component
    )
    ftp.temp_filename = str(internal)
    try:
        started = ftp.cmd_get([remote_path, str(staging)])
        if started.error_code != mavftp.FtpError.Success:
            raise EvidenceCollectionError(
                f"MAVFTP start failed for {remote_path}: {started.error_code.name}"
            )
        result = ftp.process_ftp_reply("CollectEvidence", timeout=timeout)
        if result.error_code != mavftp.FtpError.Success or not ftp.done:
            raise EvidenceCollectionError(
                f"MAVFTP failed for {remote_path}: "
                f"{result.error_code.name}, done={ftp.done}"
            )
        payload = staging.read_bytes()
        atomic_bytes(local_path, payload)
        return payload
    finally:
        staging.unlink(missing_ok=True)
        internal.unlink(missing_ok=True)


def list_directory(client: MavlinkClient, path: str) -> list[dict]:
    assert client.connection is not None
    ftp = mavftp.MAVFTP(
        client.connection, client.target_system, client.target_component
    )
    result = ftp.cmd_list([path])
    if result.error_code != mavftp.FtpError.Success:
        raise EvidenceCollectionError(
            f"MAVFTP directory listing failed for {path}: {result.error_code.name}"
        )
    return [
        {"name": entry.name, "size_b": entry.size_b, "is_dir": entry.is_dir}
        for entry in ftp.list_result
    ]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument(
        "--output-root",
        type=Path,
        default=ROOT / ".runtime/onboard/flight_controller_diagnostics",
    )
    parser.add_argument("--timeout", type=float, default=180.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.timeout <= 5.0:
        raise EvidenceCollectionError("--timeout must be greater than 5 seconds")
    require_exclusive_router_access()
    lock = acquire_diagnostic_lock(ROOT / ".runtime/onboard")
    config = load_config(args.config)
    client = MavlinkClient(
        config.flight_controller.endpoint, config.flight_controller.baud
    )
    stamp = datetime.now().astimezone().strftime("%Y%m%d-%H%M%S-%f%z")
    output_dir = args.output_root.expanduser().resolve() / f"evidence-{stamp}"
    output_dir.mkdir(parents=True, exist_ok=False)
    try:
        client.connect(timeout=config.flight_controller.heartbeat_timeout_s)
        heartbeat_before = wait_for_disarmed_heartbeat(client, timeout=6.0)
        version = request_autopilot_version(client)
        parameters, parameter_types, reported_parameter_count = read_all_parameters(
            client
        )
        params_payload = parameter_text(parameters).encode("utf-8")
        atomic_bytes(output_dir / "all-parameters.param", params_payload)
        atomic_json(
            output_dir / "all-parameters.json",
            {
                "values": parameters,
                "mav_param_types": parameter_types,
                "received_count": len(parameters),
                "reported_count": reported_parameter_count,
            },
        )

        mission = client.download_mission(timeout=20.0)
        mission_payload = mission_text(mission).encode("utf-8")
        atomic_bytes(output_dir / "flight-controller-mission.waypoints", mission_payload)

        listings = {
            "@SYS": list_directory(client, "@SYS"),
            "/": list_directory(client, "/"),
            "/APM/LOGS": list_directory(client, "/APM/LOGS"),
        }
        files: dict[str, dict] = {}
        for remote_path in SYS_FILES:
            local_path = output_dir / remote_path.removeprefix("@SYS/")
            try:
                payload = mavftp_download(
                    client, remote_path, local_path, args.timeout
                )
                files[remote_path] = {
                    "available": True,
                    "path": str(local_path),
                    "size_bytes": len(payload),
                    "sha256": hashlib.sha256(payload).hexdigest(),
                }
            except EvidenceCollectionError as exc:
                # @SYS is a virtual directory: the generic listing advertises
                # entries that a particular board/build may not implement.
                # Preserve that fact but continue collecting the remaining
                # independent evidence files.
                files[remote_path] = {
                    "available": False,
                    "error": str(exc),
                }
        heartbeat_after = wait_for_disarmed_heartbeat(client, timeout=8.0)
        metadata = {
            "ok": True,
            "captured_at": datetime.now().astimezone().isoformat(),
            "output_dir": str(output_dir),
            "target_system": client.target_system,
            "target_component": client.target_component,
            "heartbeat_before": heartbeat_before,
            "heartbeat_after": heartbeat_after,
            "autopilot_version": version,
            "parameters": {
                "received_count": len(parameters),
                "reported_count": reported_parameter_count,
                "sha256": hashlib.sha256(params_payload).hexdigest(),
            },
            "mission": {
                "count": len(mission),
                "sha256": hashlib.sha256(mission_payload).hexdigest(),
            },
            "directory_listings": listings,
            "files": files,
            "mutations_performed": [],
        }
        atomic_json(output_dir / "evidence.json", metadata)
        print(
            json.dumps(
                {
                    "ok": True,
                    "output_dir": str(output_dir),
                    "parameter_count": len(parameters),
                    "reported_parameter_count": reported_parameter_count,
                    "mission_count": len(mission),
                    "sys_files": len(files),
                    "sd_root_entries": len(listings["/"]),
                    "sd_log_entries": len(listings["/APM/LOGS"]),
                    "armed_before": heartbeat_before["armed"],
                    "armed_after": heartbeat_after["armed"],
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
    except (EvidenceCollectionError, CrashDumpDownloadError) as exc:
        print(f"[CUADC] REFUSED/FAILED: {exc}", file=sys.stderr)
        raise SystemExit(2) from exc
