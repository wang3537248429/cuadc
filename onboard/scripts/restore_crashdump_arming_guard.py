#!/usr/bin/env python3
"""Restore ArduPilot's CrashDump arming guard after evidence preservation.

The only flight-controller mutation this tool permits is changing
``ARMING_CRSDP_IGN`` from 1 to 0.  It requires the hash of a locally preserved,
valid ArduPilot crash dump, proves the aircraft is disarmed before and after,
and writes an auditable receipt.  It does not clear the dump, arm, change mode,
touch the mission, or command an actuator.
"""

from __future__ import annotations

import argparse
from datetime import datetime
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time
from typing import TextIO

from pymavlink import mavutil


APP_ROOT = Path(__file__).resolve().parents[1]
ROOT = APP_ROOT if (APP_ROOT / "src/cuadc_onboard").is_dir() else APP_ROOT.parent
SOURCE_ROOT = ROOT / "src"
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))

from cuadc_onboard.config import DEFAULT_CONFIG, load_config  # noqa: E402
from cuadc_onboard.mavlink_io import MavlinkClient  # noqa: E402


SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
CRASH_DUMP_MAGIC = b"cC"
AUTOPILOT_COMPONENT = mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1


class GuardRestoreError(RuntimeError):
    """A safety refusal or parameter confirmation failure."""


def _service_state(unit: str) -> str:
    systemctl = shutil.which("systemctl")
    if systemctl is None:
        raise GuardRestoreError("systemctl is required for exclusive access")
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
        raise GuardRestoreError(
            "cuadc-onboard.service must be stopped before parameter maintenance"
        )
    if main_state not in {"inactive", "failed", "unknown", ""}:
        raise GuardRestoreError(
            f"cannot prove cuadc-onboard.service is stopped (state={main_state!r})"
        )
    router_state = _service_state("cuadc-onboard-router.service")
    if router_state != "active":
        raise GuardRestoreError(
            "cuadc-onboard-router.service must remain active "
            f"(state={router_state!r})"
        )


def acquire_lock(runtime_dir: Path) -> TextIO:
    runtime_dir.mkdir(parents=True, exist_ok=True)
    # All tools which consume the router's single companion endpoint must use
    # the same advisory lock.  A separate "maintenance" lock would allow this
    # parameter write to race a crash-dump/evidence MAVFTP session.
    path = runtime_dir / "diagnostic-mavftp.lock"
    stream = path.open("a+", encoding="utf-8")
    try:
        fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        stream.close()
        raise GuardRestoreError(f"another maintenance process holds {path}") from exc
    stream.seek(0)
    stream.truncate()
    stream.write(f"pid={os.getpid()} started_at={time.time():.6f}\n")
    stream.flush()
    os.fsync(stream.fileno())
    return stream


def verify_preserved_dump(path: Path, expected_hash: str) -> tuple[int, str]:
    if not SHA256_RE.fullmatch(expected_hash):
        raise GuardRestoreError("--confirm-crashdump-sha256 must be lowercase SHA-256")
    try:
        payload = path.expanduser().resolve(strict=True).read_bytes()
    except OSError as exc:
        raise GuardRestoreError(f"preserved crash dump cannot be read: {exc}") from exc
    if not payload.startswith(CRASH_DUMP_MAGIC):
        raise GuardRestoreError("preserved file lacks ArduPilot crash-dump magic 0x6343")
    actual_hash = hashlib.sha256(payload).hexdigest()
    if actual_hash != expected_hash:
        raise GuardRestoreError(
            f"preserved crash-dump SHA-256 mismatch: {actual_hash}"
        )
    return len(payload), actual_hash


def _is_target_heartbeat(client: MavlinkClient, message) -> bool:
    try:
        return bool(
            message is not None
            and int(message.get_srcSystem()) == client.target_system
            and int(message.get_srcComponent()) == client.target_component
            and client.target_component == AUTOPILOT_COMPONENT
        )
    except (AttributeError, TypeError, ValueError):
        return False


def wait_disarmed(client: MavlinkClient, timeout: float = 6.0) -> dict:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        heartbeat = client.recv(
            "HEARTBEAT",
            timeout=min(0.5, max(0.0, deadline - time.monotonic())),
        )
        if not _is_target_heartbeat(client, heartbeat):
            continue
        if client.is_armed(heartbeat):
            raise GuardRestoreError(
                "flight controller is armed; guard restoration refused"
            )
        return {
            "armed": False,
            "base_mode": int(heartbeat.base_mode),
            "custom_mode": int(heartbeat.custom_mode),
            "system_status": int(heartbeat.system_status),
        }
    raise GuardRestoreError("fresh target-autopilot heartbeat was not received")


def _fsync_directory(path: Path) -> None:
    descriptor = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def atomic_json(path: Path, payload: dict) -> None:
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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--crash-dump", type=Path, required=True)
    parser.add_argument("--confirm-crashdump-sha256", required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    dump_size, dump_hash = verify_preserved_dump(
        args.crash_dump, args.confirm_crashdump_sha256
    )
    require_exclusive_router_access()
    lock = acquire_lock(ROOT / ".runtime/onboard")
    config = load_config(args.config)
    client = MavlinkClient(
        config.flight_controller.endpoint, config.flight_controller.baud
    )
    changed = False
    try:
        client.connect(timeout=config.flight_controller.heartbeat_timeout_s)
        before_heartbeat = wait_disarmed(client)
        before = {
            "ARMING_CHECK": client.read_parameter("ARMING_CHECK", timeout=5.0),
            "ARMING_CRSDP_IGN": client.read_parameter(
                "ARMING_CRSDP_IGN", timeout=5.0
            ),
        }
        if abs(before["ARMING_CHECK"] - 1.0) > 1e-6:
            raise GuardRestoreError(
                f"ARMING_CHECK is not the required all-checks value 1 ({before['ARMING_CHECK']})"
            )
        # Parameter reads can take several seconds on a shared telemetry link.
        # Re-prove the aircraft is disarmed immediately before the only
        # permitted mutation instead of relying on the connection heartbeat.
        before_mutation_heartbeat = wait_disarmed(client)
        if abs(before["ARMING_CRSDP_IGN"] - 1.0) <= 1e-6:
            client.set_parameters({"ARMING_CRSDP_IGN": 0.0})
            changed = True
        elif abs(before["ARMING_CRSDP_IGN"]) > 1e-6:
            raise GuardRestoreError(
                "ARMING_CRSDP_IGN has an unexpected value: "
                f"{before['ARMING_CRSDP_IGN']}"
            )
        after_value = client.read_parameter("ARMING_CRSDP_IGN", timeout=5.0)
        if abs(after_value) > 1e-6:
            raise GuardRestoreError(
                f"ARMING_CRSDP_IGN readback is not 0 ({after_value})"
            )
        after_heartbeat = wait_disarmed(client)
        stamp = datetime.now().astimezone().strftime("%Y%m%d-%H%M%S-%f%z")
        receipt_path = (
            ROOT
            / ".runtime/onboard/flight_controller_diagnostics"
            / f"crashdump-arming-guard-restored-{stamp}.json"
        )
        receipt_path.parent.mkdir(parents=True, exist_ok=True)
        receipt = {
            "ok": True,
            "recorded_at": datetime.now().astimezone().isoformat(),
            "target_system": client.target_system,
            "target_component": client.target_component,
            "preserved_crash_dump": str(args.crash_dump.resolve()),
            "preserved_crash_dump_size": dump_size,
            "preserved_crash_dump_sha256": dump_hash,
            "before": before,
            "after": {
                "ARMING_CHECK": before["ARMING_CHECK"],
                "ARMING_CRSDP_IGN": after_value,
            },
            "changed": changed,
            "heartbeat_before": before_heartbeat,
            "heartbeat_before_mutation": before_mutation_heartbeat,
            "heartbeat_after": after_heartbeat,
            "crash_dump_cleared": False,
            "allowed_mutation": "ARMING_CRSDP_IGN: 1 -> 0",
        }
        atomic_json(receipt_path, receipt)
        print(
            json.dumps(
                {
                    "ok": True,
                    "changed": changed,
                    "ARMING_CHECK": before["ARMING_CHECK"],
                    "ARMING_CRSDP_IGN_before": before["ARMING_CRSDP_IGN"],
                    "ARMING_CRSDP_IGN_after": after_value,
                    "armed_before": before_heartbeat["armed"],
                    "armed_after": after_heartbeat["armed"],
                    "crash_dump_cleared": False,
                    "receipt": str(receipt_path),
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
    except GuardRestoreError as exc:
        print(f"[CUADC] REFUSED/FAILED: {exc}", file=sys.stderr)
        raise SystemExit(2) from exc
