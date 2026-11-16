#!/usr/bin/env python3
"""Safely install the mission pinned by the onboard configuration.

This is an explicit maintenance tool, not part of normal startup.  Before
running it on the Raspberry Pi, stop ``cuadc-onboard.service`` while leaving
``cuadc-onboard-router.service`` running.  The installer refuses an active
main service when systemd can report its state, and also takes an exclusive
process lock.  The main service must remain stopped until the final readback
verification has completed.

The tool never arms, changes mode/current mission item, or sends a servo
command.  Its only flight-controller writes are the standard full mission
clear/count/item transfer required by an explicitly confirmed, hash-pinned
mission installation.
"""

from __future__ import annotations

import argparse
from datetime import datetime
import fcntl
import hashlib
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time
from typing import Iterable, Sequence, TextIO

from pymavlink import mavutil


APP_ROOT = Path(__file__).resolve().parents[1]
ROOT = APP_ROOT if (APP_ROOT / "src/cuadc_onboard").is_dir() else APP_ROOT.parent
SOURCE_ROOT = ROOT / "src"
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))

from cuadc_onboard.config import load_config  # noqa: E402
from cuadc_onboard.mavlink_io import MavlinkClient, MavlinkTimeout  # noqa: E402
from cuadc_onboard.mission import (  # noqa: E402
    DO_SET_SERVO,
    MissionContract,
    MissionItem,
    compare_mission_reference,
    load_qgc_waypoints,
    validate_mission,
)


SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
MISSION_TYPE = mavutil.mavlink.MAV_MISSION_TYPE_MISSION
AUTOPILOT_COMPONENT = mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
GLOBAL_FRAMES = {
    mavutil.mavlink.MAV_FRAME_GLOBAL,
    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT,
    mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
    mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT,
}


class MissionInstallError(RuntimeError):
    """Refusal or protocol failure during a mission installation."""


def sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def require_exact_pin(config, mission_path: Path, confirmation: str) -> tuple[Path, str]:
    pinned_path = config.mission.expected_path()
    if pinned_path is None or not config.mission.expected_sha256:
        raise MissionInstallError(
            "mission.expected_file and mission.expected_sha256 must both be pinned"
        )
    try:
        requested_path = mission_path.expanduser().resolve(strict=True)
        pinned_path = pinned_path.resolve(strict=True)
    except OSError as exc:
        raise MissionInstallError(f"mission path cannot be resolved: {exc}") from exc
    if requested_path != pinned_path:
        raise MissionInstallError(
            "--mission does not exactly match mission.expected_file: "
            f"requested={requested_path}, pinned={pinned_path}"
        )

    pinned_hash = str(config.mission.expected_sha256)
    if not SHA256_RE.fullmatch(pinned_hash):
        raise MissionInstallError(
            "mission.expected_sha256 must be exactly 64 lowercase hexadecimal characters"
        )
    if confirmation != pinned_hash:
        raise MissionInstallError(
            "--confirm-sha256 does not exactly match mission.expected_sha256"
        )
    actual_hash = sha256_bytes(requested_path.read_bytes())
    if actual_hash != pinned_hash:
        raise MissionInstallError(
            f"mission file SHA-256 changed: pinned={pinned_hash}, actual={actual_hash}"
        )
    return requested_path, actual_hash


def require_main_service_stopped() -> None:
    """Reject a known-running main service; the router is intentionally allowed."""

    systemctl = shutil.which("systemctl")
    if systemctl is None:
        print(
            "[CUADC] WARNING: systemctl is unavailable; the operator must verify "
            "cuadc-onboard.service is stopped.",
            file=sys.stderr,
        )
        return
    result = subprocess.run(
        [systemctl, "is-active", "cuadc-onboard.service"],
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
    )
    state = result.stdout.strip()
    if state in {"active", "activating", "reloading", "deactivating"}:
        raise MissionInstallError(
            "cuadc-onboard.service is not stopped; stop it while leaving "
            "cuadc-onboard-router.service running"
        )
    if state not in {"inactive", "failed", "unknown", ""}:
        raise MissionInstallError(
            f"cannot prove cuadc-onboard.service is stopped (state={state!r})"
        )


def acquire_install_lock(runtime_dir: Path) -> TextIO:
    runtime_dir.mkdir(parents=True, exist_ok=True)
    lock_path = runtime_dir / "mission-install.lock"
    stream = lock_path.open("a+", encoding="utf-8")
    try:
        fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        stream.close()
        raise MissionInstallError(
            f"another mission installer holds {lock_path}"
        ) from exc
    stream.seek(0)
    stream.truncate()
    stream.write(f"pid={os.getpid()} started_at={time.time():.6f}\n")
    stream.flush()
    os.fsync(stream.fileno())
    return stream


def _float_text(value: float) -> str:
    if math.isnan(value):
        return "nan"
    if not math.isfinite(value):
        raise MissionInstallError(f"cannot back up non-finite mission value {value!r}")
    return format(float(value), ".17g")


def qgc_waypoint_text(items: Sequence[MissionItem]) -> str:
    lines = ["QGC WPL 110"]
    for expected_seq, item in enumerate(items):
        if item.seq != expected_seq:
            raise MissionInstallError(
                f"downloaded mission sequence is not contiguous at {expected_seq}: {item.seq}"
            )
        columns = (
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
        lines.append("\t".join(columns))
    return "\n".join(lines) + "\n"


def save_timestamped_backup(
    runtime_dir: Path,
    items: Sequence[MissionItem],
    target_system: int,
) -> tuple[Path, str]:
    backup_dir = runtime_dir / "mission_backups"
    backup_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().astimezone().strftime("%Y%m%d-%H%M%S-%f%z")
    path = backup_dir / f"pre-install-{stamp}-sys{target_system}.waypoints"
    temporary = path.with_suffix(path.suffix + f".tmp-{os.getpid()}")
    payload = qgc_waypoint_text(items).encode("utf-8")
    try:
        with temporary.open("xb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        _fsync_directory(backup_dir)
    finally:
        temporary.unlink(missing_ok=True)
    return path, sha256_bytes(payload)


def _fsync_directory(path: Path) -> None:
    directory_fd = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(directory_fd)
    finally:
        os.close(directory_fd)


def archive_stale_patch_backup(runtime_dir: Path) -> tuple[Path, str] | None:
    """Preserve an old dynamic-patch backup after a verified full install."""

    source = runtime_dir / "mission_patch_backup.json"
    if not source.is_file():
        return None
    backup_dir = runtime_dir / "mission_backups"
    backup_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().astimezone().strftime("%Y%m%d-%H%M%S-%f%z")
    destination = backup_dir / (
        f"stale-post-install-mission-patch-{stamp}-pid{os.getpid()}.json"
    )
    payload_hash = sha256_bytes(source.read_bytes())
    os.replace(source, destination)
    _fsync_directory(runtime_dir)
    _fsync_directory(backup_dir)
    return destination, payload_hash


def physical_install_mission(
    reference: Sequence[MissionItem],
    contract: MissionContract,
    release_enabled: bool,
) -> list[MissionItem]:
    """Build the active mission that may reside in the flight controller."""

    release_seq = contract.fallback_release_seq
    release_items = [item for item in reference if item.command == DO_SET_SERVO]
    if len(release_items) != 1 or release_items[0].seq != release_seq:
        raise MissionInstallError(
            "approved mission must contain exactly one DO_SET_SERVO at the "
            f"contract release slot {release_seq}"
        )
    if not release_enabled:
        raise MissionInstallError(
            "active-only mission installer requires release.enabled=true; "
            "flight-controller mission was not changed"
        )
    return list(reference)


def semantic_install_readback(
    readback: Sequence[MissionItem],
    reference: Sequence[MissionItem],
    contract: MissionContract,
    release_enabled: bool,
) -> list[MissionItem]:
    """Return an active readback; disabled installation is always refused."""

    if not release_enabled:
        raise MissionInstallError(
            "active-only mission installer requires release.enabled=true"
        )
    return list(readback)


def _source_matches_target(client: MavlinkClient, message) -> bool:
    return client.mission_message_is_for_this_client(message)


def _validate_autopilot_heartbeat(client: MavlinkClient, heartbeat) -> None:
    if heartbeat is None or heartbeat.get_type() != "HEARTBEAT":
        raise MissionInstallError("a flight-controller HEARTBEAT was not received")
    source_system = int(heartbeat.get_srcSystem())
    source_component = int(heartbeat.get_srcComponent())
    if source_system <= 0 or source_system != client.target_system:
        raise MissionInstallError(
            f"unexpected heartbeat system {source_system}; target is {client.target_system}"
        )
    if source_component != AUTOPILOT_COMPONENT or source_component != client.target_component:
        raise MissionInstallError(
            f"heartbeat component {source_component} is not the target autopilot component"
        )
    if int(heartbeat.autopilot) == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
        raise MissionInstallError("heartbeat source is not a flight controller")
    if MavlinkClient.is_armed(heartbeat):
        raise MissionInstallError("flight controller is armed; mission installation refused")


def require_fresh_disarmed_heartbeat(
    client: MavlinkClient,
    timeout: float,
) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        heartbeat = client.recv(
            "HEARTBEAT",
            timeout=max(0.0, min(0.5, deadline - time.monotonic())),
        )
        if heartbeat is None or not _source_matches_target(client, heartbeat):
            continue
        _validate_autopilot_heartbeat(client, heartbeat)
        return
    raise MavlinkTimeout("fresh target flight-controller HEARTBEAT not received")


def _check_heartbeat_during_transfer(client: MavlinkClient, message) -> bool:
    if message.get_type() != "HEARTBEAT":
        return False
    if _source_matches_target(client, message):
        _validate_autopilot_heartbeat(client, message)
    return True


def _send_item_int(client: MavlinkClient, item: MissionItem) -> None:
    assert client.connection is not None
    if item.frame in GLOBAL_FRAMES:
        x = int(round(item.x * 1e7))
        y = int(round(item.y * 1e7))
    else:
        x = int(round(item.x))
        y = int(round(item.y))
    client.connection.mav.mission_item_int_send(
        client.target_system,
        client.target_component,
        item.seq,
        item.frame,
        item.command,
        item.current,
        item.autocontinue,
        item.param1,
        item.param2,
        item.param3,
        item.param4,
        x,
        y,
        item.z,
        MISSION_TYPE,
    )


def _send_item_float(client: MavlinkClient, item: MissionItem) -> None:
    assert client.connection is not None
    client.connection.mav.mission_item_send(
        client.target_system,
        client.target_component,
        item.seq,
        item.frame,
        item.command,
        item.current,
        item.autocontinue,
        item.param1,
        item.param2,
        item.param3,
        item.param4,
        item.x,
        item.y,
        item.z,
        MISSION_TYPE,
    )


def _wait_for_clear_ack(client: MavlinkClient, timeout: float) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        message = client.recv(
            ["MISSION_ACK", "HEARTBEAT"],
            timeout=max(0.0, min(0.5, deadline - time.monotonic())),
        )
        if message is None or _check_heartbeat_during_transfer(client, message):
            continue
        if not _source_matches_target(client, message):
            continue
        result = int(message.type)
        if result != mavutil.mavlink.MAV_MISSION_ACCEPTED:
            raise MissionInstallError(f"MISSION_CLEAR_ALL rejected with ACK {result}")
        return
    raise MavlinkTimeout("MISSION_CLEAR_ALL ACK not received")


def upload_full_mission(
    client: MavlinkClient,
    items: Sequence[MissionItem],
    timeout: float,
) -> None:
    if not items:
        raise MissionInstallError("refusing to upload an empty mission")
    expected_sequences = set(range(len(items)))
    if {item.seq for item in items} != expected_sequences:
        raise MissionInstallError("mission sequences must be contiguous from zero")
    assert client.connection is not None

    client.connection.mav.mission_clear_all_send(
        client.target_system,
        client.target_component,
        MISSION_TYPE,
    )
    _wait_for_clear_ack(client, min(5.0, timeout))
    require_fresh_disarmed_heartbeat(client, min(3.0, timeout))

    client.connection.mav.mission_count_send(
        client.target_system,
        client.target_component,
        len(items),
        MISSION_TYPE,
    )
    served: set[int] = set()
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        message = client.recv(
            ["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK", "HEARTBEAT"],
            timeout=max(0.0, min(0.5, deadline - time.monotonic())),
        )
        if message is None or _check_heartbeat_during_transfer(client, message):
            continue
        if not _source_matches_target(client, message):
            continue
        message_type = message.get_type()
        if message_type == "MISSION_ACK":
            result = int(message.type)
            if result != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                raise MissionInstallError(f"mission upload rejected with ACK {result}")
            if served != expected_sequences:
                missing = sorted(expected_sequences - served)
                raise MissionInstallError(
                    f"mission ACK arrived before all items were requested; missing={missing}"
                )
            return

        sequence = int(message.seq)
        if sequence not in expected_sequences:
            raise MissionInstallError(
                f"flight controller requested invalid mission sequence {sequence}"
            )
        item = items[sequence]
        if message_type == "MISSION_REQUEST_INT":
            _send_item_int(client, item)
        else:
            _send_item_float(client, item)
        served.add(sequence)
    raise MavlinkTimeout("full mission upload timed out before accepted ACK")


def _close_client(client: MavlinkClient) -> None:
    if client.connection is not None:
        client.connection.close()
        client.connection = None


def install(args: argparse.Namespace) -> Path:
    config_path = args.config.expanduser().resolve(strict=True)
    config = load_config(config_path)
    mission_path, mission_hash = require_exact_pin(
        config,
        args.mission,
        args.confirm_sha256,
    )
    reference = load_qgc_waypoints(mission_path)
    contract, validation_errors = validate_mission(
        reference,
        payload_servo=config.release.servo_channel,
        min_release_pwm=float(config.release.release_pwm),
    )
    if contract is None or validation_errors:
        raise MissionInstallError(
            "pinned mission failed safety validation: " + "; ".join(validation_errors)
        )
    physical_mission = physical_install_mission(
        reference,
        contract,
        release_enabled=config.release.enabled,
    )
    # Close the small time-of-check/time-of-use window before any connection.
    if sha256_bytes(mission_path.read_bytes()) != mission_hash:
        raise MissionInstallError("mission file changed after validation")

    runtime_dir = config.runtime.path()
    lock_stream = acquire_install_lock(runtime_dir)
    client = MavlinkClient(
        endpoint=config.flight_controller.endpoint,
        baud=config.flight_controller.baud,
    )
    try:
        require_main_service_stopped()
        client.connect(timeout=config.flight_controller.heartbeat_timeout_s)
        _validate_autopilot_heartbeat(client, client.initial_heartbeat)
        require_fresh_disarmed_heartbeat(
            client,
            min(5.0, config.flight_controller.heartbeat_timeout_s),
        )

        previous = client.download_mission(timeout=args.timeout)
        backup_path, backup_hash = save_timestamped_backup(
            runtime_dir,
            previous,
            client.target_system,
        )
        print(
            f"[CUADC] Existing mission backed up: {backup_path} "
            f"sha256={backup_hash} items={len(previous)}"
        )

        # Re-check every mutable safety precondition immediately before clear.
        require_main_service_stopped()
        require_fresh_disarmed_heartbeat(client, min(5.0, args.timeout))
        if sha256_bytes(mission_path.read_bytes()) != mission_hash:
            raise MissionInstallError("mission file changed before upload")

        upload_full_mission(client, physical_mission, timeout=args.timeout)
        require_fresh_disarmed_heartbeat(client, min(5.0, args.timeout))
        readback = client.download_mission(timeout=args.timeout)
        physical_errors, physical_report = compare_mission_reference(
            readback,
            physical_mission,
            max_position_error_m=config.mission.max_position_error_m,
        )
        if physical_errors:
            raise MissionInstallError(
                "flight-controller physical mission readback mismatch: "
                + "; ".join(physical_errors)
            )
        semantic_readback = semantic_install_readback(
            readback,
            reference,
            contract,
            release_enabled=config.release.enabled,
        )
        semantic_errors, semantic_report = compare_mission_reference(
            semantic_readback,
            reference,
            max_position_error_m=config.mission.max_position_error_m,
        )
        if semantic_errors:
            raise MissionInstallError(
                "flight-controller semantic mission readback mismatch: "
                + "; ".join(semantic_errors)
            )
        verified_contract, verified_contract_errors = validate_mission(
            semantic_readback,
            payload_servo=config.release.servo_channel,
            min_release_pwm=float(config.release.release_pwm),
        )
        if verified_contract != contract or verified_contract_errors:
            raise MissionInstallError(
                "semantic readback no longer satisfies the approved mission contract: "
                + "; ".join(verified_contract_errors)
            )
        archived_patch = archive_stale_patch_backup(runtime_dir)
        if archived_patch is not None:
            archived_path, archived_hash = archived_patch
            print(
                f"[CUADC] Stale dynamic-patch backup archived: {archived_path} "
                f"sha256={archived_hash}"
            )
        release_seq = contract.fallback_release_seq
        release_command = next(
            item.command for item in physical_mission if item.seq == release_seq
        )
        print(
            f"[CUADC] Mission installed and read back successfully: "
            f"{mission_path} sha256={mission_hash} items={len(readback)} "
            f"mode=ACTIVE physical_release_seq={release_seq} "
            f"physical_release_command={release_command} "
            f"physical_max_position_error_m={physical_report['max_position_error_m']} "
            f"semantic_max_position_error_m={semantic_report['max_position_error_m']}"
        )
        return backup_path
    finally:
        try:
            _close_client(client)
        finally:
            fcntl.flock(lock_stream.fileno(), fcntl.LOCK_UN)
            lock_stream.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Install the exact hash-pinned mission on a disarmed flight controller",
        epilog=(
            "SAFETY: first stop cuadc-onboard.service, keep "
            "cuadc-onboard-router.service running, and keep the aircraft disarmed. "
            "Restart the main service only after this tool reports a successful readback."
        ),
    )
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--mission", required=True, type=Path)
    parser.add_argument(
        "--confirm-sha256",
        required=True,
        help="exact lowercase SHA-256 pinned by mission.expected_sha256",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=30.0,
        help="timeout in seconds for each mission transfer (default: 30)",
    )
    return parser


def main(argv: Iterable[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if not math.isfinite(args.timeout) or args.timeout <= 0.0:
        raise SystemExit("--timeout must be positive")
    try:
        install(args)
    except (MissionInstallError, MavlinkTimeout, OSError, ValueError) as exc:
        print(f"[CUADC] Mission installation refused/failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
