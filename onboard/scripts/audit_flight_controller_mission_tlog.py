#!/usr/bin/env python3
"""Audit the flight-controller mission from the router TLOG, without I/O to it.

This program only opens regular files.  It deliberately has no endpoint,
serial-port or UDP options and never calls a MAVLink send method.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass, is_dataclass, replace
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time
from typing import Any, Sequence


APP_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = APP_ROOT / "src"
if SOURCE_ROOT.is_dir() and str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))
if (APP_ROOT / "config/onboard.yaml").is_file():
    os.environ.setdefault("CUADC_ONBOARD_ROOT", str(APP_ROOT))

from pymavlink import mavutil  # noqa: E402

from cuadc_onboard.config import DEFAULT_CONFIG, OnboardConfig, load_config  # noqa: E402
from cuadc_onboard.mission import (  # noqa: E402
    DO_SET_SERVO,
    MissionItem,
    compare_mission_reference,
    load_patch_backup,
    load_qgc_waypoints,
    validate_mission,
)


AUTOPILOT_COMPONENT_ID = mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
COMPANION_COMPONENT_ID = mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER
MISSION_TYPE = mavutil.mavlink.MAV_MISSION_TYPE_MISSION
NAV_DELAY = mavutil.mavlink.MAV_CMD_NAV_DELAY
ARMED_FLAG = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED


@dataclass(frozen=True)
class MissionDownload:
    source_system: int
    source_component: int
    target_system: int
    target_component: int
    count: int
    started_at: float
    completed_at: float
    items: tuple[MissionItem, ...]


@dataclass(frozen=True)
class HeartbeatRecord:
    source_system: int
    source_component: int
    observed_at: float
    base_mode: int
    custom_mode: int
    vehicle_type: int
    autopilot: int
    system_status: int
    mode: str

    @property
    def armed(self) -> bool:
        return bool(self.base_mode & ARMED_FLAG)


@dataclass
class _PendingDownload:
    source_system: int
    source_component: int
    target_system: int
    target_component: int
    count: int
    started_at: float
    items: dict[int, MissionItem]


class MissionDownloadTracker:
    """Collect complete FC-to-companion mission download responses."""

    def __init__(self, target_component: int = COMPANION_COMPONENT_ID):
        self.target_component = int(target_component)
        self.pending: _PendingDownload | None = None
        self.latest_complete: MissionDownload | None = None
        self.heartbeats: dict[tuple[int, int], HeartbeatRecord] = {}

    def observe(self, message: Any) -> None:
        message_type = message.get_type()
        if message_type == "BAD_DATA":
            return
        try:
            source_system = int(message.get_srcSystem())
            source_component = int(message.get_srcComponent())
        except (AttributeError, TypeError, ValueError):
            return
        observed_at = _message_timestamp(message)

        if (
            message_type == "HEARTBEAT"
            and source_component == AUTOPILOT_COMPONENT_ID
        ):
            self.heartbeats[(source_system, source_component)] = HeartbeatRecord(
                source_system=source_system,
                source_component=source_component,
                observed_at=observed_at,
                base_mode=int(getattr(message, "base_mode", 0)),
                custom_mode=int(getattr(message, "custom_mode", 0)),
                vehicle_type=int(getattr(message, "type", -1)),
                autopilot=int(getattr(message, "autopilot", -1)),
                system_status=int(getattr(message, "system_status", -1)),
                mode=_mode_name(message),
            )
            return

        if (
            message_type == "MISSION_COUNT"
            and source_component == AUTOPILOT_COMPONENT_ID
            and int(getattr(message, "target_component", -1))
            == self.target_component
            and _mission_type(message) == MISSION_TYPE
        ):
            count = int(getattr(message, "count", -1))
            if count < 0:
                self.pending = None
                return
            self.pending = _PendingDownload(
                source_system=source_system,
                source_component=source_component,
                target_system=int(getattr(message, "target_system", 0)),
                target_component=self.target_component,
                count=count,
                started_at=observed_at,
                items={},
            )
            if count == 0:
                self._complete(observed_at)
            return

        if message_type not in {"MISSION_ITEM", "MISSION_ITEM_INT"}:
            return
        pending = self.pending
        if pending is None:
            return
        if (source_system, source_component) != (
            pending.source_system,
            pending.source_component,
        ):
            return
        if (
            int(getattr(message, "target_component", -1))
            != pending.target_component
            or int(getattr(message, "target_system", -1))
            != pending.target_system
            or _mission_type(message) != MISSION_TYPE
        ):
            return
        try:
            item = MissionItem.from_message(message)
        except (AttributeError, TypeError, ValueError, OverflowError):
            return
        if not 0 <= item.seq < pending.count:
            return
        pending.items[item.seq] = item
        if len(pending.items) == pending.count:
            self._complete(observed_at)

    def heartbeat_for(self, source_system: int) -> HeartbeatRecord | None:
        return self.heartbeats.get((source_system, AUTOPILOT_COMPONENT_ID))

    def _complete(self, completed_at: float) -> None:
        assert self.pending is not None
        pending = self.pending
        self.latest_complete = MissionDownload(
            source_system=pending.source_system,
            source_component=pending.source_component,
            target_system=pending.target_system,
            target_component=pending.target_component,
            count=pending.count,
            started_at=pending.started_at,
            completed_at=completed_at,
            items=tuple(pending.items[seq] for seq in range(pending.count)),
        )
        self.pending = None


def _message_timestamp(message: Any) -> float:
    try:
        value = float(getattr(message, "_timestamp", 0.0) or 0.0)
    except (TypeError, ValueError, OverflowError):
        return 0.0
    return value if math.isfinite(value) else 0.0


def _mission_type(message: Any) -> int:
    # MAVLink 1 packets may not carry the MAVLink 2 extension field.  In that
    # case the protocol-defined default is the normal flight mission.
    try:
        return int(getattr(message, "mission_type", MISSION_TYPE))
    except (TypeError, ValueError):
        return -1


def _mode_name(message: Any) -> str:
    try:
        value = str(mavutil.mode_string_v10(message))
        if value and value != "Mode(0x00000000)":
            return value
    except (AttributeError, KeyError, TypeError, ValueError):
        pass
    return f"CUSTOM_MODE_{int(getattr(message, 'custom_mode', 0))}"


def _iso8601(timestamp: float | None) -> str | None:
    if timestamp is None or not math.isfinite(timestamp) or timestamp <= 0.0:
        return None
    return datetime.fromtimestamp(timestamp, timezone.utc).isoformat().replace(
        "+00:00", "Z"
    )


def _regular_file(path: Path) -> Path:
    resolved = path.expanduser().resolve()
    if not resolved.is_file():
        raise ValueError(f"TLOG is not a regular file: {resolved}")
    return resolved


def resolve_tlog(config: OnboardConfig, requested: Path | None) -> Path:
    candidate = requested.expanduser().resolve() if requested is not None else None
    if candidate is not None and candidate.is_file():
        return _regular_file(candidate)
    directory = candidate if candidate is not None else config.runtime.path() / "tlogs"
    if not directory.is_dir():
        raise ValueError(f"TLOG directory does not exist: {directory}")
    logs = [path for path in directory.glob("cuadc-*.tlog") if path.is_file()]
    if not logs and requested is not None:
        logs = [path for path in directory.glob("*.tlog") if path.is_file()]
    if not logs:
        raise ValueError(f"no router TLOG found in: {directory}")
    return _regular_file(max(logs, key=lambda path: path.stat().st_mtime_ns))


def scan_tlog(path: Path) -> tuple[MissionDownload | None, HeartbeatRecord | None]:
    """Read a regular TLOG to EOF.  No transport endpoint is accepted here."""

    path = _regular_file(path)
    tracker = MissionDownloadTracker()
    log = mavutil.mavlink_connection(str(path), robust_parsing=True)
    try:
        while True:
            message = log.recv_match(blocking=False)
            if message is None:
                break
            tracker.observe(message)
    finally:
        log.close()
    cycle = tracker.latest_complete
    heartbeat = tracker.heartbeat_for(cycle.source_system) if cycle else None
    return cycle, heartbeat


def _zero(value: float, tolerance: float = 1e-6) -> bool:
    return math.isfinite(value) and abs(value) <= tolerance


def is_zero_parameter_nav_delay(item: MissionItem) -> bool:
    return item.command == NAV_DELAY and all(
        _zero(value)
        for value in (
            item.param1,
            item.param2,
            item.param3,
            item.param4,
            item.x,
            item.y,
            item.z,
        )
    )


def shadow_physical_slot_errors(
    actual: Sequence[MissionItem], expected: Sequence[MissionItem]
) -> list[str]:
    """Require a legacy physical FC release slot to be a strict no-op."""

    actual_by_seq = {item.seq: item for item in actual}
    errors: list[str] = []
    for expected_item in expected:
        if expected_item.command != DO_SET_SERVO:
            continue
        live_item = actual_by_seq.get(expected_item.seq)
        if live_item is None:
            errors.append(
                f"shadow physical release slot seq {expected_item.seq} is missing"
            )
        elif not (
            is_zero_parameter_nav_delay(live_item)
            and live_item.current == 0
            and live_item.autocontinue == 1
        ):
            errors.append(
                f"shadow physical release slot seq {expected_item.seq} must be "
                "a zero-parameter/position NAV_DELAY with current=0 and "
                "autocontinue=1"
            )
    return errors


def restore_shadow_semantics(
    actual: Sequence[MissionItem],
    expected: Sequence[MissionItem],
    backup: Sequence[MissionItem],
    max_position_error_m: float,
    *,
    enabled: bool,
) -> tuple[list[MissionItem], list[dict[str, Any]], list[str]]:
    """Restore only a proven shadow-neutralized release item for comparison."""

    restored = list(actual)
    restorations: list[dict[str, Any]] = []
    errors: list[str] = []
    if enabled:
        return restored, restorations, errors
    expected_by_seq = {item.seq: item for item in expected}
    backup_by_seq = {item.seq: item for item in backup}
    for index, live_item in enumerate(restored):
        expected_item = expected_by_seq.get(live_item.seq)
        if (
            expected_item is None
            or expected_item.command != DO_SET_SERVO
            or not is_zero_parameter_nav_delay(live_item)
        ):
            continue
        backup_item = backup_by_seq.get(live_item.seq)
        if backup_item is None:
            errors.append(
                f"shadow NAV_DELAY at seq {live_item.seq} has no backup entry"
            )
            continue
        backup_errors, _ = compare_mission_reference(
            [backup_item],
            [expected_item],
            max_position_error_m=max_position_error_m,
        )
        if backup_errors:
            errors.append(
                f"shadow NAV_DELAY backup at seq {live_item.seq} does not match "
                f"the pinned reference: {'; '.join(backup_errors)}"
            )
            continue
        restored[index] = replace(backup_item, seq=live_item.seq)
        restorations.append(
            {
                "seq": live_item.seq,
                "live_command": live_item.command,
                "semantic_command": backup_item.command,
                "reason": "zero-parameter NAV_DELAY matched pinned backup",
            }
        )
    return restored, restorations, errors


def mission_sha256(items: Sequence[MissionItem]) -> str:
    def number(value: float) -> str | None:
        return None if not math.isfinite(value) else format(value, ".17g")

    payload = [
        {
            "seq": item.seq,
            "frame": item.frame,
            "command": item.command,
            "current": item.current,
            "autocontinue": item.autocontinue,
            "param1": number(item.param1),
            "param2": number(item.param2),
            "param3": number(item.param3),
            "param4": number(item.param4),
            "x": number(item.x),
            "y": number(item.y),
            "z": number(item.z),
        }
        for item in items
    ]
    encoded = json.dumps(
        payload, ensure_ascii=True, sort_keys=True, separators=(",", ":")
    ).encode("ascii")
    return hashlib.sha256(encoded).hexdigest()


def _jsonable(value: Any) -> Any:
    if is_dataclass(value):
        return _jsonable(asdict(value))
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def audit(
    config: OnboardConfig,
    tlog_path: Path,
    max_age_s: float,
    *,
    now: float | None = None,
) -> dict[str, Any]:
    errors: list[str] = []
    cycle, heartbeat = scan_tlog(tlog_path)
    # A live router TLOG can keep growing while a large file is scanned.  Use
    # the time at completion, otherwise messages appended during the scan can
    # be incorrectly reported as several seconds "in the future".
    now = time.time() if now is None else float(now)
    expected_path = config.mission.expected_path()
    if expected_path is None:
        raise ValueError("mission.expected_file is required for TLOG audit")
    expected_bytes = expected_path.read_bytes()
    expected_file_hash = hashlib.sha256(expected_bytes).hexdigest()
    configured_hash = config.mission.expected_sha256.lower()
    hash_matches = expected_file_hash == configured_hash
    if not hash_matches:
        errors.append(
            "pinned mission SHA-256 mismatch: "
            f"configured={configured_hash}, actual={expected_file_hash}"
        )
    expected = load_qgc_waypoints(expected_path)

    result: dict[str, Any] = {
        "schema": "cuadc-flight-controller-mission-tlog-audit.v1",
        "ok": False,
        "audited_at": _iso8601(now),
        "tlog": str(tlog_path),
        "max_age_s": max_age_s,
        "mission_observed_at": None,
        "age_s": None,
        "fresh": False,
        "count": None,
        "hash": {
            "expected_file": str(expected_path),
            "configured_sha256": configured_hash,
            "computed_sha256": expected_file_hash,
            "matches": hash_matches,
            "live_raw_mission_sha256": None,
            "live_semantic_mission_sha256": None,
        },
        "armed": None,
        "mode": None,
        "heartbeat": None,
        "transaction": None,
        "reference": None,
        "contract": None,
        "shadow_semantic_restorations": [],
        "errors": errors,
    }
    if cycle is None:
        errors.append(
            "no complete FC-to-companion MISSION_COUNT + seq 0..N-1 "
            "download cycle found"
        )
        return result

    age_s = max(0.0, now - cycle.completed_at)
    timestamp_missing = cycle.completed_at <= 0.0
    timestamp_in_future = cycle.completed_at > now + 5.0
    fresh = (
        not timestamp_missing
        and not timestamp_in_future
        and age_s <= max_age_s
    )
    result["count"] = cycle.count
    result["mission_observed_at"] = _iso8601(cycle.completed_at)
    result["age_s"] = round(age_s, 3)
    result["fresh"] = fresh
    result["transaction"] = {
        "source_system": cycle.source_system,
        "source_component": cycle.source_component,
        "target_system": cycle.target_system,
        "target_component": cycle.target_component,
        "started_at": _iso8601(cycle.started_at),
        "completed_at": _iso8601(cycle.completed_at),
        "completed_at_unix": cycle.completed_at,
        "age_s": round(age_s, 3),
        "fresh": fresh,
    }
    if timestamp_missing:
        errors.append("mission download has no valid TLOG timestamp")
    elif timestamp_in_future:
        errors.append("mission download timestamp is more than 5 s in the future")
    elif not fresh:
        errors.append(
            f"mission download is stale: age={age_s:.3f}s, max={max_age_s:.3f}s"
        )

    if heartbeat is None:
        errors.append("no flight-controller HEARTBEAT found for the mission SYSID")
    else:
        heartbeat_age_s = max(0.0, now - heartbeat.observed_at)
        result["armed"] = heartbeat.armed
        result["mode"] = heartbeat.mode
        result["heartbeat"] = {
            "source_system": heartbeat.source_system,
            "source_component": heartbeat.source_component,
            "observed_at": _iso8601(heartbeat.observed_at),
            "observed_at_unix": heartbeat.observed_at,
            "age_s": round(heartbeat_age_s, 3),
            "base_mode": heartbeat.base_mode,
            "custom_mode": heartbeat.custom_mode,
            "vehicle_type": heartbeat.vehicle_type,
            "autopilot": heartbeat.autopilot,
            "system_status": heartbeat.system_status,
        }

    backup_path = config.runtime.path() / "mission_patch_backup.json"
    backup = load_patch_backup(backup_path)
    if not config.release.enabled:
        errors.extend(shadow_physical_slot_errors(cycle.items, expected))
    semantic_actual, restorations, restoration_errors = restore_shadow_semantics(
        cycle.items,
        expected,
        backup,
        config.mission.max_position_error_m,
        enabled=config.release.enabled,
    )
    errors.extend(restoration_errors)
    reference_errors, reference_report = compare_mission_reference(
        semantic_actual,
        expected,
        max_position_error_m=config.mission.max_position_error_m,
    )
    errors.extend(f"reference: {error}" for error in reference_errors)
    contract, contract_errors = validate_mission(
        semantic_actual,
        payload_servo=config.release.servo_channel,
    )
    errors.extend(f"contract: {error}" for error in contract_errors)

    result["hash"]["live_raw_mission_sha256"] = mission_sha256(cycle.items)
    result["hash"]["live_semantic_mission_sha256"] = mission_sha256(
        semantic_actual
    )
    result["reference"] = _jsonable(reference_report)
    result["contract"] = _jsonable(contract)
    result["shadow_semantic_restorations"] = restorations
    result["backup"] = {
        "path": str(backup_path),
        "found": backup_path.is_file(),
        "item_count": len(backup),
    }
    result["ok"] = not errors
    return result


def _positive_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise argparse.ArgumentTypeError("must be a positive finite number")
    return parsed


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Read-only audit of the last complete FC mission download in a "
            "Raspberry Pi router TLOG"
        )
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument(
        "--tlog",
        type=Path,
        help=(
            "router .tlog file or directory; defaults to the newest "
            "runtime/tlogs/cuadc-*.tlog"
        ),
    )
    parser.add_argument("--max-age", type=_positive_float, default=30.0)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_argument_parser().parse_args(argv)
    try:
        config = load_config(args.config.expanduser().resolve())
        tlog_path = resolve_tlog(config, args.tlog)
        result = audit(config, tlog_path, args.max_age)
    except Exception as exc:
        result = {
            "schema": "cuadc-flight-controller-mission-tlog-audit.v1",
            "ok": False,
            "audited_at": _iso8601(time.time()),
            "tlog": str(args.tlog) if args.tlog is not None else None,
            "max_age_s": args.max_age,
            "mission_observed_at": None,
            "age_s": None,
            "fresh": False,
            "count": None,
            "hash": None,
            "armed": None,
            "mode": None,
            "heartbeat": None,
            "transaction": None,
            "reference": None,
            "contract": None,
            "errors": [f"{type(exc).__name__}: {exc}"],
        }
    print(json.dumps(_jsonable(result), ensure_ascii=False, indent=2))
    return 0 if result.get("ok") is True else 1


if __name__ == "__main__":
    raise SystemExit(main())
