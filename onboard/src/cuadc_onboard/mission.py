from __future__ import annotations

import math
import hashlib
import json
import os
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Iterable, Sequence

from pymavlink import mavutil

from .config import (
    ORIGIN_LAT_DEG,
    ORIGIN_LON_DEG,
    PAYLOAD_RELEASE_PWM,
    PAYLOAD_SERVO,
    SCAN_END_TAG,
    SCAN_START_TAG,
)


JUMP_TAG = mavutil.mavlink.MAV_CMD_JUMP_TAG
DO_SET_SERVO = mavutil.mavlink.MAV_CMD_DO_SET_SERVO
NAV_WAYPOINT = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
NAV_DELAY = mavutil.mavlink.MAV_CMD_NAV_DELAY
NAV_LOITER_TURNS = mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS
NAV_LOITER_TO_ALT = mavutil.mavlink.MAV_CMD_NAV_LOITER_TO_ALT
NAV_LAND = mavutil.mavlink.MAV_CMD_NAV_LAND
GLOBAL_RELATIVE_ALT_INT = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT

EARTH_RADIUS_M = 6378137.0
GRAVITY_MPS2 = 9.80665
MAX_HOME_DISTANCE_M = 300.0
ROUTE_PLANNING_LIMIT_M = 288.0
MAX_BANK_DEG = 30.0
MIN_LOITER_RADIUS_M = 35.0
MIN_ATTACK_STRAIGHT_M = 70.0

MISSION_MODE_PASSTHROUGH = "PASSTHROUGH"
MISSION_MODE_PAYLOAD = "PAYLOAD"
MISSION_MODE_RECON_ONLY = "RECON_ONLY"
MISSION_MODE_CONTRACT_WARNING = "CONTRACT_WARNING"
MISSION_MODE_DYNAMIC_READY = "DYNAMIC_READY"
MISSION_MODE_FIXED_FALLBACK = "FIXED_FALLBACK"
MISSION_MODE_BLOCKED = "BLOCKED"


@dataclass(frozen=True)
class MissionItem:
    seq: int
    frame: int
    command: int
    current: int = 0
    autocontinue: int = 1
    param1: float = 0.0
    param2: float = 0.0
    param3: float = 0.0
    param4: float = math.nan
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    @classmethod
    def from_message(cls, message) -> "MissionItem":
        message_type = message.get_type()
        x = float(message.x)
        y = float(message.y)
        if message_type == "MISSION_ITEM_INT":
            x /= 1e7
            y /= 1e7
        return cls(
            seq=int(message.seq),
            frame=int(message.frame),
            command=int(message.command),
            current=int(message.current),
            autocontinue=int(message.autocontinue),
            param1=float(message.param1),
            param2=float(message.param2),
            param3=float(message.param3),
            param4=float(message.param4),
            x=x,
            y=y,
            z=float(message.z),
        )


@dataclass(frozen=True)
class MissionContract:
    scan_start_seq: int
    scan_end_seq: int
    decision_seq: int
    fallback_start_seq: int
    fallback_release_seq: int
    fallback_exit_seq: int
    land_seq: int

    @property
    def retry_start_seq(self) -> int:
        return self.scan_start_seq + 1


@dataclass(frozen=True)
class ReconContract:
    """A tagged scan window that can select/log targets but never actuate."""

    scan_start_seq: int
    scan_end_seq: int
    decision_seq: int
    land_seq: int


@dataclass(frozen=True)
class FixedFallbackContract:
    """Minimum executable mission that always reaches a fixed release."""

    scan_start_seq: int
    scan_end_seq: int
    retry_start_seq: int
    fallback_release_seq: int
    land_seq: int


@dataclass(frozen=True)
class ScanRectangle:
    """Two opposite scan lanes that bound an adjustable reconnaissance box.

    Mission Planner remains the source of truth.  The first two and last two
    NAV_WAYPOINT items between JUMP_TAG 9101/9102 are treated as the two long
    sides of the box.  Any waypoints between those pairs are outside-the-box
    turn helpers and are deliberately excluded from coverage accounting.
    """

    lane1_start_seq: int
    lane1_end_seq: int
    lane2_start_seq: int
    lane2_end_seq: int
    corner_latlon: tuple[tuple[float, float], ...]
    turn_waypoint_seqs: tuple[int, ...]
    length_m: float
    width_m: float
    heading_deg: float


@dataclass(frozen=True)
class MissionAssessment:
    mode: str
    contract: MissionContract | FixedFallbackContract | None
    errors: tuple[str, ...] = ()
    warnings: tuple[str, ...] = ()

    @property
    def launch_allowed(self) -> bool:
        return self.mode != MISSION_MODE_BLOCKED and self.contract is not None


def validate_route_safety(
    items: Sequence[MissionItem],
    origin_lat_deg: float | None = None,
    origin_lon_deg: float | None = None,
    max_home_distance_m: float | None = None,
) -> list[str]:
    """Validate finite positions and an optional configured home boundary."""

    errors: list[str] = []
    position_commands = {
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        NAV_WAYPOINT,
        NAV_LOITER_TURNS,
        NAV_LOITER_TO_ALT,
        NAV_LAND,
        mavutil.mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT,
    }
    for item in items:
        if item.command not in position_commands:
            continue
        if item.x == 0.0 and item.y == 0.0 and (
            item.seq == 0
            or item.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
        ):
            # Mission Planner may leave HOME empty until ArduPlane supplies it.
            continue
        if not all(math.isfinite(value) for value in (item.x, item.y, item.z)):
            errors.append(f"mission sequence {item.seq} has non-finite coordinates")
            continue
        if not (-90.0 <= item.x <= 90.0 and -180.0 <= item.y <= 180.0):
            errors.append(f"mission sequence {item.seq} has invalid latitude/longitude")
            continue
        if item.x == 0.0 and item.y == 0.0:
            errors.append(f"mission sequence {item.seq} has an empty position")
            continue
        if (
            origin_lat_deg is not None
            and origin_lon_deg is not None
            and max_home_distance_m is not None
        ):
            north_m, east_m = latlon_to_ne(
                item.x, item.y, origin_lat_deg, origin_lon_deg
            )
            extent = math.hypot(north_m, east_m)
            if item.command == NAV_LOITER_TURNS:
                extent += abs(item.param3)
            if extent > max_home_distance_m:
                errors.append(
                    f"mission sequence {item.seq} exceeds the "
                    f"{max_home_distance_m:.0f} m home boundary"
                )
    return errors


def mission_fingerprint(items: Sequence[MissionItem]) -> str:
    """Return a stable identity for a flight-controller mission readback.

    The transient ``current`` flag is intentionally excluded. Coordinates are
    rounded well below the configured two-metre route tolerance so repeated
    downloads of the same EEPROM mission remain stable while real route edits
    still produce a new identity.
    """

    def number(value: float) -> float | None:
        if math.isnan(value):
            return None
        return round(float(value), 7)

    payload = [
        (
            item.seq,
            item.frame,
            item.command,
            item.autocontinue,
            number(item.param1),
            number(item.param2),
            number(item.param3),
            number(item.param4),
            number(item.x),
            number(item.y),
            number(item.z),
        )
        for item in items
    ]
    encoded = json.dumps(payload, separators=(",", ":"), ensure_ascii=True).encode()
    return hashlib.sha256(encoded).hexdigest()


POSITION_COMMANDS = {
    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
    mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS,
    mavutil.mavlink.MAV_CMD_NAV_LOITER_TO_ALT,
    mavutil.mavlink.MAV_CMD_NAV_LAND,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    mavutil.mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT,
    mavutil.mavlink.MAV_CMD_NAV_VTOL_TAKEOFF,
    mavutil.mavlink.MAV_CMD_NAV_VTOL_LAND,
}


def load_qgc_waypoints(path: Path) -> list[MissionItem]:
    """Load a Mission Planner/QGroundControl ``QGC WPL 110`` file.

    Sequence numbers are deliberately required to be contiguous.  This keeps
    the approved file suitable for an exact, auditable flight-controller
    upload instead of silently resequencing a malformed plan.
    """

    try:
        lines = path.read_text(encoding="utf-8-sig").splitlines()
    except OSError as exc:
        raise ValueError(f"cannot read mission file {path}: {exc}") from exc
    if not lines or lines[0].strip() != "QGC WPL 110":
        raise ValueError(f"mission file {path} is not QGC WPL 110")
    items: list[MissionItem] = []
    for line_number, raw_line in enumerate(lines[1:], start=2):
        line = raw_line.strip()
        if not line:
            continue
        fields = line.split()
        if len(fields) != 12:
            raise ValueError(
                f"mission line {line_number} needs 12 fields, got {len(fields)}"
            )
        try:
            item = MissionItem(
                seq=int(fields[0]),
                current=int(fields[1]),
                frame=int(fields[2]),
                command=int(fields[3]),
                param1=float(fields[4]),
                param2=float(fields[5]),
                param3=float(fields[6]),
                param4=float(fields[7]),
                x=float(fields[8]),
                y=float(fields[9]),
                z=float(fields[10]),
                autocontinue=int(fields[11]),
            )
        except ValueError as exc:
            raise ValueError(
                f"mission line {line_number} contains an invalid number"
            ) from exc
        if item.seq != len(items):
            raise ValueError(
                f"mission sequence must be contiguous: expected {len(items)}, "
                f"got {item.seq} on line {line_number}"
            )
        items.append(item)
    if not items:
        raise ValueError("mission file contains no mission items")
    return items


def _horizontal_distance_m(first: MissionItem, second: MissionItem) -> float:
    mean_latitude = math.radians((first.x + second.x) * 0.5)
    north_m = math.radians(first.x - second.x) * EARTH_RADIUS_M
    east_m = (
        math.radians(first.y - second.y)
        * EARTH_RADIUS_M
        * math.cos(mean_latitude)
    )
    return math.hypot(north_m, east_m)


def _parameter_matches(actual: float, expected: float, tolerance: float) -> bool:
    if math.isnan(actual) and math.isnan(expected):
        return True
    if not math.isfinite(actual) or not math.isfinite(expected):
        return actual == expected
    return abs(actual - expected) <= tolerance


def compare_mission_reference(
    actual_items: Sequence[MissionItem],
    expected_items: Sequence[MissionItem],
    max_position_error_m: float = 2.0,
) -> tuple[list[str], dict]:
    """Compare a flight-controller readback with a hash-pinned mission.

    ArduPilot legitimately normalises a few fields while storing a mission:
    HOME coordinates/current, frames on non-positional commands, integer
    loiter radius and LAND heading.  Those documented normalisations are
    accepted; commands, sequence, all real waypoints and meaningful
    parameters remain strict.
    """

    if max_position_error_m <= 0.0:
        raise ValueError("max_position_error_m must be positive")
    errors: list[str] = []
    actual_by_seq = {item.seq: item for item in actual_items}
    expected_by_seq = {item.seq: item for item in expected_items}
    if len(actual_items) != len(expected_items):
        errors.append(
            "mission item count differs: "
            f"actual={len(actual_items)}, expected={len(expected_items)}"
        )
    if len(actual_by_seq) != len(actual_items):
        errors.append("actual mission contains duplicate sequence numbers")
    if len(expected_by_seq) != len(expected_items):
        errors.append("expected mission contains duplicate sequence numbers")
    missing = sorted(set(expected_by_seq) - set(actual_by_seq))
    extra = sorted(set(actual_by_seq) - set(expected_by_seq))
    if missing:
        errors.append(f"mission is missing sequences {missing}")
    if extra:
        errors.append(f"mission has unexpected sequences {extra}")

    position_errors: dict[str, float] = {}
    normalizations: list[str] = []
    max_seen_position_error_m = 0.0
    for seq in sorted(set(actual_by_seq) & set(expected_by_seq)):
        actual = actual_by_seq[seq]
        expected = expected_by_seq[seq]
        if actual.command != expected.command:
            errors.append(
                f"seq {seq} command differs: actual={actual.command}, "
                f"expected={expected.command}"
            )
            continue
        if actual.autocontinue != expected.autocontinue:
            errors.append(
                f"seq {seq} autocontinue differs: actual={actual.autocontinue}, "
                f"expected={expected.autocontinue}"
            )

        is_home = seq == 0
        is_position = actual.command in POSITION_COMMANDS and not is_home
        if is_position:
            # MAV_FRAME_*_INT and their non-INT counterparts have identical
            # coordinate semantics for a MISSION_ITEM_INT transaction.
            equivalent_frames = {0: 5, 3: 6, 10: 11}
            actual_frame = equivalent_frames.get(actual.frame, actual.frame)
            expected_frame = equivalent_frames.get(expected.frame, expected.frame)
            if actual_frame != expected_frame:
                errors.append(
                    f"seq {seq} frame differs: actual={actual.frame}, "
                    f"expected={expected.frame}"
                )
            if not all(
                math.isfinite(value)
                for value in (actual.x, actual.y, expected.x, expected.y)
            ):
                errors.append(f"seq {seq} contains a non-finite position")
            else:
                position_error = _horizontal_distance_m(actual, expected)
                position_errors[str(seq)] = round(position_error, 3)
                max_seen_position_error_m = max(
                    max_seen_position_error_m, position_error
                )
                if position_error > max_position_error_m:
                    errors.append(
                        f"seq {seq} position differs by {position_error:.2f} m "
                        f"(limit {max_position_error_m:.2f} m)"
                    )
            if not _parameter_matches(actual.z, expected.z, 0.25):
                errors.append(
                    f"seq {seq} altitude differs: actual={actual.z:.3f}, "
                    f"expected={expected.z:.3f}"
                )
        elif actual.frame != expected.frame:
            normalizations.append(
                f"seq {seq} non-position frame {expected.frame}->{actual.frame}"
            )

        parameter_tolerances = [0.05, 0.05, 0.05, 0.05]
        if actual.command == NAV_LOITER_TURNS:
            parameter_tolerances[2] = 0.75
        for parameter_index, (actual_value, expected_value, tolerance) in enumerate(
            zip(
                (actual.param1, actual.param2, actual.param3, actual.param4),
                (expected.param1, expected.param2, expected.param3, expected.param4),
                parameter_tolerances,
            ),
            start=1,
        ):
            if actual.command == NAV_LAND and parameter_index == 4:
                if not _parameter_matches(actual_value, expected_value, tolerance):
                    normalizations.append(
                        f"seq {seq} LAND param4 {expected_value}->{actual_value}"
                    )
                continue
            if actual.command == JUMP_TAG and parameter_index == 1:
                matches = int(round(actual_value)) == int(round(expected_value))
            else:
                matches = _parameter_matches(actual_value, expected_value, tolerance)
            if not matches:
                errors.append(
                    f"seq {seq} param{parameter_index} differs: "
                    f"actual={actual_value}, expected={expected_value}"
                )

    report = {
        "matched": not errors,
        "actual_count": len(actual_items),
        "expected_count": len(expected_items),
        "max_position_error_m": round(max_seen_position_error_m, 3),
        "position_errors_m": position_errors,
        "accepted_normalizations": normalizations,
    }
    return errors, report


def is_shadow_neutralization(
    actual: MissionItem,
    expected: MissionItem,
) -> bool:
    """Return whether ``actual`` is the safe NAV_DELAY form of a release slot."""

    if (
        actual.seq != expected.seq
        or expected.command != DO_SET_SERVO
        or actual.command != NAV_DELAY
        or actual.autocontinue != expected.autocontinue
    ):
        return False
    return all(
        math.isfinite(value) and abs(value) <= 1e-4
        for value in (
            actual.param1,
            actual.param2,
            actual.param3,
            actual.param4,
            actual.x,
            actual.y,
            actual.z,
        )
    )


def neutralize_release_item(item: MissionItem) -> MissionItem:
    """Return the exact position-free no-op used by legacy releases."""

    return replace(
        item,
        command=NAV_DELAY,
        param1=0.0,
        param2=0.0,
        param3=0.0,
        param4=0.0,
        x=0.0,
        y=0.0,
        z=0.0,
    )


def validate_mission(
    items: Sequence[MissionItem],
    payload_servo: int = PAYLOAD_SERVO,
    min_release_pwm: float = 1700.0,
) -> tuple[MissionContract | None, list[str]]:
    errors: list[str] = []
    starts = [item.seq for item in items if item.command == JUMP_TAG and int(round(item.param1)) == SCAN_START_TAG]
    ends = [item.seq for item in items if item.command == JUMP_TAG and int(round(item.param1)) == SCAN_END_TAG]
    if len(starts) != 1:
        errors.append(f"mission needs exactly one JUMP_TAG {SCAN_START_TAG}")
    if len(ends) != 1:
        errors.append(f"mission needs exactly one JUMP_TAG {SCAN_END_TAG}")
    if errors:
        return None, errors
    start, end = starts[0], ends[0]
    if start >= end:
        errors.append("SCAN_START must appear before SCAN_END")
        return None, errors

    by_seq = {item.seq: item for item in items}
    decision = by_seq.get(end + 1)
    if decision is None or decision.command != NAV_WAYPOINT:
        errors.append("SCAN_END must be followed by a decision-guard NAV_WAYPOINT")
    pattern = [NAV_LOITER_TURNS, NAV_WAYPOINT, DO_SET_SERVO, NAV_WAYPOINT]
    actual = [by_seq.get(end + offset) for offset in range(2, 6)]
    if any(item is None for item in actual) or [item.command for item in actual if item] != pattern:
        errors.append(
            "SCAN_END must be followed by NAV_LOITER_TURNS, NAV_WAYPOINT, "
            "DO_SET_SERVO, NAV_WAYPOINT fallback slots"
        )
    else:
        if not 0.45 <= actual[0].param1 <= 0.55:
            errors.append("fallback NAV_LOITER_TURNS must request one half turn")
        if not 0 <= abs(actual[0].param3) <= 90.0:
            errors.append("fallback loiter radius must be within 0..90 m")
        if (
            int(round(actual[2].param1)) != payload_servo
            or actual[2].param2 < min_release_pwm
        ):
            errors.append(
                f"fallback DO_SET_SERVO must set servo {payload_servo} "
                f"to at least {min_release_pwm:.0f} PWM"
            )
    lands = [item.seq for item in items if item.command == NAV_LAND and item.seq > end]
    if not lands:
        errors.append("mission needs an automatic NAV_LAND after SCAN_END")
    if errors:
        return None, errors
    return (
        MissionContract(
            scan_start_seq=start,
            scan_end_seq=end,
            decision_seq=end + 1,
            fallback_start_seq=end + 2,
            fallback_release_seq=end + 4,
            fallback_exit_seq=end + 5,
            land_seq=lands[0],
        ),
        [],
    )


def validate_fixed_fallback_mission(
    items: Sequence[MissionItem],
    payload_servo: int = PAYLOAD_SERVO,
    min_release_pwm: float = 1700.0,
) -> tuple[FixedFallbackContract | None, list[str]]:
    """Validate the minimum AUTO scan, fixed release, and landing route."""

    if not items:
        return None, ["mission contains no items"]
    errors: list[str] = []
    starts = [
        item.seq
        for item in items
        if item.command == JUMP_TAG
        and int(round(item.param1)) == SCAN_START_TAG
    ]
    ends = [
        item.seq
        for item in items
        if item.command == JUMP_TAG
        and int(round(item.param1)) == SCAN_END_TAG
    ]
    if len(starts) != 1:
        errors.append(f"mission needs exactly one JUMP_TAG {SCAN_START_TAG}")
    if len(ends) != 1:
        errors.append(f"mission needs exactly one JUMP_TAG {SCAN_END_TAG}")
    if errors:
        return None, errors
    start, end = starts[0], ends[0]
    if start >= end:
        return None, ["SCAN_START must appear before SCAN_END"]

    if not any(item.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF and item.seq < start for item in items):
        errors.append("mission needs an automatic NAV_TAKEOFF before SCAN_START")
    scan_points = [
        item
        for item in items
        if start < item.seq < end and item.command == NAV_WAYPOINT
    ]
    if len(scan_points) < 2:
        errors.append("scan window needs at least two NAV_WAYPOINT items")
    releases = [
        item.seq
        for item in items
        if (
            item.seq > end
            and item.command == DO_SET_SERVO
            and int(round(item.param1)) == payload_servo
            and item.param2 >= min_release_pwm
        )
    ]
    if not releases:
        errors.append(
            f"mission needs a fallback DO_SET_SERVO for servo {payload_servo} "
            f"at or above {min_release_pwm:.0f} PWM after SCAN_END"
        )
    release_seq = releases[0] if releases else math.inf
    lands = [
        item.seq
        for item in items
        if item.command == NAV_LAND and item.seq > release_seq
    ]
    if not lands:
        errors.append("mission needs an automatic NAV_LAND after fallback release")
    if errors:
        return None, errors
    return (
        FixedFallbackContract(
            scan_start_seq=start,
            scan_end_seq=end,
            retry_start_seq=scan_points[0].seq,
            fallback_release_seq=int(release_seq),
            land_seq=lands[0],
        ),
        [],
    )


def assess_mission(
    items: Sequence[MissionItem],
    payload_servo: int = PAYLOAD_SERVO,
    min_release_pwm: float = 1700.0,
    origin_lat_deg: float | None = None,
    origin_lon_deg: float | None = None,
    max_home_distance_m: float | None = None,
) -> MissionAssessment:
    """Return dynamic, fixed-fallback, or blocked mission capability."""

    safety_errors = validate_route_safety(
        items,
        origin_lat_deg=origin_lat_deg,
        origin_lon_deg=origin_lon_deg,
        max_home_distance_m=max_home_distance_m,
    )
    if safety_errors:
        return MissionAssessment(
            MISSION_MODE_BLOCKED,
            None,
            errors=tuple(safety_errors),
        )
    dynamic, dynamic_errors = validate_mission(
        items,
        payload_servo=payload_servo,
        min_release_pwm=min_release_pwm,
    )
    if dynamic is not None and not dynamic_errors:
        return MissionAssessment(MISSION_MODE_DYNAMIC_READY, dynamic)
    fixed, fixed_errors = validate_fixed_fallback_mission(
        items,
        payload_servo=payload_servo,
        min_release_pwm=min_release_pwm,
    )
    if fixed is not None and not fixed_errors:
        return MissionAssessment(
            MISSION_MODE_FIXED_FALLBACK,
            fixed,
            warnings=tuple(dynamic_errors),
        )
    return MissionAssessment(
        MISSION_MODE_BLOCKED,
        None,
        errors=tuple(fixed_errors),
        warnings=tuple(dynamic_errors),
    )


def validate_recon_mission(
    items: Sequence[MissionItem],
) -> tuple[ReconContract | None, list[str]]:
    """Validate a 9101/9102 scan-only route with no payload commands.

    The route deliberately has no fallback/release slots.  It gives the
    companion a bounded scan window and a deterministic selection point while
    remaining safe even if ``release.enabled`` was accidentally left true.
    """

    errors: list[str] = []
    starts = [
        item.seq
        for item in items
        if item.command == JUMP_TAG
        and int(round(item.param1)) == SCAN_START_TAG
    ]
    ends = [
        item.seq
        for item in items
        if item.command == JUMP_TAG
        and int(round(item.param1)) == SCAN_END_TAG
    ]
    if len(starts) != 1:
        errors.append(f"recon mission needs exactly one JUMP_TAG {SCAN_START_TAG}")
    if len(ends) != 1:
        errors.append(f"recon mission needs exactly one JUMP_TAG {SCAN_END_TAG}")
    if errors:
        return None, errors

    start, end = starts[0], ends[0]
    if start >= end:
        return None, ["recon SCAN_START must appear before SCAN_END"]

    by_seq = {item.seq: item for item in items}
    scan_points = [
        item
        for item in items
        if start < item.seq < end and item.command == NAV_WAYPOINT
    ]
    if not scan_points:
        errors.append("recon scan window needs at least one NAV_WAYPOINT")

    decision = by_seq.get(end + 1)
    if decision is None or decision.command != NAV_WAYPOINT:
        errors.append("recon SCAN_END must be followed by a decision NAV_WAYPOINT")

    actuator_commands = {
        mavutil.mavlink.MAV_CMD_DO_SET_RELAY,
        mavutil.mavlink.MAV_CMD_DO_REPEAT_RELAY,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        mavutil.mavlink.MAV_CMD_DO_REPEAT_SERVO,
    }
    found_actuators = sorted(
        item.seq for item in items if item.command in actuator_commands
    )
    if found_actuators:
        errors.append(
            "recon-only mission cannot contain relay/servo commands at "
            f"sequences {found_actuators}"
        )

    lands = [item.seq for item in items if item.command == NAV_LAND and item.seq > end]
    if not lands:
        errors.append("recon mission needs an automatic NAV_LAND after SCAN_END")
    if errors:
        return None, errors
    return (
        ReconContract(
            scan_start_seq=start,
            scan_end_seq=end,
            decision_seq=end + 1,
            land_seq=lands[0],
        ),
        [],
    )


def classify_mission_contract(
    items: Sequence[MissionItem],
    payload_servo: int = PAYLOAD_SERVO,
    min_release_pwm: float = 1700.0,
) -> tuple[str, MissionContract | ReconContract | None, list[str]]:
    """Classify a live route without treating an ordinary route as invalid.

    A mission with neither CUADC tag is a normal ArduPlane mission and the
    companion stays out of its way. If either tag is present, the strict
    payload contract is checked; malformed or incomplete contracts are still
    left for the flight controller to navigate, but payload actions are
    disabled and the errors are reported as a warning.
    """

    starts = [
        item
        for item in items
        if item.command == JUMP_TAG
        and int(round(item.param1)) == SCAN_START_TAG
    ]
    ends = [
        item
        for item in items
        if item.command == JUMP_TAG
        and int(round(item.param1)) == SCAN_END_TAG
    ]
    if not starts and not ends:
        return MISSION_MODE_PASSTHROUGH, None, []
    contract, errors = validate_mission(
        items,
        payload_servo=payload_servo,
        min_release_pwm=min_release_pwm,
    )
    if not errors and contract is not None:
        return MISSION_MODE_PAYLOAD, contract, []
    recon_contract, recon_errors = validate_recon_mission(items)
    if not recon_errors and recon_contract is not None:
        return MISSION_MODE_RECON_ONLY, recon_contract, []
    return MISSION_MODE_CONTRACT_WARNING, None, [*errors, *recon_errors]


def crossed_scan_end(previous_seq: int, current_seq: int, scan_end_seq: int) -> bool:
    """Return true once even if a transient JUMP_TAG sequence was skipped."""
    return previous_seq < scan_end_seq <= current_seq


def retry_entry_seq(items, contract) -> int:
    """Acquire a planned holding turn before another pass, when provided."""
    turns=[item.seq for item in items if item.seq<contract.scan_start_seq
           and item.command==mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS]
    return max(turns) if turns else contract.retry_start_seq


def ne_to_latlon(
    north_m: float,
    east_m: float,
    origin_lat_deg: float = ORIGIN_LAT_DEG,
    origin_lon_deg: float = ORIGIN_LON_DEG,
) -> tuple[float, float]:
    lat = origin_lat_deg + math.degrees(north_m / EARTH_RADIUS_M)
    lon = origin_lon_deg + math.degrees(
        east_m / (EARTH_RADIUS_M * math.cos(math.radians(origin_lat_deg)))
    )
    return lat, lon


def latlon_to_ne(
    lat_deg: float,
    lon_deg: float,
    origin_lat_deg: float = ORIGIN_LAT_DEG,
    origin_lon_deg: float = ORIGIN_LON_DEG,
) -> tuple[float, float]:
    north = math.radians(lat_deg - origin_lat_deg) * EARTH_RADIUS_M
    east = (
        math.radians(lon_deg - origin_lon_deg)
        * EARTH_RADIUS_M
        * math.cos(math.radians(origin_lat_deg))
    )
    return north, east


def derive_scan_rectangle(
    items: Sequence[MissionItem],
    contract: MissionContract | FixedFallbackContract | ReconContract,
    origin_lat_deg: float = ORIGIN_LAT_DEG,
    origin_lon_deg: float = ORIGIN_LON_DEG,
) -> ScanRectangle | None:
    """Recognise a low-threshold, adjustable two-lane scan rectangle.

    The route may contain any number of turn-helper NAV_WAYPOINT items between
    the lane pairs.  Geometry that is not sufficiently rectangle-like simply
    falls back to the existing polyline coverage behaviour; it never blocks a
    mission launch.
    """

    waypoints = [
        item
        for item in items
        if (
            contract.scan_start_seq < item.seq < contract.scan_end_seq
            and item.command == NAV_WAYPOINT
            and not (item.x == 0.0 and item.y == 0.0)
        )
    ]
    if len(waypoints) < 4:
        return None

    lane1_start, lane1_end = waypoints[0], waypoints[1]
    lane2_start, lane2_end = waypoints[-2], waypoints[-1]
    local = [
        latlon_to_ne(item.x, item.y, origin_lat_deg, origin_lon_deg)
        for item in (lane1_start, lane1_end, lane2_start, lane2_end)
    ]
    first_n = local[1][0] - local[0][0]
    first_e = local[1][1] - local[0][1]
    second_n = local[3][0] - local[2][0]
    second_e = local[3][1] - local[2][1]
    first_length = math.hypot(first_n, first_e)
    second_length = math.hypot(second_n, second_e)
    # No operational minimum lane length is imposed here.  Only reject a
    # truly degenerate pair so the direction normalisation below is defined.
    if min(first_length, second_length) <= 1e-6:
        return None

    opposite_alignment = (
        first_n * second_n + first_e * second_e
    ) / (first_length * second_length)
    if opposite_alignment > -0.75:
        return None
    if min(first_length, second_length) / max(first_length, second_length) < 0.65:
        return None

    unit_n = first_n / first_length
    unit_e = first_e / first_length

    def along_and_cross(point: tuple[float, float]) -> tuple[float, float]:
        delta_n = point[0] - local[0][0]
        delta_e = point[1] - local[0][1]
        return (
            delta_n * unit_n + delta_e * unit_e,
            delta_n * unit_e - delta_e * unit_n,
        )

    lane2_start_along, lane2_start_cross = along_and_cross(local[2])
    lane2_end_along, lane2_end_cross = along_and_cross(local[3])
    if lane2_start_cross * lane2_end_cross <= 0.0:
        return None
    width_m = 0.5 * (
        abs(lane2_start_cross) + abs(lane2_end_cross)
    )
    if width_m < 3.0:
        return None
    if abs(abs(lane2_start_cross) - abs(lane2_end_cross)) > max(
        15.0, width_m * 0.35
    ):
        return None
    endpoint_error = max(
        abs(lane2_start_along - first_length),
        abs(lane2_end_along),
    )
    if endpoint_error > max(first_length, second_length) * 0.35:
        return None

    return ScanRectangle(
        lane1_start_seq=lane1_start.seq,
        lane1_end_seq=lane1_end.seq,
        lane2_start_seq=lane2_start.seq,
        lane2_end_seq=lane2_end.seq,
        corner_latlon=tuple(
            (item.x, item.y)
            for item in (lane1_start, lane1_end, lane2_start, lane2_end)
        ),
        turn_waypoint_seqs=tuple(item.seq for item in waypoints[2:-2]),
        length_m=0.5 * (first_length + second_length),
        width_m=width_m,
        heading_deg=(math.degrees(math.atan2(first_e, first_n)) + 360.0)
        % 360.0,
    )


def scan_coverage_cells(
    items: Sequence[MissionItem],
    contract: MissionContract | FixedFallbackContract,
    cell_size_m: float = 5.0,
    origin_lat_deg: float = ORIGIN_LAT_DEG,
    origin_lon_deg: float = ORIGIN_LON_DEG,
) -> list[tuple[float, float, float]]:
    """Sample the scan lanes into five-metre coverage cells.

    For a recognised rectangle only the two long reconnaissance lanes count
    toward coverage.  The outside turn helpers therefore cannot make a good
    camera run look incomplete.  Non-rectangular legacy routes keep the
    original full-polyline behaviour.
    """

    if cell_size_m <= 0.0:
        raise ValueError("cell_size_m must be positive")
    waypoints = [
        item
        for item in items
        if (
            contract.scan_start_seq < item.seq < contract.scan_end_seq
            and item.command == NAV_WAYPOINT
            and not (item.x == 0.0 and item.y == 0.0)
        )
    ]
    if len(waypoints) < 2:
        return []
    rectangle = derive_scan_rectangle(
        items,
        contract,
        origin_lat_deg=origin_lat_deg,
        origin_lon_deg=origin_lon_deg,
    )
    if rectangle is None:
        segments = list(zip(waypoints, waypoints[1:]))
    else:
        by_seq = {item.seq: item for item in waypoints}
        segments = [
            (
                by_seq[rectangle.lane1_start_seq],
                by_seq[rectangle.lane1_end_seq],
            ),
            (
                by_seq[rectangle.lane2_start_seq],
                by_seq[rectangle.lane2_end_seq],
            ),
        ]
    cells: list[tuple[float, float, float]] = []
    for first, second in segments:
        first_n, first_e = latlon_to_ne(
            first.x, first.y, origin_lat_deg, origin_lon_deg
        )
        second_n, second_e = latlon_to_ne(
            second.x, second.y, origin_lat_deg, origin_lon_deg
        )
        distance = math.hypot(second_n - first_n, second_e - first_e)
        divisions = max(1, int(math.ceil(distance / cell_size_m)))
        for index in range(divisions + 1):
            ratio = index / divisions
            cell = (
                first_n + ratio * (second_n - first_n),
                first_e + ratio * (second_e - first_e),
                first.z + ratio * (second.z - first.z),
            )
            if not cells or math.hypot(
                cell[0] - cells[-1][0], cell[1] - cells[-1][1]
            ) > 0.25:
                cells.append(cell)
    return cells


def update_scan_coverage(
    cells: Sequence[tuple[float, float, float]],
    covered: set[int],
    north_m: float,
    east_m: float,
    agl_m: float,
    camera_age_s: float,
    max_cross_track_m: float = 12.0,
    altitude_tolerance_m: float = 5.0,
    max_camera_age_s: float = 3.0,
) -> float:
    """Mark cells covered only when camera, position and height are usable."""

    if not cells:
        return 0.0
    if camera_age_s < 0.0 or camera_age_s > max_camera_age_s:
        return len(covered) / len(cells)
    for index, (cell_n, cell_e, cell_alt) in enumerate(cells):
        if index in covered or abs(agl_m - cell_alt) > altitude_tolerance_m:
            continue
        if math.hypot(north_m - cell_n, east_m - cell_e) <= max_cross_track_m:
            covered.add(index)
    return len(covered) / len(cells)


def coordinated_turn_radius(
    speed_mps: float,
    bank_deg: float = MAX_BANK_DEG,
) -> float:
    """Return the still-air coordinated-turn radius for a fixed-wing turn."""

    if speed_mps <= 0.0:
        raise ValueError("speed_mps must be positive")
    if not 0.0 < bank_deg < 90.0:
        raise ValueError("bank_deg must be within 0..90")
    return speed_mps * speed_mps / (
        GRAVITY_MPS2 * math.tan(math.radians(bank_deg))
    )


def _choose_half_turn(
    current_north_m: float,
    current_east_m: float,
    release_north_m: float,
    release_east_m: float,
    radius_m: float,
    max_home_distance_m: float,
    outbound_north: float | None = None,
    outbound_east: float | None = None,
) -> tuple[float, float, float, float, float, float]:
    """Choose a tangent half-turn that reverses the scan toward the target.

    Candidate loiter centres start at the scan-tangent position and extend
    laterally when necessary.  The extra offset is important when the target
    is close to the decision guard: it guarantees a settled straight segment
    at least as long as the nominal ballistic lead before release.
    """

    if (outbound_north is None) != (outbound_east is None):
        raise ValueError("outbound north/east heading must be provided together")
    if outbound_north is None:
        distance_home = math.hypot(current_north_m, current_east_m)
        if distance_home < 1.0:
            outbound_north, outbound_east = 1.0, 0.0
        else:
            outbound_north = current_north_m / distance_home
            outbound_east = current_east_m / distance_home
    else:
        outbound_length = math.hypot(outbound_north, outbound_east)
        if outbound_length < 1e-6:
            raise ValueError("outbound heading cannot be zero")
        outbound_north /= outbound_length
        outbound_east /= outbound_length
    right_north, right_east = -outbound_east, outbound_north
    attack_distance = math.hypot(release_north_m, release_east_m)
    attack_north = -release_north_m / max(1.0, attack_distance)
    attack_east = -release_east_m / max(1.0, attack_distance)
    candidates: list[tuple[float, float, float, float, float, float, float]] = []
    candidate_centres = (
        (scale, side, side)
        for scale in (1.0, 1.5, 1.6, 1.75, 2.0, 2.25)
        for side in (1.0, -1.0)
    )
    for center_offset_scale, side, orbit_direction in candidate_centres:
        center_north = (
            current_north_m
            + side * right_north * radius_m * center_offset_scale
        )
        center_east = (
            current_east_m
            + side * right_east * radius_m * center_offset_scale
        )
        conservative_extent = math.hypot(center_north, center_east) + radius_m
        if conservative_extent > max_home_distance_m:
            continue
        # ArduPlane can only form a real tangent to the following release
        # waypoint when that waypoint is outside the loiter circle. Keeping a
        # margin also prevents numerical/tuning error from selecting the
        # opposite, outbound tangent.
        to_release_north = release_north_m - center_north
        to_release_east = release_east_m - center_east
        release_distance = math.hypot(to_release_north, to_release_east)
        if release_distance <= radius_m + 0.5:
            continue
        base_scale = radius_m * radius_m / (release_distance * release_distance)
        offset_scale = (
            radius_m
            * math.sqrt(release_distance * release_distance - radius_m * radius_m)
            / (release_distance * release_distance)
        )
        base_north = center_north + base_scale * to_release_north
        base_east = center_east + base_scale * to_release_east
        perpendicular_north = -to_release_east
        perpendicular_east = to_release_north
        # The direction matches the aircraft's outbound tangent at the nearest
        # point on this circle. Reversing it makes L1 approach the circle
        # against the requested orbit and can produce a large capture spiral.
        entry_angle = math.atan2(
            current_east_m - center_east,
            current_north_m - center_north,
        )
        for tangent_side in (1.0, -1.0):
            tangent_north = base_north + tangent_side * offset_scale * perpendicular_north
            tangent_east = base_east + tangent_side * offset_scale * perpendicular_east
            radial_north = tangent_north - center_north
            radial_east = tangent_east - center_east
            if orbit_direction > 0.0:
                heading_north, heading_east = -radial_east, radial_north
            else:
                heading_north, heading_east = radial_east, -radial_north
            path_north = release_north_m - tangent_north
            path_east = release_east_m - tangent_east
            path_length = math.hypot(path_north, path_east)
            if path_length < MIN_ATTACK_STRAIGHT_M:
                continue
            directed = (
                heading_north * path_north + heading_east * path_east
            ) / max(1e-6, radius_m * path_length)
            if directed < 0.999:
                continue
            path_north /= path_length
            path_east /= path_length
            tangent_angle = math.atan2(radial_east, radial_north)
            if orbit_direction > 0.0:
                sweep = (tangent_angle - entry_angle) % (2.0 * math.pi)
            else:
                sweep = (entry_angle - tangent_angle) % (2.0 * math.pi)
            attack_alignment = path_north * attack_north + path_east * attack_east
            # Prefer an inward tangent while accounting for the distance to
            # acquire the circle and reach the target-bound tangent.  Starting
            # on a one-radius-offset circle is substantially more repeatable
            # than asking L1 to intercept a distant loiter circle.
            estimated_route_m = radius_m * center_offset_scale + path_length
            score = (
                -100.0 * attack_alignment
                + 8.0 * abs(sweep - math.pi)
                + 0.5 * estimated_route_m
            )
            candidates.append(
                (
                    score,
                    center_north,
                    center_east,
                    orbit_direction,
                    path_north,
                    path_east,
                    sweep / (2.0 * math.pi),
                )
            )
    if not candidates:
        raise ValueError(
            f"no {radius_m:.1f} m half-turn fits inside "
            f"the {max_home_distance_m:.0f} m home boundary"
        )
    (
        _,
        center_north,
        center_east,
        direction,
        tangent_north,
        tangent_east,
        turn_fraction,
    ) = min(candidates)
    return (
        center_north,
        center_east,
        direction,
        tangent_north,
        tangent_east,
        turn_fraction,
    )


def build_attack_patch(
    items: Sequence[MissionItem],
    contract: MissionContract,
    target_north_m: float,
    target_east_m: float,
    current_north_m: float,
    current_east_m: float,
    nominal_speed_mps: float = 16.0,
    nominal_agl_m: float | None = None,
    max_home_distance_m: float = ROUTE_PLANNING_LIMIT_M,
    planning_bank_deg: float = MAX_BANK_DEG,
    origin_lat_deg: float = ORIGIN_LAT_DEG,
    origin_lon_deg: float = ORIGIN_LON_DEG,
    payload_servo: int = PAYLOAD_SERVO,
    release_pwm: float = PAYLOAD_RELEASE_PWM,
    outbound_north: float | None = None,
    outbound_east: float | None = None,
    attack_altitude_limit_m: float | None = 20.0,
) -> list[MissionItem]:
    by_seq = {item.seq: item for item in items}
    templates = [by_seq[seq] for seq in range(contract.fallback_start_seq, contract.fallback_exit_seq + 1)]
    altitude = 20.0 if nominal_agl_m is None else max(5.0, nominal_agl_m)
    if attack_altitude_limit_m is not None:
        altitude = min(attack_altitude_limit_m, altitude)
    radius = max(
        MIN_LOITER_RADIUS_M,
        coordinated_turn_radius(nominal_speed_mps, planning_bank_deg),
    )
    (
        loiter_n,
        loiter_e,
        direction,
        attack_heading_n,
        attack_heading_e,
        geometric_turn_fraction,
    ) = _choose_half_turn(
        current_north_m,
        current_east_m,
        target_north_m,
        target_east_m,
        radius,
        max_home_distance_m,
        outbound_north,
        outbound_east,
    )
    # The active navigation waypoint is the selected target itself.  The
    # companion releases at the dynamically computed upstream point, while L1
    # keeps converging on the target even when the actual loiter exit differs
    # from the ideal geometric tangent.  A waypoint beyond the target preserves
    # the ideal tangent instead and can carry that entry error all the way
    # through the ballistic release line.
    attack_n = target_north_m
    attack_e = target_east_m
    exit_heading_n, exit_heading_e = attack_heading_n, attack_heading_e
    exit_n = target_north_m + exit_heading_n * 30.0
    exit_e = target_east_m + exit_heading_e * 30.0
    loiter_lat, loiter_lon = ne_to_latlon(
        loiter_n, loiter_e, origin_lat_deg, origin_lon_deg
    )
    attack_lat, attack_lon = ne_to_latlon(
        attack_n, attack_e, origin_lat_deg, origin_lon_deg
    )
    exit_distance = math.hypot(exit_n, exit_e)
    if exit_distance > max_home_distance_m:
        scale = max_home_distance_m / exit_distance
        exit_n *= scale
        exit_e *= scale
    exit_lat, exit_lon = ne_to_latlon(
        exit_n, exit_e, origin_lat_deg, origin_lon_deg
    )
    return [
        replace(
            templates[0],
            command=NAV_LOITER_TURNS,
            # Encode the target-bound tangent phase. A fixed half-turn can
            # miss that phase and force one additional orbit while ArduPlane
            # waits for its next-waypoint heading check to become true.
            param1=geometric_turn_fraction,
            param2=0.0,
            param3=direction * radius,
            param4=1.0,
            x=loiter_lat,
            y=loiter_lon,
            z=altitude,
        ),
        replace(
            templates[1],
            param2=3.0,
            x=attack_lat,
            y=attack_lon,
            z=altitude,
        ),
        replace(
            templates[2],
            command=DO_SET_SERVO,
            param1=float(payload_servo),
            # Keep the mission latch high after the companion's real-time
            # ballistic command.  This is hardware-command redundancy, not a
            # second release decision: it prevents a brief MAVLink command
            # loss from leaving the payload attached.
            param2=float(release_pwm),
            x=0.0,
            y=0.0,
            z=0.0,
        ),
        replace(templates[3], x=exit_lat, y=exit_lon, z=max(10.0, altitude - 5.0)),
    ]


def build_descent_side_patch(
    items: Sequence[MissionItem],
    contract: MissionContract,
    side: float,
) -> list[MissionItem]:
    """Mirror the public descent route toward the estimated target side."""

    side_sign = 1.0 if side >= 0.0 else -1.0
    patched: list[MissionItem] = []
    for item in items[
        contract.fallback_exit_seq + 1 : contract.land_seq + 1
    ]:
        if item.x == 0.0 and item.y == 0.0:
            patched.append(item)
            continue
        north_m, east_m = latlon_to_ne(item.x, item.y)
        # The approach arc may use either side, but the last two waypoints are
        # already on the runway axis.  Keep NAV_LAND on that axis as well so
        # the physical ground roll does not amplify a lateral aim offset.
        mirrored_east = (
            0.0
            if item.seq == contract.land_seq
            else side_sign * abs(east_m)
        )
        latitude, longitude = ne_to_latlon(north_m, mirrored_east)
        patched.append(replace(item, x=latitude, y=longitude))
    return patched


def mission_route_metrics(items: Sequence[MissionItem]) -> dict[str, float]:
    """Return conservative public route length and home-distance metrics."""

    navigable = {
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        NAV_WAYPOINT,
        NAV_LOITER_TURNS,
        NAV_LOITER_TO_ALT,
        NAV_LAND,
    }
    previous = (0.0, 0.0)
    previous_altitude_m = 0.0
    length = 0.0
    maximum = 0.0
    for item in items:
        if item.command not in navigable or (item.x == 0.0 and item.y == 0.0):
            continue
        point = latlon_to_ne(item.x, item.y)
        length += math.hypot(point[0] - previous[0], point[1] - previous[1])
        if item.command in (NAV_LOITER_TURNS, NAV_LOITER_TO_ALT):
            if item.command == NAV_LOITER_TO_ALT:
                # LOITER_TO_ALT exits after reaching the requested altitude;
                # it is not inherently a half-circle command. The simulation
                # profile flies about 16 m/s and allows 3 m/s descent, so the
                # altitude change is the useful nominal path-length estimate.
                altitude_change_m = abs(item.z - previous_altitude_m)
                length += altitude_change_m * 16.0 / 3.0
                maximum = max(maximum, math.hypot(*point) + abs(item.param2))
            else:
                length += (
                    2.0
                    * math.pi
                    * abs(item.param3)
                    * max(0.0, item.param1)
                )
                maximum = max(maximum, math.hypot(*point) + abs(item.param3))
        else:
            maximum = max(maximum, math.hypot(*point))
        previous = point
        previous_altitude_m = item.z
    return {
        "nominal_length_m": length,
        "max_home_distance_m": maximum,
    }


def resequence(items: Iterable[MissionItem]) -> list[MissionItem]:
    return [replace(item, seq=index) for index, item in enumerate(items)]


def save_patch_backup(path: Path, items: Sequence[MissionItem]) -> None:
    payload = [
        {
            "seq": item.seq,
            "frame": item.frame,
            "command": item.command,
            "current": item.current,
            "autocontinue": item.autocontinue,
            "param1": item.param1,
            "param2": item.param2,
            "param3": item.param3,
            "param4": None if math.isnan(item.param4) else item.param4,
            "x": item.x,
            "y": item.y,
            "z": item.z,
        }
        for item in items
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + f".tmp-{os.getpid()}")
    try:
        with temporary.open("w", encoding="utf-8") as stream:
            stream.write(json.dumps(payload, ensure_ascii=False, indent=2) + "\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        directory_fd = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    finally:
        temporary.unlink(missing_ok=True)


def load_patch_backup(path: Path) -> list[MissionItem]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
        return [
            MissionItem(
                seq=int(item["seq"]),
                frame=int(item["frame"]),
                command=int(item["command"]),
                current=int(item["current"]),
                autocontinue=int(item["autocontinue"]),
                param1=float(item["param1"]),
                param2=float(item["param2"]),
                param3=float(item["param3"]),
                param4=(
                    math.nan
                    if item.get("param4") is None
                    else float(item["param4"])
                ),
                x=float(item["x"]),
                y=float(item["y"]),
                z=float(item["z"]),
            )
            for item in payload
        ]
    except (FileNotFoundError, json.JSONDecodeError, KeyError, TypeError, ValueError):
        return []
