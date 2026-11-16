from __future__ import annotations

import math
import json
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Iterable, Sequence

from pymavlink import mavutil

from .config import (
    ORIGIN_ALT_M,
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
NAV_LOITER_UNLIM = mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM
NAV_LOITER_TIME = mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME
NAV_LOITER_TURNS = mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS
NAV_LOITER_TO_ALT = mavutil.mavlink.MAV_CMD_NAV_LOITER_TO_ALT
NAV_LAND = mavutil.mavlink.MAV_CMD_NAV_LAND
NAV_TAKEOFF = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
GLOBAL_RELATIVE_ALT_INT = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT

EARTH_RADIUS_M = 6378137.0
GRAVITY_MPS2 = 9.80665
MAX_HOME_DISTANCE_M = 300.0
ROUTE_PLANNING_LIMIT_M = 288.0
MAX_BANK_DEG = 30.0
MIN_LOITER_RADIUS_M = 35.0
MIN_ATTACK_STRAIGHT_M = 70.0

MISSION_MODE_DYNAMIC_READY = "DYNAMIC_READY"
MISSION_MODE_FIXED_FALLBACK = "FIXED_FALLBACK"
MISSION_MODE_RAW_REPLAY = "RAW_REPLAY_READY"
MISSION_MODE_BLOCKED = "BLOCKED"

# Commands supported by ArduPlane's mission library and by the simulator's
# explicit physical adapters.  A route item outside this list is rejected so
# an ignored/unknown command cannot be reported as a successful replay.
SUPPORTED_REPLAY_COMMANDS = frozenset(
    {
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,
        mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS,
        mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME,
        mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        mavutil.mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT,
        mavutil.mavlink.MAV_CMD_NAV_DELAY,
        mavutil.mavlink.MAV_CMD_CONDITION_DELAY,
        mavutil.mavlink.MAV_CMD_CONDITION_CHANGE_ALT,
        mavutil.mavlink.MAV_CMD_CONDITION_DISTANCE,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        mavutil.mavlink.MAV_CMD_DO_JUMP,
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,
        mavutil.mavlink.MAV_CMD_DO_SET_RELAY,
        mavutil.mavlink.MAV_CMD_DO_REPEAT_RELAY,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        mavutil.mavlink.MAV_CMD_DO_REPEAT_SERVO,
        mavutil.mavlink.MAV_CMD_DO_SET_ROI,
        mavutil.mavlink.MAV_CMD_DO_DIGICAM_CONFIGURE,
        mavutil.mavlink.MAV_CMD_DO_DIGICAM_CONTROL,
        mavutil.mavlink.MAV_CMD_DO_SET_CAM_TRIGG_DIST,
        JUMP_TAG,
    }
)


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
class FixedFallbackContract:
    """Minimum executable tagged mission with a fixed payload fallback."""

    scan_start_seq: int
    scan_end_seq: int
    retry_start_seq: int
    fallback_release_seq: int
    land_seq: int


@dataclass(frozen=True)
class RawReplayContract:
    """The uploaded mission remains authoritative throughout this run."""

    home_lat_deg: float
    home_lon_deg: float
    home_alt_m: float
    first_flight_seq: int
    land_seq: int | None
    release_sequences: tuple[int, ...] = ()
    automatic_takeoff: bool = False
    # Optional tags delimit the route's reconnaissance leg for coverage
    # reporting. They never authorize a mission rewrite.
    scan_start_seq: int = -1
    scan_end_seq: int = -1
    # Raw routes do not have generated fallback slots, but the companion
    # state machine still uses this boundary while draining the final scan
    # frame.  Keep it at the uploaded landing item so raw replay follows the
    # same one-shot scan-end timing without inventing mission commands.
    fallback_start_seq: int = -1


@dataclass(frozen=True)
class ScanRectangle:
    """Two opposite scan lanes that bound an adjustable reconnaissance box."""

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
    contract: MissionContract | FixedFallbackContract | RawReplayContract | None
    errors: tuple[str, ...] = ()
    warnings: tuple[str, ...] = ()

    @property
    def launch_allowed(self) -> bool:
        return self.mode != MISSION_MODE_BLOCKED and self.contract is not None


def infer_replay_home(items: Sequence[MissionItem]) -> tuple[float, float, float, bool]:
    """Use the uploaded HOME when present, otherwise align to the first route point.

    ArduPilot treats mission sequence zero as its live HOME record and ignores
    the planned coordinates sent in that slot.  After Mission Planner upload,
    a restarted SITL can therefore report its old default Home hundreds or
    thousands of kilometres from the uploaded route.  In that case the first
    executable global point is the only standard-MAVLink origin available.
    """

    if not items:
        return ORIGIN_LAT_DEG, ORIGIN_LON_DEG, ORIGIN_ALT_M, True
    ordered = sorted(items, key=lambda item: item.seq)
    home = ordered[0]
    home_valid = (
        math.isfinite(home.x)
        and math.isfinite(home.y)
        and -90.0 <= home.x <= 90.0
        and -180.0 <= home.y <= 180.0
        and (home.x != 0.0 or home.y != 0.0)
    )
    flight_positions = [
        item
        for item in ordered[1:]
        if item.command in {
            NAV_WAYPOINT,
            NAV_TAKEOFF,
            NAV_LAND,
            NAV_LOITER_UNLIM,
            NAV_LOITER_TIME,
            NAV_LOITER_TURNS,
            NAV_LOITER_TO_ALT,
            mavutil.mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT,
        }
        and math.isfinite(item.x)
        and math.isfinite(item.y)
        and -90.0 <= item.x <= 90.0
        and -180.0 <= item.y <= 180.0
        and (item.x != 0.0 or item.y != 0.0)
    ]
    displaced = False
    if home_valid and flight_positions:
        distances = [
            math.hypot(*latlon_to_ne(item.x, item.y, home.x, home.y))
            for item in flight_positions
        ]
        displaced = max(distances, default=0.0) > 100_000.0
    if (not home_valid or displaced) and flight_positions:
        first = flight_positions[0]
        return first.x, first.y, ORIGIN_ALT_M, True
    if home_valid:
        altitude = home.z if math.isfinite(home.z) else ORIGIN_ALT_M
        return home.x, home.y, altitude, False
    return ORIGIN_LAT_DEG, ORIGIN_LON_DEG, ORIGIN_ALT_M, True


def assess_raw_replay(
    items: Sequence[MissionItem],
    payload_servo: int = PAYLOAD_SERVO,
    payload_pwm: float = PAYLOAD_RELEASE_PWM,
    warning_home_distance_m: float = MAX_HOME_DISTANCE_M,
) -> MissionAssessment:
    """Check executable structure while retaining rule deviations as warnings."""

    if not items:
        return MissionAssessment(MISSION_MODE_BLOCKED, None, errors=("mission contains no items",))
    ordered = sorted(items, key=lambda item: item.seq)
    if [item.seq for item in ordered] != list(range(len(ordered))):
        return MissionAssessment(
            MISSION_MODE_BLOCKED,
            None,
            errors=("mission sequence numbers must be unique and contiguous from zero",),
        )
    home = ordered[0]
    if home.seq != 0 or home.command not in {
        NAV_WAYPOINT,
        NAV_TAKEOFF,
        NAV_LAND,
    }:
        return MissionAssessment(
            MISSION_MODE_BLOCKED,
            None,
            errors=("mission item 0 must contain the HOME position",),
        )
    home_lat, home_lon, home_alt, inferred_home = infer_replay_home(ordered)
    if not math.isfinite(home_alt):
        return MissionAssessment(
            MISSION_MODE_BLOCKED,
            None,
            errors=("mission HOME has invalid altitude",),
        )
    if inferred_home and not any(
        item.command in {NAV_WAYPOINT, NAV_TAKEOFF, NAV_LAND}
        and math.isfinite(item.x)
        and math.isfinite(item.y)
        and -90.0 <= item.x <= 90.0
        and -180.0 <= item.y <= 180.0
        and (item.x != 0.0 or item.y != 0.0)
        for item in ordered[1:]
    ):
        return MissionAssessment(
            MISSION_MODE_BLOCKED,
            None,
            errors=("mission HOME and route have no valid global position",),
        )

    errors: list[str] = []
    warnings: list[str] = []
    if inferred_home:
        warnings.append(
            "uploaded HOME is empty or displaced; aligned to the first executable waypoint"
        )
    for item in ordered:
        if item.command not in SUPPORTED_REPLAY_COMMANDS:
            errors.append(
                f"mission sequence {item.seq} uses unsupported command {item.command}"
            )
    if errors:
        return MissionAssessment(MISSION_MODE_BLOCKED, None, errors=tuple(errors))

    # Validate coordinates against this mission's own HOME, but deliberately
    # do not reject a route merely because it exceeds the competition radius.
    coordinate_errors = validate_route_safety(
        ordered,
        origin_lat_deg=home_lat,
        origin_lon_deg=home_lon,
    )
    if coordinate_errors:
        return MissionAssessment(
            MISSION_MODE_BLOCKED,
            None,
            errors=tuple(coordinate_errors),
        )
    distances: list[tuple[int, float]] = []
    for item in ordered:
        if item.seq == 0:
            continue
        if item.command not in {
            NAV_TAKEOFF,
            NAV_WAYPOINT,
            NAV_LOITER_UNLIM,
            NAV_LOITER_TURNS,
            NAV_LOITER_TIME,
            NAV_LOITER_TO_ALT,
            NAV_LAND,
            mavutil.mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT,
        } or (item.x == 0.0 and item.y == 0.0):
            continue
        north_m, east_m = latlon_to_ne(item.x, item.y, home_lat, home_lon)
        extent = math.hypot(north_m, east_m)
        if item.command in {NAV_LOITER_TURNS, NAV_LOITER_TIME, NAV_LOITER_UNLIM}:
            extent += abs(item.param3)
        distances.append((item.seq, extent))
    max_distance = max((distance for _, distance in distances), default=0.0)
    if max_distance > warning_home_distance_m:
        warnings.append(
            f"route reaches {max_distance:.0f} m from HOME, beyond the "
            f"{warning_home_distance_m:.0f} m competition boundary"
        )

    takeoffs = [item.seq for item in ordered if item.command == NAV_TAKEOFF]
    if not takeoffs:
        warnings.append(
            "mission has no NAV_TAKEOFF; the simulator will use its physical hand-launch initial condition"
        )
    lands = [item.seq for item in ordered if item.command == NAV_LAND]
    if not lands:
        warnings.append("mission has no NAV_LAND item; route replay may finish without an automatic landing phase")
    releases = [
        item.seq
        for item in ordered
        if item.command == DO_SET_SERVO
        and int(round(item.param1)) == int(payload_servo)
        and math.isfinite(item.param2)
        and abs(item.param2 - payload_pwm) <= 0.5
    ]
    scan_starts = [
        item.seq
        for item in ordered
        if item.command == JUMP_TAG and int(round(item.param1)) == SCAN_START_TAG
    ]
    scan_ends = [
        item.seq
        for item in ordered
        if item.command == JUMP_TAG and int(round(item.param1)) == SCAN_END_TAG
    ]
    unmatched_servo_items = [
        item
        for item in ordered
        if item.command == DO_SET_SERVO
        and int(round(item.param1)) == int(payload_servo)
        and item.seq not in releases
    ]
    for item in unmatched_servo_items:
        warnings.append(
            f"mission sequence {item.seq} uses servo {payload_servo} at "
            f"{item.param2:g} PWM; no physical payload mapping is configured for it"
        )
    contract = RawReplayContract(
        home_lat_deg=home_lat,
        home_lon_deg=home_lon,
        home_alt_m=home_alt,
        first_flight_seq=next(
            (item.seq for item in ordered[1:] if item.command in {
                NAV_TAKEOFF,
                NAV_WAYPOINT,
                NAV_LOITER_UNLIM,
                NAV_LOITER_TURNS,
                NAV_LOITER_TIME,
                NAV_LAND,
            }),
            1,
        ),
        land_seq=lands[-1] if lands else None,
        release_sequences=tuple(releases),
        automatic_takeoff=bool(takeoffs),
        scan_start_seq=scan_starts[0] if len(scan_starts) == 1 else -1,
        scan_end_seq=scan_ends[0] if len(scan_ends) == 1 else -1,
        fallback_start_seq=lands[-1] if lands else ordered[-1].seq,
    )
    return MissionAssessment(
        MISSION_MODE_RAW_REPLAY,
        contract,
        warnings=tuple(warnings),
    )


def validate_route_safety(
    items: Sequence[MissionItem],
    origin_lat_deg: float | None = None,
    origin_lon_deg: float | None = None,
    max_home_distance_m: float | None = None,
) -> list[str]:
    """Validate finite coordinates and an optional home-distance boundary."""

    errors: list[str] = []
    position_commands = {
        NAV_TAKEOFF,
        NAV_WAYPOINT,
        NAV_LOITER_TURNS,
        mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,
        mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME,
        NAV_LOITER_TO_ALT,
        NAV_LAND,
        mavutil.mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT,
    }
    for item in items:
        if item.command not in position_commands:
            continue
        if item.x == 0.0 and item.y == 0.0 and (
            item.seq == 0 or item.command == NAV_TAKEOFF
        ):
            # Mission Planner may leave the HOME row empty until the flight
            # controller supplies its current home position.
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


def validate_fixed_fallback_mission(
    items: Sequence[MissionItem],
    payload_servo: int = PAYLOAD_SERVO,
    min_release_pwm: float = 1700.0,
) -> tuple[FixedFallbackContract | None, list[str]]:
    """Validate only the structure required to finish the task safely.

    Smooth half-turn and dynamic-patch slots are deliberately not required.
    If they are missing, ArduPlane follows the fixed route drawn in Mission
    Planner and executes its own payload command before landing.
    """

    errors: list[str] = []
    if not items:
        return None, ["mission contains no items"]
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

    takeoffs = [item.seq for item in items if item.command == NAV_TAKEOFF and item.seq < start]
    if not takeoffs:
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


def derive_scan_rectangle(
    items: Sequence[MissionItem],
    contract: MissionContract | FixedFallbackContract | RawReplayContract,
    origin_lat_deg: float = ORIGIN_LAT_DEG,
    origin_lon_deg: float = ORIGIN_LON_DEG,
) -> ScanRectangle | None:
    """Recognise the same low-threshold two-lane rectangle as the real aircraft."""

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
    contract: MissionContract | FixedFallbackContract | RawReplayContract,
    cell_size_m: float = 5.0,
    origin_lat_deg: float = ORIGIN_LAT_DEG,
    origin_lon_deg: float = ORIGIN_LON_DEG,
) -> list[tuple[float, float, float]]:
    """Sample only the two long lanes when the route is rectangle-like."""

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
    """Mark nearby planned cells covered when pose and camera are healthy."""

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
    before_land=items[contract.land_seq-1]
    approach_n,approach_e=latlon_to_ne(before_land.x,before_land.y)
    east_west_approach=abs(approach_n)<3 and abs(approach_e)>10
    patched: list[MissionItem] = []
    for item in items[
        contract.fallback_exit_seq + 1 : contract.land_seq + 1
    ]:
        if item.x == 0.0 and item.y == 0.0:
            patched.append(item)
            continue
        north_m, east_m = latlon_to_ne(item.x, item.y)
        # The approach arc may use either side, but the last two waypoints are
        # already on the runway axis.  Keep NAV_LAND on that axis as well: a
        # 10 m lateral aim offset becomes roughly 80 m after the Gazebo ground
        # roll because the short final segment points diagonally.
        mirrored_east = (
            0.0
            if item.seq == contract.land_seq and not east_west_approach
            else side_sign * abs(east_m)
        )
        latitude, longitude = ne_to_latlon(north_m, mirrored_east)
        # Reflecting the arc's centre also reflects its direction. Keeping the
        # old clockwise radius on the opposite side creates a reversal at the
        # entry tangent and a large overshoot before landing.
        radius = item.param3
        if east_west_approach and item.command == NAV_LOITER_TURNS:
            radius *= side_sign
        patched.append(replace(item, x=latitude, y=longitude, param3=radius))
    return patched


def mission_route_metrics(items: Sequence[MissionItem]) -> dict[str, float]:
    """Return conservative public route length and home-distance metrics."""

    navigable = {
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        NAV_WAYPOINT,
        NAV_LOITER_TURNS,
        mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,
        mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME,
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
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


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
