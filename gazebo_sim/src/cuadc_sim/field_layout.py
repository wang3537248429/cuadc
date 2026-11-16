from __future__ import annotations

import json
import math
import os
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Sequence
from types import SimpleNamespace

from pymavlink import mavutil

from .config import ORIGIN_ALT_M, ORIGIN_LAT_DEG, ORIGIN_LON_DEG, RUNTIME_DIR
from .mission import (
    JUMP_TAG,
    MissionItem,
    NAV_LAND,
    NAV_TAKEOFF,
    NAV_WAYPOINT,
    infer_replay_home,
    latlon_to_ne,
    derive_scan_rectangle,
)


@dataclass(frozen=True)
class FieldLayout:
    group_centers_ne: dict[str, tuple[float,float]] | None = None
    origin_lat_deg: float = ORIGIN_LAT_DEG
    origin_lon_deg: float = ORIGIN_LON_DEG
    origin_alt_m: float = ORIGIN_ALT_M
    runway_center_north_m: float = 0.0
    runway_center_east_m: float = 0.0
    runway_heading_deg: float = 0.0
    runway_length_m: float = 145.0
    runway_width_m: float = 72.0
    target_center_north_m: float = 200.0
    target_center_east_m: float = 0.0
    target_heading_deg: float = 0.0
    initial_heading_deg: float = 0.0
    field_size_m: float = 700.0
    route_max_distance_m: float = 0.0

    def to_dict(self) -> dict:
        return asdict(self)


def derive_field_layout(items: Sequence[MissionItem]) -> FieldLayout:
    """Estimate field geometry from Home, scan-tag waypoints and landing approach."""

    if not items:
        return FieldLayout()
    ordered = sorted(items, key=lambda item: item.seq)
    origin_lat, origin_lon, origin_alt, _ = infer_replay_home(ordered)

    starts = [
        item.seq for item in ordered
        if item.command == JUMP_TAG and int(round(item.param1)) == 9101
    ]
    ends = [
        item.seq for item in ordered
        if item.command == JUMP_TAG and int(round(item.param1)) == 9102
    ]
    scan = []
    if len(starts) == 1 and len(ends) == 1 and starts[0] < ends[0]:
        scan = [
            item for item in ordered
            if starts[0] < item.seq < ends[0]
            and item.command in {NAV_WAYPOINT, NAV_TAKEOFF, NAV_LAND}
            and (item.x != 0.0 or item.y != 0.0)
        ]
    if scan:
        scan_ne = [latlon_to_ne(item.x, item.y, origin_lat, origin_lon) for item in scan]
        rectangle = derive_scan_rectangle(ordered, SimpleNamespace(scan_start_seq=starts[0],scan_end_seq=ends[0]),
                                          origin_lat_deg=origin_lat,origin_lon_deg=origin_lon)
        if rectangle is not None:
            scan_ne = [latlon_to_ne(lat,lon,origin_lat,origin_lon) for lat,lon in rectangle.corner_latlon]
        target_n = sum(point[0] for point in scan_ne) / len(scan_ne)
        target_e = sum(point[1] for point in scan_ne) / len(scan_ne)
        scan_heading = 0.0
        for first, second in zip(scan_ne, scan_ne[1:]):
            delta_n, delta_e = second[0] - first[0], second[1] - first[1]
            if math.hypot(delta_n, delta_e) > 8.0:
                scan_heading = math.degrees(math.atan2(delta_e, delta_n)) % 360.0
                break
    else:
        target_n, target_e = 200.0, 0.0
        scan_heading = 0.0

    position_items = [
        item for item in ordered
        if item.seq > 0
        and item.command in {NAV_WAYPOINT, NAV_TAKEOFF, NAV_LAND}
        and (item.x != 0.0 or item.y != 0.0)
    ]
    initial_heading = 0.0
    first_course = next(
        (item for item in position_items if item.seq > 0 and item.command in {NAV_WAYPOINT, NAV_TAKEOFF}),
        None,
    )
    if first_course is not None:
        course_n, course_e = latlon_to_ne(first_course.x, first_course.y, origin_lat, origin_lon)
        if math.hypot(course_n, course_e) > 1.0:
            initial_heading = math.degrees(math.atan2(course_e, course_n)) % 360.0
        else:
            next_course = next(
                (item for item in position_items if item.seq > first_course.seq),
                None,
            )
            if next_course is not None:
                next_n, next_e = latlon_to_ne(
                    next_course.x,
                    next_course.y,
                    first_course.x,
                    first_course.y,
                )
                if math.hypot(next_n, next_e) > 1.0:
                    initial_heading = math.degrees(math.atan2(next_e, next_n)) % 360.0
    land = next((item for item in reversed(position_items) if item.command == NAV_LAND), None)
    runway_center_n = runway_center_e = 0.0
    heading_deg = 0.0
    if land is not None:
        land_point = latlon_to_ne(land.x, land.y, origin_lat, origin_lon)
        approach = [item for item in position_items if item.seq < land.seq][-3:]
        approach_points = [
            latlon_to_ne(item.x, item.y, origin_lat, origin_lon)
            for item in approach
        ] + [land_point]
        runway_center_n = sum(point[0] for point in approach_points) / len(approach_points)
        runway_center_e = sum(point[1] for point in approach_points) / len(approach_points)
        if len(approach_points) >= 2:
            delta_n = approach_points[-1][0] - approach_points[-2][0]
            delta_e = approach_points[-1][1] - approach_points[-2][1]
            if math.hypot(delta_n, delta_e) > 1.0:
                heading_deg = math.degrees(math.atan2(delta_e, delta_n)) % 360.0

    max_distance = 0.0
    for item in ordered:
        if item.seq == 0:
            continue
        if item.command not in {
            NAV_WAYPOINT,
            NAV_TAKEOFF,
            NAV_LAND,
            mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,
            mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME,
            mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS,
            mavutil.mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT,
        } or (item.x == 0.0 and item.y == 0.0):
            continue
        north, east = latlon_to_ne(item.x, item.y, origin_lat, origin_lon)
        max_distance = max(max_distance, math.hypot(north, east) + abs(item.param3))
    field_size = max(700.0, math.ceil((max_distance * 2.0 + 160.0) / 100.0) * 100.0)
    return FieldLayout(
        origin_lat_deg=origin_lat,
        origin_lon_deg=origin_lon,
        origin_alt_m=origin_alt,
        runway_center_north_m=runway_center_n,
        runway_center_east_m=runway_center_e,
        runway_heading_deg=heading_deg,
        target_center_north_m=target_n,
        target_center_east_m=target_e,
        target_heading_deg=scan_heading,
        initial_heading_deg=initial_heading,
        field_size_m=field_size,
        route_max_distance_m=max_distance,
    )


def write_field_layout(layout: FieldLayout, path: Path = RUNTIME_DIR / "field_layout.json") -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(layout.to_dict(), ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    os.replace(temporary, path)


def load_field_layout(path: Path = RUNTIME_DIR / "field_layout.json") -> FieldLayout:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        return FieldLayout(**data)
    except (FileNotFoundError, json.JSONDecodeError, TypeError, ValueError):
        return FieldLayout()
