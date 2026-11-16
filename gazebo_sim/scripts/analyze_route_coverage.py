#!/usr/bin/env python3
"""Estimate camera coverage for a QGroundControl reconnaissance route.

The report separates coverage of the complete 60 m target zone from coverage
of the four 0.6 m target-panel centres used by the simulator.  It is a
planning aid: attitude, wind, blur, occlusion and physical-camera calibration
still require flight validation.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from cuadc_sim.field_layout import derive_field_layout  # noqa: E402
from cuadc_sim.mission import MissionItem, derive_scan_rectangle  # noqa: E402

EARTH_RADIUS_M = 6378137.0
POSITION_COMMANDS = {16, 18, 21, 22, 31, 82}
SCAN_TAG = 600
SCAN_START = 9101
SCAN_END = 9102


def load_route(path: Path) -> list[MissionItem]:
    rows: list[MissionItem] = []
    lines = path.read_text(encoding="utf-8").splitlines()
    if not lines or lines[0].strip() != "QGC WPL 110":
        raise ValueError(f"{path}: expected QGC WPL 110")
    for line in lines[1:]:
        if not line.strip():
            continue
        fields = line.split()
        if len(fields) != 12:
            raise ValueError(f"{path}: expected 12 fields, got {len(fields)}")
        rows.append(
            MissionItem(
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
        )
    return rows


def local_xy(item: MissionItem, origin_lat: float, origin_lon: float) -> tuple[float, float]:
    north = math.radians(item.x - origin_lat) * EARTH_RADIUS_M
    east = math.radians(item.y - origin_lon) * EARTH_RADIUS_M * math.cos(math.radians(origin_lat))
    return north, east


def distance_to_segment(point, first, second) -> float:
    px, py = point
    ax, ay = first
    bx, by = second
    vx, vy = bx - ax, by - ay
    length_sq = vx * vx + vy * vy
    if length_sq <= 1.0e-12:
        return math.hypot(px - ax, py - ay)
    ratio = max(0.0, min(1.0, ((px - ax) * vx + (py - ay) * vy) / length_sq))
    return math.hypot(px - (ax + ratio * vx), py - (ay + ratio * vy))


def covered(point, segments, half_angle_rad: float) -> bool:
    return any(
        distance_to_segment(point, first, second)
        <= max(1.0, altitude * math.tan(half_angle_rad))
        for first, second, altitude in segments
    )


def analyze(route: Path, horizontal_fov_deg: float = 80.0) -> dict:
    items = load_route(route)
    layout = derive_field_layout(items)
    starts = [
        item.seq
        for item in items
        if item.command == SCAN_TAG and round(item.param1) == SCAN_START
    ]
    ends = [
        item.seq
        for item in items
        if item.command == SCAN_TAG and round(item.param1) == SCAN_END
    ]
    if len(starts) != 1 or len(ends) != 1 or starts[0] >= ends[0]:
        raise ValueError(f"{route}: route must contain one ordered 9101/9102 pair")
    origin = items[0]
    scan_items = [
        item
        for item in items
        if starts[0] < item.seq < ends[0]
        and item.command in POSITION_COMMANDS
        and (item.x != 0.0 or item.y != 0.0)
    ]
    if len(scan_items) < 2:
        raise ValueError(f"{route}: reconnaissance segment has fewer than two points")
    scan_xy = [local_xy(item, origin.x, origin.y) for item in scan_items]
    segments = [
        (first, second, (left.z + right.z) / 2.0)
        for (first, left), (second, right) in zip(zip(scan_xy, scan_items), zip(scan_xy[1:], scan_items[1:]))
    ]
    half_angle = math.radians(horizontal_fov_deg) / 2.0
    target_north = layout.target_center_north_m
    target_east = layout.target_center_east_m
    grid_total = grid_covered = 0
    for north_index in range(-60, 61):
        for east_index in range(-60, 61):
            grid_total += 1
            point = (target_north + north_index * 0.5, target_east + east_index * 0.5)
            grid_covered += int(covered(point, segments, half_angle))

    angle = math.radians(layout.target_heading_deg)
    panel_centres = []
    panel_covered = 0
    for local_north in (-17.5, 17.5):
        for local_east in (-17.5, 17.5):
            point = (
                target_north + local_north * math.cos(angle) - local_east * math.sin(angle),
                target_east + local_north * math.sin(angle) + local_east * math.cos(angle),
            )
            is_covered = covered(point, segments, half_angle)
            panel_centres.append({"north_m": round(point[0], 2), "east_m": round(point[1], 2), "covered": is_covered})
            panel_covered += int(is_covered)
    return {
        "schema": "cuadc-route-coverage.v1",
        "route": route.name,
        "camera_horizontal_fov_deg": horizontal_fov_deg,
        "scan_waypoint_count": len(scan_items),
        "scan_altitude_range_m": [min(item.z for item in scan_items), max(item.z for item in scan_items)],
        "route_max_distance_m": round(layout.route_max_distance_m, 2),
        "target_zone_60m_square_coverage": round(grid_covered / grid_total, 4),
        "target_panel_centres_covered": panel_covered,
        "target_panel_centres_total": len(panel_centres),
        "target_panel_centres_coverage": round(panel_covered / len(panel_centres), 4),
        "panel_centres": panel_centres,
        "limitations": [
            "Uses the simulated 80 degree horizontal FOV and flat-ground footprint.",
            "Does not model blur, occlusion, attitude, wind, exposure or physical-camera calibration.",
        ],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("route", type=Path)
    parser.add_argument("--fov-deg", type=float, default=80.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = analyze(args.route.resolve(), args.fov_deg)
    text = json.dumps(result, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
    print(text, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
