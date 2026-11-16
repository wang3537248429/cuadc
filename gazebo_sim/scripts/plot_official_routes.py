#!/usr/bin/env python3
"""Plot the checked-in official A/B routes in local North/East coordinates.

The plot is deliberately based on the uploaded WPL files and the checked-in
official arena profiles.  It does not generate or smooth replacement
waypoints.  An optional route_state.json can be supplied to overlay a real
simulation trajectory from the latest run.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from PIL import Image
from PIL.PngImagePlugin import PngInfo


EARTH_RADIUS_M = 6378137.0
NAV_COMMANDS = {16, 18, 21, 22, 23, 24, 30, 84, 85}
JUMP_TAG = 600
DO_SET_SERVO = 183


def latlon_to_ne(lat: float, lon: float, origin_lat: float, origin_lon: float) -> tuple[float, float]:
    north = math.radians(lat - origin_lat) * EARTH_RADIUS_M
    east = math.radians(lon - origin_lon) * EARTH_RADIUS_M * math.cos(math.radians(origin_lat))
    return north, east


def load_route(path: Path) -> tuple[dict, list[dict]]:
    rows = []
    for raw in path.read_text(encoding="utf-8").splitlines():
        if not raw or raw.startswith("QGC WPL"):
            continue
        values = raw.split()
        if len(values) < 12:
            continue
        rows.append(
            {
                "seq": int(values[0]),
                "command": int(values[3]),
                "param1": float(values[4]),
                "param2": float(values[5]),
                "lat": float(values[8]),
                "lon": float(values[9]),
                "alt": float(values[10]),
            }
        )
    if not rows:
        raise ValueError(f"no mission items in {path}")
    home = rows[0]
    return home, rows


def local_route(path: Path, arena: dict) -> tuple[list[dict], dict]:
    home, rows = load_route(path)
    origin_lat = float(arena["field_layout"]["origin_lat_deg"])
    origin_lon = float(arena["field_layout"]["origin_lon_deg"])
    points = []
    for item in rows:
        if item["command"] in NAV_COMMANDS and (item["lat"] or item["lon"]):
            north, east = latlon_to_ne(item["lat"], item["lon"], origin_lat, origin_lon)
            points.append({**item, "north": north, "east": east})
    release = next((item for item in rows if item["command"] == DO_SET_SERVO), None)
    release_point = None
    if release:
        before_release = [point for point in points if point["seq"] < release["seq"]]
        release_point = before_release[-1] if before_release else None
    metadata = {
        "home": home,
        "release": release,
        "release_point": release_point,
        "target_center": (
            float(arena["field_layout"]["target_center_north_m"]),
            float(arena["field_layout"]["target_center_east_m"]),
        ),
        "target_heading": float(arena["field_layout"]["target_heading_deg"]),
        "runway_center": (
            float(arena["field_layout"]["runway_center_north_m"]),
            float(arena["field_layout"]["runway_center_east_m"]),
        ),
        "runway_heading": float(arena["field_layout"]["runway_heading_deg"]),
        "group_centers": {
            name: (float(point[0]), float(point[1]))
            for name, point in arena.get("group_centers_ne", {}).items()
        },
    }
    return points, metadata


def rotated_rectangle(center: tuple[float, float], heading_deg: float, half_size: float) -> list[tuple[float, float]]:
    angle = math.radians(heading_deg)
    result = []
    for along, cross in ((-half_size, -half_size), (-half_size, half_size), (half_size, half_size), (half_size, -half_size)):
        result.append(
            (
                center[0] + along * math.cos(angle) - cross * math.sin(angle),
                center[1] + along * math.sin(angle) + cross * math.cos(angle),
            )
        )
    return result


def load_actual_state(path: Path | None) -> dict | None:
    if path is None or not path.is_file():
        return None
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return None
    if data.get("schema") != "route_state.v1":
        return None
    return data


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--actual-state", type=Path, default=None)
    args = parser.parse_args()
    root = args.root.resolve()
    output = args.output or root / "docs/assets/official_route_trajectories.png"
    output.parent.mkdir(parents=True, exist_ok=True)
    actual = load_actual_state(args.actual_state)

    plt.rcParams["font.sans-serif"] = ["Noto Sans CJK SC", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False
    fig, axes = plt.subplots(1, 2, figsize=(15, 7), dpi=180, constrained_layout=True)
    colors = {"A": "#2563eb", "B": "#dc2626"}
    for axis, group in zip(axes, ("A", "B")):
        route_path = root / f"gazebo_sim/config/missions/official_route_{group.lower()}.waypoints"
        arena_path = root / f"gazebo_sim/config/arena_official_{group.lower()}.json"
        points, meta = local_route(route_path, json.loads(arena_path.read_text(encoding="utf-8")))
        east = [point["east"] for point in points]
        north = [point["north"] for point in points]
        color = colors[group]
        axis.plot(east, north, color=color, linewidth=1.8, marker="o", markersize=2.8, label="Mission Planner route", zorder=3)
        axis.scatter([0], [0], marker="*", s=125, color="#111827", edgecolor="white", linewidth=0.8, label="HOME", zorder=6)
        flight_points = [point for point in points if point["seq"] > 0]
        if flight_points:
            first, last = flight_points[0], flight_points[-1]
            axis.scatter([first["east"]], [first["north"]], marker=">", s=58, color="#16a34a", edgecolor="white", linewidth=0.7, label=f"takeoff seq {first['seq']}", zorder=6)
            axis.scatter([last["east"]], [last["north"]], marker="s", s=48, color="#dc2626", edgecolor="white", linewidth=0.7, label=f"land seq {last['seq']}", zorder=6)
        # Official profiles contain both target zones.  The selected zone is
        # aligned with the uploaded reconnaissance leg; the other zone is
        # its 180-degree mirror about the runway centre.
        zone_colors = {"A": "#2563eb", "B": "#dc2626"}
        for zone_name, center in meta["group_centers"].items():
            selected_zone = zone_name == group
            axis.add_patch(
                Polygon(
                    rotated_rectangle(center, meta["target_heading"], 30),
                    closed=True,
                    facecolor=zone_colors.get(zone_name, "#f59e0b"),
                    edgecolor=zone_colors.get(zone_name, "#b45309"),
                    alpha=0.18 if selected_zone else 0.08,
                    linewidth=1.5 if selected_zone else 1.0,
                    label=f"{zone_name} target zone (60 m × 60 m)",
                    zorder=1,
                )
            )
            axis.scatter(
                [center[1]],
                [center[0]],
                marker="+",
                s=90,
                color=zone_colors.get(zone_name, "#b45309"),
                linewidth=1.8,
                label=f"{zone_name} zone centre",
                zorder=5,
            )
        runway = meta["runway_center"]
        axis.scatter([runway[1]], [runway[0]], marker="P", s=62, color="#111827", label="runway centre", zorder=6)
        if meta["release_point"]:
            axis.scatter([meta["release_point"]["east"]], [meta["release_point"]["north"]], marker="X", s=90, color="#7c3aed", edgecolor="white", linewidth=0.7, label="release command vicinity", zorder=6)
            axis.annotate(f"release seq {meta['release']['seq'] if meta['release'] else ''}", (meta["release_point"]["east"], meta["release_point"]["north"]), xytext=(6, 8), textcoords="offset points", fontsize=8, color="#5b21b6")
        for point in points[:: max(1, len(points) // 9)]:
            axis.annotate(str(point["seq"]), (point["east"], point["north"]), xytext=(3, 3), textcoords="offset points", fontsize=7, color="#374151")
        if actual and str(actual.get("group", "")).upper() == group:
            trajectory = actual.get("trajectory", [])
            if len(trajectory) >= 2:
                axis.plot([item["east_m"] for item in trajectory], [item["north_m"] for item in trajectory], color="#111827", linewidth=1.0, alpha=0.75, label="latest simulated track", zorder=4)
            release = actual.get("release") or {}
            impact = actual.get("impact") or {}
            selected = actual.get("selected_target") or {}
            if release.get("aircraft_north_m") is not None:
                axis.scatter([release["aircraft_east_m"]], [release["aircraft_north_m"]], marker="D", s=48, color="#9333ea", label="actual release", zorder=7)
            if impact.get("north_m") is not None:
                axis.scatter([impact["east_m"]], [impact["north_m"]], marker="v", s=60, color="#059669", label="actual impact", zorder=7)
            if selected.get("north_m") is not None:
                axis.scatter([selected["east_m"]], [selected["north_m"]], marker="o", s=45, facecolors="none", edgecolors="#059669", label="selected target", zorder=7)
        axis.set_title(f"Official {group} route")
        axis.set_xlabel("East (m)")
        axis.set_ylabel("North (m)")
        axis.grid(True, linewidth=0.5, alpha=0.35)
        axis.set_aspect("equal", adjustable="datalim")
        axis.legend(loc="best", fontsize=7, frameon=True)
    fig.suptitle("CUADC official route trajectories (local North/East coordinates)", fontsize=15)
    fig.savefig(output, bbox_inches="tight")
    # Matplotlib writes a Software text chunk by default.  Keep the published
    # figure free of generator and host metadata so the release audit treats
    # it like the other checked-in visual assets.
    with Image.open(output) as image:
        image.save(output, pnginfo=PngInfo(), optimize=True)
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
