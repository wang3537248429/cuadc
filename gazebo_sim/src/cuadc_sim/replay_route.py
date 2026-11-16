from __future__ import annotations

import hashlib
import json
import math
from dataclasses import asdict
from pathlib import Path
from typing import Sequence

from .field_layout import FieldLayout
from .mission import (
    MissionItem,
    NAV_LAND,
    NAV_LOITER_TIME,
    NAV_LOITER_TURNS,
    NAV_LOITER_UNLIM,
    NAV_TAKEOFF,
    NAV_WAYPOINT,
)


def serialize_mission(items: Sequence[MissionItem]) -> list[dict]:
    """Serialize every uploaded field without JSON's non-standard NaN values."""

    result: list[dict] = []
    for item in items:
        values = asdict(item)
        for key, value in tuple(values.items()):
            if isinstance(value, float) and not math.isfinite(value):
                values[key] = "NaN" if math.isnan(value) else ("Infinity" if value > 0 else "-Infinity")
        result.append(values)
    return result


def deserialize_mission(values: Sequence[dict]) -> list[MissionItem]:
    result: list[MissionItem] = []
    for raw in values:
        data = dict(raw)
        for key, value in tuple(data.items()):
            if key in {"param1", "param2", "param3", "param4", "x", "y", "z"}:
                data[key] = float(value)
        result.append(MissionItem(**data))
    return result


def mission_route_compatible(
    reference: Sequence[MissionItem],
    observed: Sequence[MissionItem],
) -> bool:
    """Compare a saved upload with ArduPilot's downloaded mission.

    ArduPilot may replace sequence zero with its live HOME while preserving
    every executable item.  That one record is intentionally skipped here so
    a saved Mission Planner upload can restore its original HOME after a SITL
    restart without accepting a different route.
    """

    if len(reference) != len(observed):
        return False
    fields = ("frame", "command", "autocontinue", "param1", "param2", "param3", "param4", "x", "y", "z")
    for expected, actual in zip(reference, observed):
        if expected.seq != actual.seq:
            return False
        if expected.seq == 0:
            continue
        for field in fields:
            left = getattr(expected, field)
            right = getattr(actual, field)
            if isinstance(left, float) or isinstance(right, float):
                if math.isnan(float(left)) and math.isnan(float(right)):
                    continue
                tolerance = (
                    1.0
                    if (
                        field == "param2"
                        and expected.command == NAV_WAYPOINT
                    )
                    else 1e-5
                )
                if not math.isclose(float(left), float(right), rel_tol=0.0, abs_tol=tolerance):
                    return False
            elif left != right:
                return False
    return True


def mission_fingerprint(items: Sequence[MissionItem]) -> str:
    serialized = serialize_mission(items)
    if serialized:
        home = serialized[0]
        valid_home = (
            isinstance(home.get("x"), (int, float))
            and isinstance(home.get("y"), (int, float))
            and math.isfinite(float(home["x"]))
            and math.isfinite(float(home["y"]))
            and -90.0 <= float(home["x"]) <= 90.0
            and -180.0 <= float(home["y"]) <= 180.0
            and (float(home["x"]) != 0.0 or float(home["y"]) != 0.0)
        )
        route_positions = [
            item
            for item in items[1:]
            if item.command in {
                NAV_WAYPOINT,
                NAV_LOITER_UNLIM,
                NAV_LOITER_TURNS,
                NAV_LOITER_TIME,
                NAV_LAND,
                NAV_TAKEOFF,
                82,
            }
            and math.isfinite(item.x)
            and math.isfinite(item.y)
            and -90.0 <= item.x <= 90.0
            and -180.0 <= item.y <= 180.0
            and (item.x != 0.0 or item.y != 0.0)
        ]
        if valid_home and route_positions:
            from .mission import latlon_to_ne

            distances = [
                math.hypot(*latlon_to_ne(item.x, item.y, home["x"], home["y"]))
                for item in route_positions
            ]
            displaced = max(distances, default=0.0) > 100_000.0
        else:
            displaced = True
        if displaced:
            home["x"] = "AUTOPILOT_HOME"
            home["y"] = "AUTOPILOT_HOME"
            home["z"] = "AUTOPILOT_HOME"
    payload = json.dumps(serialized, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def write_replay_route(
    items: Sequence[MissionItem],
    layout: FieldLayout,
    path: Path,
) -> str:
    fingerprint = mission_fingerprint(items)
    payload = {
        "schema": "cuadc-mission-replay.v1",
        "fingerprint": fingerprint,
        "items": serialize_mission(items),
        "layout": layout.to_dict(),
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)
    return fingerprint


def load_replay_route(path: Path) -> dict | None:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError):
        return None
    if payload.get("schema") != "cuadc-mission-replay.v1":
        return None
    return payload


def mission_to_sdf(items: Sequence[MissionItem], origin_lat: float, origin_lon: float) -> list[dict]:
    from .mission import (
        NAV_LAND,
        NAV_LOITER_TO_ALT,
        NAV_LOITER_TURNS,
        NAV_TAKEOFF,
        NAV_WAYPOINT,
        latlon_to_ne,
    )
    from pymavlink import mavutil

    position_commands = {
        NAV_LAND,
        NAV_LOITER_TO_ALT,
        NAV_LOITER_TURNS,
        NAV_TAKEOFF,
        NAV_WAYPOINT,
        mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME,
        mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,
        mavutil.mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT,
    }
    points = []
    for item in items:
        if item.command not in position_commands or (item.x == 0.0 and item.y == 0.0):
            continue
        north, east = latlon_to_ne(item.x, item.y, origin_lat, origin_lon)
        points.append(
            {
                "seq": item.seq,
                "command": item.command,
                "north_m": north,
                "east_m": east,
                "alt_m": item.z,
            }
        )
    return points


def _material(color: str, texture_uri: str | None = None, roughness: float = 0.94) -> str:
    if texture_uri:
        return (
            "<material><ambient>0.72 0.72 0.70 1</ambient>"
            "<diffuse>0.78 0.78 0.76 1</diffuse><specular>0.04 0.04 0.04 1</specular>"
            f"<pbr><metal><albedo_map>{texture_uri}</albedo_map><roughness>{roughness:.2f}</roughness>"
            "<metalness>0</metalness></metal></pbr></material>"
        )
    return (
        f"<material><ambient>{color} 1</ambient><diffuse>{color} 1</diffuse>"
        f"<specular>0.03 0.03 0.03 1</specular><pbr><metal><roughness>{roughness:.2f}</roughness>"
        "<metalness>0</metalness></metal></pbr></material>"
    )


def field_surfaces_sdf(layout: FieldLayout, selected_group: str = "A", group_centers: dict | None = None) -> str:
    """Create a tiled grass ground, concrete airfield pad, joints and painted lines."""

    size = layout.field_size_m
    grass_scale = size / 10000.0
    px = layout.runway_center_north_m
    py = -layout.runway_center_east_m
    yaw_deg = -layout.runway_heading_deg
    runway_length = layout.runway_length_m
    runway_width = layout.runway_width_m
    concrete = "model://cuadc_world_assets/materials/textures/concrete_synthetic.jpg"
    parts = [
        f"""
    <model name="ground">
      <static>true</static><link name="ground_link">
        <collision name="ground_collision"><geometry><plane><normal>0 0 1</normal><size>{size:.1f} {size:.1f}</size></plane></geometry>
          <surface><friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction></surface>
        </collision>
        <visual name="grass_visual"><pose>0 0 -0.018 0 0 0</pose><cast_shadows>false</cast_shadows>
          <geometry><mesh><uri>model://cuadc_world_assets/meshes/airfield.dae</uri><scale>{grass_scale:.6f} {grass_scale:.6f} 1</scale></mesh></geometry>
          <material><ambient>0.38 0.42 0.32 1</ambient><diffuse>0.76 0.78 0.69 1</diffuse><specular>0.025 0.025 0.025 1</specular>
            <pbr><metal><albedo_map>model://cuadc_world_assets/materials/textures/Grass004_2K_Color.jpg</albedo_map><roughness>0.97</roughness><metalness>0</metalness></metal></pbr>
          </material>
        </visual>
      </link>
    </model>
    <model name="runway_apron"><static>true</static><pose degrees="true">{px:.4f} {py:.4f} 0 0 0 {yaw_deg:.4f}</pose><link name="apron">
      <visual name="apron_transition"><pose>0 0 0.003 0 0 0</pose><cast_shadows>false</cast_shadows><geometry><box><size>{runway_length + 28.0:.2f} {runway_width + 24.0:.2f} 0.006</size></box></geometry>{_material('0.35 0.39 0.31')}</visual>
      <visual name="concrete_pad"><pose>0 0 0.008 0 0 0</pose><cast_shadows>false</cast_shadows><geometry><box><size>{runway_length:.2f} {runway_width:.2f} 0.012</size></box></geometry>{_material('0.62 0.63 0.62', concrete, 0.96)}</visual>
"""
    ]
    # Expansion joints echo the slab seams visible in the supplied flight video.
    joint_x = 10.0
    index = 0
    while joint_x < runway_length / 2.0 - 3.0:
        for sign in (-1.0, 1.0):
            x = sign * joint_x
            parts.append(
                f'<visual name="slab_joint_{index:02d}"><pose degrees="true">{x:.2f} 0 0.015 0 0 0</pose><cast_shadows>false</cast_shadows><geometry><box><size>0.035 {runway_width - 0.6:.2f} 0.003</size></box></geometry>{_material("0.44 0.45 0.44")}</visual>'
            )
            index += 1
        joint_x += 10.0
    for y in (-runway_width / 2.0 + 0.25, runway_width / 2.0 - 0.25):
        parts.append(
            f'<visual name="slab_edge_{index:02d}"><pose>0 {y:.2f} 0.015 0 0 0</pose><cast_shadows>false</cast_shadows><geometry><box><size>{runway_length - 0.5:.2f} 0.22 0.004</size></box></geometry>{_material("0.88 0.89 0.86")}</visual>'
        )
        index += 1
    # A runway centerline and paired threshold bars give the open pad readable
    # scale cues without adding collision geometry or costly shadow casters.
    parts.append(
        f'<visual name="runway_centerline"><pose>0 0 0.016 0 0 0</pose><cast_shadows>false</cast_shadows><geometry><box><size>{runway_length - 14.0:.2f} 0.18 0.004</size></box></geometry>{_material("0.91 0.91 0.87")}</visual>'
    )
    for sign in (-1.0, 1.0):
        x = sign * (runway_length / 2.0 - 5.0)
        for dash in (-1.0, 1.0):
            y = dash * 0.75
            parts.append(
                f'<visual name="threshold_{index:02d}"><pose>{x:.2f} {y:.2f} 0.016 0 0 0</pose><cast_shadows>false</cast_shadows><geometry><box><size>0.65 3.4 0.004</size></box></geometry>{_material("0.94 0.94 0.90")}</visual>'
            )
            index += 1
    parts.append("</link></model>")

    centers = {selected_group: (layout.target_center_north_m, layout.target_center_east_m)}
    other = "B" if selected_group == "A" else "A"
    # Put the unselected practice zone perpendicular to the scan axis, outside
    # its 60 m boundary, while remaining inside the route-derived terrain.
    perpendicular_n = math.sin(math.radians(layout.target_heading_deg))
    perpendicular_e = -math.cos(math.radians(layout.target_heading_deg))
    centers[other] = (
        layout.target_center_north_m + perpendicular_n * 110.0,
        layout.target_center_east_m + perpendicular_e * 110.0,
    )
    if group_centers is not None:
        centers=group_centers
    for group, (north, east) in centers.items():
        x, y = north, -east
        color = "0.10 0.28 0.82" if group == "A" else "0.78 0.12 0.10"
        c_yaw = -layout.target_heading_deg
        parts.append(
            f"""
    <model name="target_zone_{group}"><static>true</static><pose degrees="true">{x:.3f} {y:.3f} 0 0 0 {c_yaw:.4f}</pose><link name="zone">
      <visual name="practice_apron"><pose>0 0 0.005 0 0 0</pose><cast_shadows>false</cast_shadows><geometry><box><size>64 64 0.008</size></box></geometry>{_material('0.66 0.66 0.65', concrete, 0.97)}</visual>
      <visual name="zone_north"><pose>30 0 0.019 0 0 0</pose><cast_shadows>false</cast_shadows><geometry><box><size>0.24 60.2 0.025</size></box></geometry>{_material(color)}</visual>
      <visual name="zone_south"><pose>-30 0 0.019 0 0 0</pose><cast_shadows>false</cast_shadows><geometry><box><size>0.24 60.2 0.025</size></box></geometry>{_material(color)}</visual>
      <visual name="zone_side_a"><pose>0 30 0.019 0 0 0</pose><cast_shadows>false</cast_shadows><geometry><box><size>60.2 0.24 0.025</size></box></geometry>{_material(color)}</visual>
      <visual name="zone_side_b"><pose>0 -30 0.019 0 0 0</pose><cast_shadows>false</cast_shadows><geometry><box><size>60.2 0.24 0.025</size></box></geometry>{_material(color)}</visual>
    </link></model>
"""
        )
    return "\n".join(parts)
