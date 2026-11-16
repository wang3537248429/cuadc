from __future__ import annotations

import argparse
import html
import json
import math
import random
import re
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable
from xml.sax.saxutils import escape

from .config import (
    GAZEBO_DIR,
    GAZEBO_WIND_FORCE_SCALE,
    ROOT,
    RUNTIME_DIR,
    SimParameters,
)
from .field_layout import write_field_layout
from .arena import load_arena
from .well_geometry import (FLOOR_THICKNESS_M, WALL_HEIGHT_M, WALL_THICKNESS_M,
                            floor_mesh_obj, wall_segments)
from .replay_route import field_surfaces_sdf
from .scenery import corner_flags_sdf


WELL_COUNT = 4
OCCUPIED_COUNT = 3
ZONE_HALF_SIZE_M = 30.0
MIN_SPACING_M = 30.0
GROUP_CENTERS_NE = {"A": (200.0, 0.0), "B": (-200.0, 0.0)}
GROUP_COLORS = {"A": "blue", "B": "red"}
_CORNER_OFFSET_M = 17.5



@dataclass(frozen=True)
class Well:
    id: str
    group: str
    occupied: bool
    north_m: float
    east_m: float
    rotation_deg: float
    arrow_rotation_deg: float
    value: int | None
    label: str | None
    asset: str

    def to_dict(self) -> dict:
        return asdict(self)


@dataclass(frozen=True)
class TargetGroup:
    name: str
    color: str
    center_north_m: float
    center_east_m: float
    wells: tuple[Well, ...]

    def to_dict(self) -> dict:
        data = asdict(self)
        data["wells"] = [well.to_dict() for well in self.wells]
        return data


@dataclass(frozen=True)
class Scenario:
    schema: str
    task: int
    seed: int
    selected_group: str
    groups: tuple[TargetGroup, ...]

    def selected(self) -> TargetGroup:
        return next(group for group in self.groups if group.name == self.selected_group)

    def to_dict(self) -> dict:
        return {
            "schema": self.schema,
            "task": self.task,
            "seed": self.seed,
            "selected_group": self.selected_group,
            "groups": [group.to_dict() for group in self.groups],
        }


def _task1_catalog(root: Path) -> list[dict]:
    manifest = json.loads((root / "assets/targets/catalog.json").read_text(encoding="utf-8"))
    catalog = manifest.get("task1", [])
    if len(catalog) != 12:
        raise ValueError("assets/targets/catalog.json must define exactly 12 Task 1 targets")
    return catalog


def _distance(left: Well, right: Well) -> float:
    return math.hypot(left.north_m - right.north_m, left.east_m - right.east_m)


def validate_group(group: TargetGroup) -> list[str]:
    errors: list[str] = []
    if len(group.wells) != WELL_COUNT:
        errors.append(f"{group.name}: expected {WELL_COUNT} wells")
    if sum(well.occupied for well in group.wells) != OCCUPIED_COUNT:
        errors.append(f"{group.name}: expected {OCCUPIED_COUNT} occupied wells")
    for well in group.wells:
        if abs(well.north_m - group.center_north_m) > ZONE_HALF_SIZE_M:
            errors.append(f"{well.id}: outside target zone north boundary")
        if abs(well.east_m - group.center_east_m) > ZONE_HALF_SIZE_M:
            errors.append(f"{well.id}: outside target zone east boundary")
    for left_index, left in enumerate(group.wells):
        for right in group.wells[left_index + 1 :]:
            spacing = _distance(left, right)
            if spacing <= MIN_SPACING_M:
                errors.append(f"{left.id}/{right.id}: spacing {spacing:.3f} <= {MIN_SPACING_M}")
    return errors


def generate_scenario(
    parameters: SimParameters,
    root: Path = ROOT,
    group_centers: dict[str, tuple[float, float]] | None = None,
    field_heading_deg: float = 0,
    layout_seed: int = 2025,
    fixed_wells: dict[str,dict] | None = None,
) -> Scenario:
    parameters.validate()
    catalog = _task1_catalog(root)
    groups: list[TargetGroup] = []
    for group_name in ("A", "B"):
        salt = 0xA511E9B3 if group_name == "A" else 0xB77294D1
        rng = random.Random((parameters.seed ^ salt ^ (parameters.task << 28)) & 0xFFFFFFFF)
        geometry_rng = random.Random(layout_seed ^ salt)
        center_north, center_east = (group_centers or GROUP_CENTERS_NE)[group_name]
        empty_index = rng.randrange(WELL_COUNT)
        occupied_indices = [index for index in range(WELL_COUNT) if index != empty_index]
        if parameters.task == 1:
            assignments: list[dict | int] = rng.sample(catalog, OCCUPIED_COUNT)
        else:
            assignments = rng.sample(range(100), OCCUPIED_COUNT)
        assignment_by_index = dict(zip(occupied_indices, assignments))
        corners = [
            (-_CORNER_OFFSET_M, -_CORNER_OFFSET_M),
            (-_CORNER_OFFSET_M, _CORNER_OFFSET_M),
            (_CORNER_OFFSET_M, -_CORNER_OFFSET_M),
            (_CORNER_OFFSET_M, _CORNER_OFFSET_M),
        ]
        geometry_rng.shuffle(corners)
        wells: list[Well] = []
        color = GROUP_COLORS[group_name]
        for index, (local_north, local_east) in enumerate(corners):
            occupied = index != empty_index
            local_north += geometry_rng.uniform(-0.8, 0.8)
            local_east += geometry_rng.uniform(-0.8, 0.8)
            angle = math.radians(field_heading_deg)
            north = center_north + local_north*math.cos(angle)-local_east*math.sin(angle)
            east = center_east + local_north*math.sin(angle)+local_east*math.cos(angle)
            assignment = assignment_by_index.get(index)
            if not occupied:
                value = None
                label = None
                asset = f"assets/targets/task1/{color}/empty.png"
            elif parameters.task == 1:
                assert isinstance(assignment, dict)
                value = int(assignment["value"])
                label = str(assignment["key"])
                asset = f"assets/targets/reference/task1/{assignment['key']}.png"
            else:
                assert isinstance(assignment, int)
                value = assignment
                label = f"{assignment:02d}"
                asset = f"assets/targets/reference/task2/number-{assignment:02d}.png"
            arrow_rotation = round(geometry_rng.uniform(0.0, 360.0), 4)
            board_rotation = round(geometry_rng.uniform(0.0, 360.0), 4)
            fixed=(fixed_wells or {}).get(f'{group_name}-{index+1:02d}')
            if fixed:
                north,east=float(fixed['north_m']),float(fixed['east_m'])
                arrow_rotation=float(fixed['arrow_rotation_deg'])
                board_rotation=float(fixed['rotation_deg'])
            target_rotation = (
                arrow_rotation
                if parameters.task == 2
                else board_rotation
            )
            wells.append(
                Well(
                    id=f"{group_name}-{index + 1:02d}",
                    group=group_name,
                    occupied=occupied,
                    north_m=round(north, 6),
                    east_m=round(east, 6),
                    rotation_deg=target_rotation,
                    arrow_rotation_deg=arrow_rotation,
                    value=value,
                    label=label,
                    asset=asset,
                )
            )
        group = TargetGroup(group_name, color, center_north, center_east, tuple(wells))
        errors = validate_group(group)
        if errors:
            raise RuntimeError("invalid generated scenario: " + "; ".join(errors))
        groups.append(group)
    return Scenario("cuadc-scenario.v2", parameters.task, parameters.seed, parameters.group, tuple(groups))


def _material(color: str, alpha: float = 1.0) -> str:
    return (
        f"<material><ambient>{color} {alpha}</ambient>"
        f"<diffuse>{color} {alpha}</diffuse><specular>0.06 0.06 0.06 {alpha}</specular>"
        "<pbr><metal><roughness>0.88</roughness><metalness>0</metalness></metal></pbr>"
        "</material>"
    )


def _target_outline_sdf(team_color: str) -> str:
    """Five 500 mm walls around the 1 m square and equilateral arrow head."""
    floor_geometry = "<geometry><mesh><uri>well_floor.obj</uri></mesh></geometry>"
    parts = [f"""
        <collision name="well_floor_collision">
          {floor_geometry}
          <surface><friction><ode><mu>0.76</mu><mu2>0.76</mu2></ode></friction></surface>
        </collision>
        <visual name="well_floor">
          {floor_geometry}
          {_material(team_color)}
        </visual>"""]
    for index, (x, y, length, yaw) in enumerate(wall_segments()):
        pose = f'<pose degrees="true">{x:.6f} {y:.6f} {WALL_HEIGHT_M / 2:.3f} 0 0 {yaw:.6f}</pose>'
        geometry = f"<geometry><box><size>{length:.6f} {WALL_THICKNESS_M:.3f} {WALL_HEIGHT_M:.3f}</size></box></geometry>"
        parts.append(f"""
        <collision name="well_wall_{index}_collision">
          {pose}{geometry}
          <surface><friction><ode><mu>0.76</mu><mu2>0.76</mu2></ode></friction></surface>
        </collision>
        <visual name="well_wall_{index}">
          {pose}{geometry}
          {_material("0.94 0.94 0.94")}
        </visual>""")
    return "".join(parts)


def _target_board_sdf(asset: str, rotation_deg: float) -> str:
    """Place a centered 600 mm panel, independently of the well heading."""
    # Use the original black/white reference panel with full UV coverage.
    # No crop, geometry or color is derived from the old blue composite image.
    return f"""
    <link name="target_board">
      <pose degrees="true">0 0 0 0 0 {rotation_deg:.4f}</pose>
      <collision name="target_board_collision">
        <pose>0 0 {FLOOR_THICKNESS_M + 0.0015:.4f} 0 0 0</pose>
        <geometry><box><size>0.600 0.600 0.003</size></box></geometry>
      </collision>
      <visual name="target_backing">
        <pose>0 0 {FLOOR_THICKNESS_M + 0.0015:.4f} 0 0 0</pose>
        <geometry><box><size>0.600 0.600 0.003</size></box></geometry>
        {_material("0.98 0.98 0.98")}
      </visual>
      <visual name="target_surface">
        <pose>0 0 {FLOOR_THICKNESS_M + 0.004:.3f} 0 0 0</pose>
        <geometry><plane><normal>0 0 1</normal><size>0.600 0.600</size></plane></geometry>
        <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>0.04 0.04 0.04 1</specular>
          <pbr><metal><albedo_map>{asset}</albedo_map><roughness>0.86</roughness><metalness>0</metalness></metal></pbr>
        </material>
      </visual>
    </link>"""


def wells_to_sdf(scenario: Scenario, root: Path = ROOT) -> str:
    chunks: list[str] = []
    for group in scenario.groups:
        team_color = "0.02 0.32 0.72" if group.name == "A" else "0.78 0.035 0.045"
        for well in group.wells:
            # Gazebo world: +X north, +Y west, +Z up.
            # Runtime worlds may live on another filesystem (for example RAM).
            # Resolve textures from the project, never from the world's parent.
            asset = escape(str((root / well.asset).resolve()))
            board = _target_board_sdf(asset, well.rotation_deg) if well.occupied else ""
            chunks.append(f"""
    <model name="{well.id}">
      <static>true</static>
      <pose>{well.north_m:.6f} {-well.east_m:.6f} 0 0 0 0</pose>
      <link name="well">
        <pose degrees="true">0 0 0 0 0 {well.arrow_rotation_deg:.4f}</pose>
        {_target_outline_sdf(team_color)}
      </link>
      {board}
    </model>""")
    return "\n".join(chunks)


def wind_vector(parameters: SimParameters) -> tuple[float, float, float]:
    angle = math.radians(parameters.wind_from_deg)
    north = -parameters.wind_speed_mps * math.cos(angle)
    east = -parameters.wind_speed_mps * math.sin(angle)
    return north, -east, 0.0


def write_runtime(
    parameters: SimParameters,
    root: Path = ROOT,
    runtime_dir: Path = RUNTIME_DIR,
    replay_data: dict | None = None,
) -> tuple[Scenario, Path]:
    # replay_data is retained for callers migrating older saved missions;
    # uploaded route geometry is deliberately never used to build the world.
    arena=load_arena(root)
    layout=arena.layout
    scenario=generate_scenario(parameters,root,arena.centers,layout.target_heading_deg,
                               arena.layout_seed,arena.wells)
    gazebo_dir = GAZEBO_DIR if root == ROOT else root
    template = (gazebo_dir / "worlds/cuadc_world.sdf.in").read_text(encoding="utf-8")
    wind = wind_vector(parameters)
    # Point the restrained aircraft toward the selected target before
    # ArduPlane starts.  Changing a 180-degree heading during the one-shot
    # release can leave the JSON backend observing the old attitude for the
    # launch impulse; making it part of the physical initial condition avoids
    # that race and lets the controller settle while the joint is still held.
    # Mission headings are compass headings (north/east), while Gazebo's
    # local +Y axis is the westward coordinate used by the generated field.
    # Negate uploaded-route headings when placing the model so its nose and
    # the hand-launch impulse point along the first route leg in world axes.
    plane_yaw_rad = (
        -math.radians(layout.initial_heading_deg)
        if layout.initial_heading_deg
        else (0.0 if parameters.group == "A" else math.pi)
    )
    field_sdf = field_surfaces_sdf(layout, parameters.group,
                                  {group.name:(group.center_north_m,group.center_east_m) for group in scenario.groups})
    world = (
        template.replace("@@SCENARIO_MODELS@@", wells_to_sdf(scenario, root) + corner_flags_sdf(scenario))
        .replace("@@FIELD_SURFACES@@", field_sdf)
        .replace("@@ORIGIN_LAT@@", f"{layout.origin_lat_deg:.10f}")
        .replace("@@ORIGIN_LON@@", f"{layout.origin_lon_deg:.10f}")
        .replace("@@ORIGIN_ALT@@", f"{layout.origin_alt_m:.3f}")
        .replace("@@WIND_VECTOR@@", f"{wind[0]:.6f} {wind[1]:.6f} {wind[2]:.6f}")
        .replace(
            "@@GAZEBO_WIND_FORCE_SCALE@@",
            f"{GAZEBO_WIND_FORCE_SCALE:.6f}",
        )
        .replace("@@PLANE_YAW@@", f"{plane_yaw_rad:.9f}")
        .replace(
            "@@SCENARIO_JSON@@",
            html.escape(
                json.dumps(
                    scenario.to_dict(),
                    ensure_ascii=False,
                    separators=(",", ":"),
                ),
                quote=True,
            ),
        )
    )
    # Generated surfaces are shared by default and uploaded-route scenarios.
    world = re.sub(
        r'\n    <model name="legacy_(?:ground|takeoff_landing_area|target_zone_A|target_zone_B)">.*?</model>',
        "", world, flags=re.DOTALL,
    )
    runtime_dir.mkdir(parents=True, exist_ok=True)
    (runtime_dir / "well_floor.obj").write_text(floor_mesh_obj(), encoding="utf-8")
    world_path = runtime_dir / "cuadc_world.sdf"
    world_path.write_text(world, encoding="utf-8")
    (runtime_dir / "scenario.json").write_text(
        json.dumps(scenario.to_dict(), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    write_field_layout(layout, runtime_dir / "field_layout.json")
    (runtime_dir / "parameters.json").write_text(
        json.dumps(parameters.to_dict(), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return scenario, world_path


def minimum_spacing(wells: Iterable[Well]) -> float:
    items = list(wells)
    return min(
        _distance(left, right)
        for index, left in enumerate(items)
        for right in items[index + 1 :]
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Generate deterministic CUADC Gazebo world")
    parser.add_argument("--task", type=int, choices=(1, 2), default=1)
    parser.add_argument("--group", choices=("A", "B"), default="A")
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--wind-speed", type=float, default=0.0)
    parser.add_argument("--wind-direction", type=float, default=0.0)
    args = parser.parse_args(argv)
    params = SimParameters(
        task=args.task,
        group=args.group,
        seed=args.seed,
        wind_speed_mps=args.wind_speed,
        wind_from_deg=args.wind_direction,
    )
    scenario, world = write_runtime(params)
    print(json.dumps({"world": str(world), "scenario": scenario.to_dict()}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
