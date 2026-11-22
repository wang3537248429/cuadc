from __future__ import annotations

import json
import math
from pathlib import Path
import pytest

from cuadc_sim.config import GAZEBO_WIND_FORCE_SCALE, SimParameters
from cuadc_sim.scenario import (
    MIN_SPACING_M,
    generate_scenario,
    minimum_spacing,
    validate_group,
    wind_vector,
    write_runtime,
    wells_to_sdf,
)
from cuadc_sim.selection import Detection, select_target


ROOT = Path(__file__).resolve().parents[1]


def test_functionally_grouped_target_assets_are_complete():
    if not (ROOT / "assets/targets/task1").exists():
        pytest.skip("源码包不分发靶标图案；此项需要本地素材包")
    catalog = json.loads(
        (ROOT / "assets/targets/catalog.json").read_text(encoding="utf-8")
    )
    for target in catalog["task1"]:
        assert (ROOT / target["icon"]).is_file()
        assert (ROOT / target["boards"]["blue"]).is_file()
        assert (ROOT / target["boards"]["red"]).is_file()
    for color in ("blue", "red"):
        assert (ROOT / catalog["empty_boards"][color]).is_file()
        for number in range(100):
            assert (
                ROOT
                / f"assets/targets/task2/{color}/number-{number:02d}.png"
            ).is_file()


def test_10000_deterministic_scenarios_follow_cuadc_rules():
    for seed in range(10_000):
        parameters = SimParameters(task=1 + seed % 2, group="A" if seed % 3 else "B", seed=seed)
        scenario = generate_scenario(parameters)
        for group in scenario.groups:
            assert not validate_group(group)
            assert len(group.wells) == 4
            assert sum(well.occupied for well in group.wells) == 3
            assert minimum_spacing(group.wells) > MIN_SPACING_M
            values = [well.value for well in group.wells if well.occupied]
            assert len(values) == 3
            assert len(set(values)) == 3
            if parameters.task == 1:
                assert all(1 <= value <= 12 for value in values)
            else:
                assert all(0 <= value <= 99 for value in values)
            detections = [
                Detection(
                    f"T{index:02d}",
                    int(well.value),
                    1.0,
                    north_m=well.north_m,
                    east_m=well.east_m,
                )
                for index, well in enumerate(group.wells, start=1)
                if well.occupied
            ]
            selected = select_target(parameters.task, detections)
            expected = max(values) if parameters.task == 1 else sorted(values)[1]
            assert selected.value == expected


def test_generation_is_reproducible_and_groups_are_independent():
    parameters = SimParameters(task=2, group="B", seed=2026)
    first = generate_scenario(parameters).to_dict()
    second = generate_scenario(parameters).to_dict()
    assert first == second
    assert first["groups"][0]["wells"] != first["groups"][1]["wells"]


def test_task_seeds_change_content_without_moving_wells():
    first=generate_scenario(SimParameters(task=1,seed=1))
    second=generate_scenario(SimParameters(task=1,seed=2))
    geometry=lambda scenario:[(well.id,well.north_m,well.east_m,well.arrow_rotation_deg,well.rotation_deg)
                              for group in scenario.groups for well in group.wells]
    assert geometry(first)==geometry(second)
    assert [well.value for g in first.groups for well in g.wells]!=[well.value for g in second.groups for well in g.wells]


def test_target_board_and_well_arrow_follow_task_orientation_rules():
    task_one = generate_scenario(SimParameters(task=1, seed=23))
    task_two = generate_scenario(SimParameters(task=2, seed=23))

    assert all(
        well.rotation_deg != well.arrow_rotation_deg
        for group in task_one.groups
        for well in group.wells
    )
    assert all(
        well.rotation_deg == well.arrow_rotation_deg
        for group in task_two.groups
        for well in group.wells
    )


def test_world_writer_keeps_selected_group_and_uses_real_assets(tmp_path: Path):
    if not (ROOT / "assets/targets/reference/task2").exists():
        pytest.skip("源码包不分发靶标图案；此项需要本地素材包")
    parameters = SimParameters(task=2, group="B", seed=17, wind_speed_mps=6, wind_from_deg=90)
    scenario, world_path = write_runtime(parameters, runtime_dir=tmp_path)
    payload = json.loads((tmp_path / "scenario.json").read_text(encoding="utf-8"))
    saved_parameters = json.loads(
        (tmp_path / "parameters.json").read_text(encoding="utf-8")
    )
    world = world_path.read_text(encoding="utf-8")
    assert payload["selected_group"] == "B"
    assert saved_parameters == parameters.to_dict()
    assert scenario.selected().name == "B"
    assert "@@SCENARIO_MODELS@@" not in world
    assert "@@SCENARIO_JSON@@" not in world
    assert "@@PLANE_YAW@@" not in world
    assert "@@GAZEBO_WIND_FORCE_SCALE@@" not in world
    assert str((Path(__file__).resolve().parents[1]/'assets/targets/reference/task2').resolve()) in world
    assert "/cuadc/scenario/state" in world
    assert "<linear_velocity>-0.000000 6.000000 0.000000</linear_velocity>" in world
    assert "<pose>0 0 1.55 0 0 3.141592654</pose>" in world
    assert (
        f"<force_approximation_scaling_factor>{GAZEBO_WIND_FORCE_SCALE:.6f}"
        "</force_approximation_scaling_factor>"
    ) in world


def test_world_writer_has_a_loadable_untextured_fallback_without_private_assets(
    tmp_path: Path,
):
    scenario = generate_scenario(SimParameters(task=2, seed=17))
    world = wells_to_sdf(scenario, root=tmp_path)
    assert "<visual name=\"target_surface\">" in world
    assert "<albedo_map>" not in world
    assert "<diffuse>1 1 1 1</diffuse>" in world


def test_group_a_aircraft_starts_facing_north(tmp_path: Path):
    _, world_path = write_runtime(
        SimParameters(task=1, group="A", seed=1),
        runtime_dir=tmp_path,
    )
    world = world_path.read_text(encoding="utf-8")
    assert world.count("<pose>0 0 1.55 0 0 0.000000000</pose>") == 2


def test_wind_direction_is_where_wind_comes_from():
    north = wind_vector(SimParameters(wind_speed_mps=5, wind_from_deg=0))
    east = wind_vector(SimParameters(wind_speed_mps=5, wind_from_deg=90))
    assert north == (-5.0, 0.0, 0.0)
    assert math.isclose(east[0], 0.0, abs_tol=1e-9)
    assert math.isclose(east[1], 5.0, abs_tol=1e-9)
