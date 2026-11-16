from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

from cuadc_sim.config import SimParameters
from cuadc_sim.scenario import generate_scenario


ROOT = Path(__file__).resolve().parents[1]
SPEC = spec_from_file_location(
    "run_acceptance_matrix", ROOT / "scripts/run_acceptance_matrix.py"
)
assert SPEC is not None and SPEC.loader is not None
SCORER = module_from_spec(SPEC)
SPEC.loader.exec_module(SCORER)


def test_truth_target_is_used_only_by_offline_acceptance_scorer():
    for task in (1, 2):
        scenario = generate_scenario(
            SimParameters(task=task, group="A", seed=17)
        ).to_dict()
        target = SCORER.true_target(scenario, task, "A")
        values = sorted(
            well["value"]
            for well in scenario["groups"][0]["wells"]
            if well["occupied"]
        )
        assert target["value"] == (max(values) if task == 1 else values[1])


def test_acceptance_score_enforces_physical_and_flight_gates():
    scenario = generate_scenario(
        SimParameters(task=1, group="A", seed=1)
    ).to_dict()
    target = SCORER.true_target(scenario, 1, "A")
    result = {
        "task": 1,
        "group": "A",
        "status": "COMPLETED",
        "detections": [
            {
                "value": well["value"],
                "north_m": well["north_m"],
                "east_m": well["east_m"],
            }
            for well in scenario["groups"][0]["wells"]
            if well["occupied"]
        ],
        "release": {"physical_confirmed_at": 1.0},
        "impact": {"north_m": target["north_m"], "east_m": target["east_m"]},
        "first_touchdown": {"in_takeoff_landing_zone": True},
        "final_stop": {"in_takeoff_landing_zone": True},
        "route_metrics": {
            "max_home_distance_m": 299.0,
            "mission_time_s": 119.0,
            "max_roll_deg": 34.0,
            "min_airspeed_mps": 13.1,
        },
    }
    score = SCORER.score_case(result, scenario, 0.0)
    assert score["hard_pass"]
    assert score["precision_3m"]
    assert score["official_drop_points"] == 500


def test_official_drop_score_uses_three_and_five_metre_zones():
    scenario = generate_scenario(
        SimParameters(task=1, group="A", seed=1)
    ).to_dict()
    target = SCORER.true_target(scenario, 1, "A")
    result = {
        "task": 1,
        "group": "A",
        "status": "COMPLETED",
        "detections": [
            {
                "value": well["value"],
                "north_m": well["north_m"],
                "east_m": well["east_m"],
            }
            for well in scenario["groups"][0]["wells"]
            if well["occupied"]
        ],
        "impact": {"north_m": target["north_m"], "east_m": target["east_m"]},
        "route_metrics": {"mission_time_s": 480.0},
    }
    assert SCORER.score_case(result, scenario, 0.0)["official_drop_points"] == 500
    result["impact"] = {"north_m": target["north_m"] + 4.0, "east_m": target["east_m"]}
    assert SCORER.score_case(result, scenario, 0.0)["official_drop_points"] == 300
    result["impact"] = {"north_m": target["north_m"] + 10.0, "east_m": target["east_m"]}
    assert SCORER.score_case(result, scenario, 0.0)["official_drop_points"] == 50
    result["impact"] = {"north_m": target["north_m"] + 40.0, "east_m": target["east_m"]}
    assert SCORER.score_case(result, scenario, 0.0)["official_drop_points"] == 0
    result["route_metrics"]["mission_time_s"] = 480.1
    assert not SCORER.score_case(result, scenario, 0.0)["checks"]["simulated_mission_within_480s"]
