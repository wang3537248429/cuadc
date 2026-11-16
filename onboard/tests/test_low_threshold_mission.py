from __future__ import annotations

from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest
from pymavlink import mavutil

from cuadc_onboard.config import RuntimeConfig, load_config
from cuadc_onboard.mission import (
    MISSION_MODE_BLOCKED,
    MISSION_MODE_DYNAMIC_READY,
    MISSION_MODE_FIXED_FALLBACK,
    FixedFallbackContract,
    MissionItem,
    assess_mission,
    derive_scan_rectangle,
    load_qgc_waypoints,
    ne_to_latlon,
    scan_coverage_cells,
)
from cuadc_onboard.runtime import OnboardRuntime


ROOT = Path(__file__).resolve().parents[1]


def active_test_config():
    """Synthetic calibrated profile for unit tests; never an installed device config."""
    base = load_config(ROOT / "config/onboard.yaml")
    return replace(
        base,
        camera=replace(base.camera, calibration_confirmed=True),
        vision=replace(base.vision, validation_confirmed=True),
        release=replace(base.release, enabled=True, ballistics_calibrated=True, precision_required=False),
    )


def active_fixture() -> list[MissionItem]:
    mission = load_qgc_waypoints(ROOT / "tests/fixtures/group_a.waypoints")
    mission[9] = replace(mission[9], param1=8.0, param2=1800.0)
    return mission


def test_complete_route_is_dynamic_ready_for_real_servo_configuration():
    assessment = assess_mission(
        active_fixture(),
        payload_servo=8,
        min_release_pwm=1800.0,
    )
    assert assessment.mode == MISSION_MODE_DYNAMIC_READY
    assert assessment.launch_allowed


def test_adjustable_rectangle_route_is_dynamic_ready_and_detected():
    mission = load_qgc_waypoints(
        ROOT / "config/demo_rectangle.waypoints"
    )
    assessment = assess_mission(
        mission,
        payload_servo=8,
        min_release_pwm=1800.0,
        origin_lat_deg=mission[0].x,
        origin_lon_deg=mission[0].y,
        max_home_distance_m=300.0,
    )
    assert assessment.mode == MISSION_MODE_DYNAMIC_READY
    assert assessment.launch_allowed
    rectangle = derive_scan_rectangle(
        mission,
        assessment.contract,
        origin_lat_deg=mission[0].x,
        origin_lon_deg=mission[0].y,
    )
    assert rectangle is not None
    assert rectangle.lane1_start_seq == 7
    assert rectangle.lane1_end_seq == 8
    assert rectangle.lane2_start_seq == 12
    assert rectangle.lane2_end_seq == 13
    assert rectangle.turn_waypoint_seqs == (9, 10, 11)
    assert rectangle.length_m == pytest.approx(165.0, abs=0.2)
    assert rectangle.width_m == pytest.approx(32.0, abs=0.2)
    assert rectangle.heading_deg == pytest.approx(0.0, abs=0.2)


def test_rectangle_coverage_excludes_outside_turn_helpers():
    mission = load_qgc_waypoints(
        ROOT / "config/demo_rectangle.waypoints"
    )
    assessment = assess_mission(
        mission,
        payload_servo=8,
        min_release_pwm=1800.0,
    )
    cells = scan_coverage_cells(
        mission,
        assessment.contract,
        origin_lat_deg=mission[0].x,
        origin_lon_deg=mission[0].y,
    )
    # 165 m / 5 m gives 34 samples on each of the two long lanes.
    assert len(cells) == 68
    assert max(north for north, _east, _alt in cells) < 72.0
    assert min(north for north, _east, _alt in cells) > -95.0
    assert {round(east) for _north, east, _alt in cells} == {24, 56}


def test_rectangle_has_no_twenty_metre_lane_length_gate():
    mission = load_qgc_waypoints(
        ROOT / "config/demo_rectangle.waypoints"
    )
    home_lat, home_lon = mission[0].x, mission[0].y
    short_corners = {
        7: (0.0, 10.0),
        8: (5.0, 10.0),
        12: (5.0, 15.0),
        13: (0.0, 15.0),
    }
    mission = [
        replace(
            item,
            x=ne_to_latlon(
                *short_corners[item.seq],
                origin_lat_deg=home_lat,
                origin_lon_deg=home_lon,
            )[0],
            y=ne_to_latlon(
                *short_corners[item.seq],
                origin_lat_deg=home_lat,
                origin_lon_deg=home_lon,
            )[1],
        )
        if item.seq in short_corners
        else item
        for item in mission
    ]
    assessment = assess_mission(
        mission,
        payload_servo=8,
        min_release_pwm=1800.0,
    )
    rectangle = derive_scan_rectangle(
        mission,
        assessment.contract,
        origin_lat_deg=home_lat,
        origin_lon_deg=home_lon,
    )
    assert rectangle is not None
    assert rectangle.length_m == pytest.approx(5.0, abs=0.01)


def test_bad_dynamic_turn_degrades_to_fixed_release_instead_of_blocking():
    mission = active_fixture()
    mission[7] = replace(mission[7], param1=0.8, param3=-10.0)
    assessment = assess_mission(
        mission,
        payload_servo=8,
        min_release_pwm=1800.0,
    )
    assert assessment.mode == MISSION_MODE_FIXED_FALLBACK
    assert assessment.launch_allowed
    assert assessment.warnings


def test_missing_release_is_blocked_because_mandatory_drop_cannot_finish():
    mission = active_fixture()
    mission[9] = replace(
        mission[9], command=mavutil.mavlink.MAV_CMD_NAV_DELAY
    )
    assessment = assess_mission(
        mission,
        payload_servo=8,
        min_release_pwm=1800.0,
    )
    assert assessment.mode == MISSION_MODE_BLOCKED
    assert not assessment.launch_allowed


def test_runtime_keeps_uploaded_route_when_scan_is_incomplete(tmp_path):
    base = active_test_config()
    runtime = OnboardRuntime(
        replace(base, runtime=RuntimeConfig(directory=str(tmp_path)))
    )
    runtime.contract = FixedFallbackContract(2, 5, 3, 8, 12)
    runtime.scan_coverage_ratio = 0.40
    runtime.vision.snapshot = lambda: {"updated_at": 0.0, "detections": []}
    requested: list[int] = []
    runtime.client = SimpleNamespace(
        set_current=requested.append,
        status_text=lambda *_args: None,
    )

    runtime._handle_scan_end()
    assert requested == []
    assert runtime.scan_attempt == 1
    assert runtime.state["status"] == "FALLBACK_ROUTE_ACTIVE"
    assert runtime.state["scan_fallback_reason"]


def test_mission_current_backward_jump_rearms_second_scan_end(tmp_path):
    base = active_test_config()
    runtime = OnboardRuntime(
        replace(base, runtime=RuntimeConfig(directory=str(tmp_path)))
    )
    runtime.contract = FixedFallbackContract(2, 5, 3, 8, 12)
    runtime.client = SimpleNamespace(target_system=1, target_component=1)
    scan_ends: list[int] = []
    runtime._handle_scan_end = lambda: scan_ends.append(
        runtime.telemetry.mission_seq
    )

    class MissionCurrent:
        def __init__(self, seq: int):
            self.seq = seq

        @staticmethod
        def get_type() -> str:
            return "MISSION_CURRENT"

        @staticmethod
        def get_srcSystem() -> int:
            return 1

        @staticmethod
        def get_srcComponent() -> int:
            return mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1

    runtime._handle_message(MissionCurrent(5))
    runtime._handle_message(MissionCurrent(3))
    runtime._handle_message(MissionCurrent(5))

    assert scan_ends == [5, 5]
