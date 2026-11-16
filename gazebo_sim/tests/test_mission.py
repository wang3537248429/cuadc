from __future__ import annotations

import math
from pathlib import Path

import pytest
from pymavlink import mavutil

from cuadc_sim.config import (
    MISSION_MAX_HOME_DISTANCE_M,
    MISSION_PLANNING_AIRSPEED_MPS,
    MISSION_PLANNING_BANK_DEG,
    PAYLOAD_NOMINAL_AGL_M,
    PAYLOAD_RELEASE_PWM,
    PAYLOAD_SERVO,
    SCAN_END_TAG,
    SCAN_START_TAG,
)
from cuadc_sim.mission import (
    MISSION_MODE_BLOCKED,
    MISSION_MODE_DYNAMIC_READY,
    MISSION_MODE_FIXED_FALLBACK,
    MissionItem,
    assess_mission,
    build_attack_patch,
    build_descent_side_patch,
    coordinated_turn_radius,
    crossed_scan_end,
    derive_scan_rectangle,
    latlon_to_ne,
    load_patch_backup,
    save_patch_backup,
    mission_route_metrics,
    scan_coverage_cells,
    update_scan_coverage,
    validate_mission,
)


def load_qgc(path: Path) -> list[MissionItem]:
    result = []
    lines = path.read_text(encoding="utf-8").splitlines()
    assert lines[0] == "QGC WPL 110"
    for line in lines[1:]:
        if not line.strip():
            continue
        fields = line.split("\t")
        result.append(
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
    return result


@pytest.mark.parametrize(
    ("template", "target_north_values"),
    (("group_a.waypoints", (182.0, 218.0)), ("group_b.waypoints", (-218.0, -182.0))),
)
def test_ptk_350ml_nominal_attack_route_fits_all_target_corners(
    template, target_north_values
):
    root = Path(__file__).resolve().parents[1]
    items = load_qgc(root / "config" / "missions" / template)
    contract, errors = validate_mission(items)
    assert contract is not None and not errors
    decision_north, decision_east = latlon_to_ne(
        items[contract.decision_seq].x,
        items[contract.decision_seq].y,
    )
    scan = [
        latlon_to_ne(item.x, item.y)
        for item in items
        if (
            contract.scan_start_seq < item.seq < contract.scan_end_seq
            and item.command == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
        )
    ]
    outbound_north = scan[-1][0] - scan[0][0]
    outbound_east = scan[-1][1] - scan[0][1]
    for target_north in target_north_values:
        for target_east in (-18.0, 18.0):
            patch = build_attack_patch(
                items,
                contract,
                target_north,
                target_east,
                decision_north,
                decision_east,
                nominal_speed_mps=MISSION_PLANNING_AIRSPEED_MPS,
                nominal_agl_m=PAYLOAD_NOMINAL_AGL_M,
                max_home_distance_m=MISSION_MAX_HOME_DISTANCE_M,
                planning_bank_deg=MISSION_PLANNING_BANK_DEG,
                outbound_north=outbound_north,
                outbound_east=outbound_east,
                attack_altitude_limit_m=None,
            )
            assert len(patch) == 4
            assert 0.0 < patch[0].param1 < 1.0
            center_north, center_east = latlon_to_ne(
                patch[0].x, patch[0].y
            )
            radial_north = decision_north - center_north
            radial_east = decision_east - center_east
            if patch[0].param3 > 0.0:
                tangent_north, tangent_east = -radial_east, radial_north
            else:
                tangent_north, tangent_east = radial_east, -radial_north
            tangent_length = math.hypot(tangent_north, tangent_east)
            outbound_length = math.hypot(outbound_north, outbound_east)
            assert (
                tangent_north * outbound_north
                + tangent_east * outbound_east
            ) / (tangent_length * outbound_length) > 0.999


@pytest.mark.parametrize(
    "template",
    (
        "group_a.waypoints",
        "group_b.waypoints",
        "route_active_mode_sim.waypoints",
    ),
)
def test_supplied_mission_template_satisfies_contract(template):
    root = Path(__file__).resolve().parents[1]
    items = load_qgc(root / "config" / "missions" / template)
    contract, errors = validate_mission(items)
    assert not errors
    assert contract is not None
    assert items[contract.scan_start_seq].param1 == SCAN_START_TAG
    assert items[contract.scan_end_seq].param1 == SCAN_END_TAG
    assert items[contract.fallback_release_seq].param1 == PAYLOAD_SERVO


def test_calibrated_land_commands_are_on_runway_centerline():
    root = Path(__file__).resolve().parents[1]
    north_items = load_qgc(root / "config/missions/group_a.waypoints")
    south_items = load_qgc(root / "config/missions/group_b.waypoints")
    north_land_m, north_east_m = latlon_to_ne(
        north_items[-1].x,
        north_items[-1].y,
    )
    south_land_m, south_east_m = latlon_to_ne(
        south_items[-1].x,
        south_items[-1].y,
    )
    # ArduPlane's flare aim is commanded before the physical touchdown.  The
    # simulation acceptance check below uses the measured first-contact and
    # final-stop positions, rather than pretending this navigation aim point
    # is the wheel-contact point.
    # ArduPlane's flare / glide continues well beyond NAV_LAND.  The symmetric
    # upstream aim was calibrated against the Gazebo contact sensor so first
    # wheel contact, rather than an altitude estimate, occurs near home.
    assert 99.0 < north_land_m < 101.0
    assert -101.0 < south_land_m < -99.0
    assert abs(north_land_m + south_land_m) < 0.1
    assert abs(north_east_m) < 0.01
    assert abs(south_east_m) < 0.01
    for items in (north_items, south_items):
        final = [latlon_to_ne(item.x, item.y) for item in items[-3:]]
        assert all(abs(east_m) < 0.01 for _north_m, east_m in final)


def test_templates_use_one_centerline_scan_and_stay_inside_300m():
    root = Path(__file__).resolve().parents[1]
    for name in ("group_a.waypoints", "group_b.waypoints"):
        items = load_qgc(root / "config/missions" / name)
        contract, errors = validate_mission(items)
        assert not errors and contract is not None
        scan = [
            item
            for item in items
            if contract.scan_start_seq < item.seq < contract.scan_end_seq
            and item.command == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
        ]
        assert len(scan) == 2
        points = [latlon_to_ne(item.x, item.y) for item in scan]
        assert all(abs(east) < 0.01 for _, east in points)
        assert abs(points[1][0] - points[0][0]) == pytest.approx(90.0, abs=0.01)
        norths = sorted(north for north, _east in points)
        if name == "group_b.waypoints":
            assert norths[0] <= -230.0
            assert norths[1] >= -170.0
        else:
            assert norths[0] <= 170.0
            assert norths[1] >= 230.0
        guard_north, guard_east = latlon_to_ne(
            items[contract.decision_seq].x,
            items[contract.decision_seq].y,
        )
        # The guard continues straight beyond the complete target zone. It
        # must never sit behind the aircraft and induce the old unsafe snap
        # reversal at the end of the camera pass.
        assert abs(guard_east) < 0.01
        assert abs(guard_north) > max(abs(north) for north, _east in points)
        assert math.copysign(1.0, guard_north) == math.copysign(1.0, points[-1][0])
        metrics = mission_route_metrics(items)
        assert metrics["nominal_length_m"] <= 950.0
        assert metrics["max_home_distance_m"] <= 300.0


def test_invalid_mission_is_blocked():
    items = [MissionItem(0, 3, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT)]
    contract, errors = validate_mission(items)
    assert contract is None
    assert len(errors) == 2


def test_scan_end_crossing_survives_transient_tag_being_skipped_once():
    assert crossed_scan_end(6, 8, 7)
    assert not crossed_scan_end(8, 9, 7)


def test_dynamic_attack_patch_is_contiguous_and_latches_release_high():
    root = Path(__file__).resolve().parents[1]
    items = load_qgc(root / "config/missions/group_a.waypoints")
    contract, _ = validate_mission(items)
    assert contract is not None
    patch = build_attack_patch(items, contract, 217.0, -16.0, 200.0, 0.0, nominal_agl_m=30)
    assert [item.seq for item in patch] == list(
        range(contract.fallback_start_seq, contract.fallback_exit_seq + 1)
    )
    assert [item.command for item in patch] == [
        mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
    ]
    assert patch[2].param1 == PAYLOAD_SERVO
    assert patch[2].param2 == PAYLOAD_RELEASE_PWM
    assert 0.0 < patch[0].param1 < 1.0
    assert abs(patch[0].param3) == pytest.approx(coordinated_turn_radius(16.0))
    assert patch[0].param4 == 1
    assert patch[1].param2 == 3
    attack_north, attack_east = latlon_to_ne(patch[1].x, patch[1].y)
    exit_north, exit_east = latlon_to_ne(patch[3].x, patch[3].y)
    attack_vector = (attack_north - 217.0, attack_east + 16.0)
    exit_vector = (exit_north - 217.0, exit_east + 16.0)
    assert math.hypot(*attack_vector) == pytest.approx(0.0, abs=0.01)
    assert math.hypot(*exit_vector) > math.hypot(*attack_vector)
    assert math.hypot(
        exit_north - attack_north,
        exit_east - attack_east,
    ) == pytest.approx(30.0, abs=0.01)


def test_dynamic_slots_have_strict_json_crash_recovery_backup(tmp_path):
    root = Path(__file__).resolve().parents[1]
    items = load_qgc(root / "config/missions/group_a.waypoints")
    contract, errors = validate_mission(items)
    assert not errors and contract is not None
    backup_path = tmp_path / "mission_patch_backup.json"
    slots = items[contract.fallback_start_seq : contract.fallback_exit_seq + 1]
    save_patch_backup(backup_path, slots)
    raw = backup_path.read_text(encoding="utf-8")
    assert "NaN" not in raw
    restored = load_patch_backup(backup_path)
    assert restored == slots


def test_low_threshold_assessment_degrades_bad_dynamic_turn_to_fixed_fallback():
    root = Path(__file__).resolve().parents[1]
    items = load_qgc(root / "config/missions/group_a.waypoints")
    items[7] = MissionItem(
        **{**items[7].__dict__, "param1": 0.8, "param3": -10.0}
    )
    assessment = assess_mission(items, min_release_pwm=PAYLOAD_RELEASE_PWM)
    assert assessment.mode == MISSION_MODE_FIXED_FALLBACK
    assert assessment.launch_allowed
    assert assessment.contract is not None
    assert assessment.warnings


def test_low_threshold_assessment_blocks_route_that_cannot_drop():
    root = Path(__file__).resolve().parents[1]
    items = load_qgc(root / "config/missions/group_a.waypoints")
    items[9] = MissionItem(
        **{**items[9].__dict__, "command": mavutil.mavlink.MAV_CMD_NAV_DELAY}
    )
    assessment = assess_mission(items, min_release_pwm=PAYLOAD_RELEASE_PWM)
    assert assessment.mode == MISSION_MODE_BLOCKED
    assert not assessment.launch_allowed
    assert any("DO_SET_SERVO" in error for error in assessment.errors)


def test_complete_template_is_dynamic_ready_and_coverage_uses_camera_health():
    root = Path(__file__).resolve().parents[1]
    items = load_qgc(root / "config/missions/group_a.waypoints")
    assessment = assess_mission(items, min_release_pwm=PAYLOAD_RELEASE_PWM)
    assert assessment.mode == MISSION_MODE_DYNAMIC_READY
    assert assessment.contract is not None
    cells = scan_coverage_cells(items, assessment.contract)
    assert len(cells) > 10
    covered: set[int] = set()
    north_m, east_m, altitude_m = cells[0]
    assert update_scan_coverage(
        cells,
        covered,
        north_m,
        east_m,
        altitude_m,
        camera_age_s=4.0,
    ) == 0.0
    assert update_scan_coverage(
        cells,
        covered,
        north_m,
        east_m,
        altitude_m,
        camera_age_s=0.2,
    ) > 0.0


def test_rectangle_route_matches_real_aircraft_coverage_contract():
    root = Path(__file__).resolve().parents[1]
    items = load_qgc(root / "config/missions/group_a_rectangle.waypoints")
    assessment = assess_mission(
        items,
        min_release_pwm=PAYLOAD_RELEASE_PWM,
        origin_lat_deg=items[0].x,
        origin_lon_deg=items[0].y,
        max_home_distance_m=300.0,
    )
    assert assessment.mode == MISSION_MODE_DYNAMIC_READY
    assert assessment.contract is not None
    rectangle = derive_scan_rectangle(items, assessment.contract)
    assert rectangle is not None
    assert rectangle.lane1_start_seq == 3
    assert rectangle.lane1_end_seq == 4
    assert rectangle.lane2_start_seq == 8
    assert rectangle.lane2_end_seq == 9
    assert rectangle.turn_waypoint_seqs == (5, 6, 7)
    assert rectangle.length_m == pytest.approx(105.0, abs=0.02)
    assert rectangle.width_m == pytest.approx(32.0, abs=0.02)
    cells = scan_coverage_cells(items, assessment.contract)
    assert len(cells) == 44
    assert max(north for north, _east, _alt in cells) < 246.0
    assert min(north for north, _east, _alt in cells) > 139.0


def test_active_fallback_route_keeps_mandatory_servo_release():
    root = Path(__file__).resolve().parents[1]
    items = load_qgc(
        root / "config/missions/route_active_mode_sim.waypoints"
    )
    assessment = assess_mission(
        items,
        min_release_pwm=PAYLOAD_RELEASE_PWM,
        origin_lat_deg=items[0].x,
        origin_lon_deg=items[0].y,
        max_home_distance_m=300.0,
    )
    assert assessment.mode == MISSION_MODE_DYNAMIC_READY
    assert assessment.contract is not None
    assert assessment.contract.fallback_release_seq == 23
    release = items[assessment.contract.fallback_release_seq]
    assert release.command == mavutil.mavlink.MAV_CMD_DO_SET_SERVO
    assert release.param1 == PAYLOAD_SERVO
    assert release.param2 == PAYLOAD_RELEASE_PWM


def test_descent_route_mirrors_to_estimated_target_side():
    root = Path(__file__).resolve().parents[1]
    items = load_qgc(root / "config/missions/group_a.waypoints")
    contract, errors = validate_mission(items)
    assert not errors and contract is not None
    east_patch = build_descent_side_patch(items, contract, 17.0)
    west_patch = build_descent_side_patch(items, contract, -17.0)
    expected_sequences = list(
        range(contract.fallback_exit_seq + 1, contract.land_seq + 1)
    )
    assert [item.seq for item in east_patch] == expected_sequences
    assert [item.seq for item in west_patch] == expected_sequences
    for east_item, west_item in zip(east_patch, west_patch, strict=True):
        east_north, east_east = latlon_to_ne(east_item.x, east_item.y)
        west_north, west_east = latlon_to_ne(west_item.x, west_item.y)
        assert east_north == pytest.approx(west_north, abs=0.01)
        assert east_east == pytest.approx(-west_east, abs=0.01)
        if east_item.seq == contract.land_seq:
            assert east_east == pytest.approx(0.0, abs=0.01)
            assert west_east == pytest.approx(0.0, abs=0.01)
        else:
            assert east_east >= -0.01


def test_high_speed_turn_is_rejected_when_it_cannot_fit_300m_boundary():
    root = Path(__file__).resolve().parents[1]
    items = load_qgc(root / "config/missions/group_a.waypoints")
    contract, _ = validate_mission(items)
    assert contract is not None
    with pytest.raises(ValueError, match="home boundary"):
        build_attack_patch(
            items,
            contract,
            200.0,
            0.0,
            230.0,
            0.0,
            nominal_speed_mps=22.0,
        )
