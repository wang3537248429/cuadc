from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]


def test_airframe_uses_physical_gazebo_actuators_and_no_pose_commands():
    from cuadc_sim.config import (
        AIRCRAFT_EMPTY_MASS_KG,
        AIRCRAFT_FUSELAGE_MASS_KG,
        AIRCRAFT_LAUNCH_MASS_KG,
        GROUND_BRAKE_FORCE_N,
        MAX_MOTOR_SHAFT_SPEED_RAD_S,
        MAX_THRUST_PER_MOTOR_N,
        MOTOR_COUNT,
        MOTOR_THRUST_COEFFICIENT,
        PAYLOAD_ACTUATOR_DELAY_S,
        PAYLOAD_TOTAL_MASS_KG,
    )

    model = (ROOT / "models/cuadc_twin/model.sdf").read_text()
    assert model.count("gz-sim-joint-position-controller-system") == 4
    assert model.count("<type>COMMAND</type>") >= 6
    assert "PoseCmd" not in model
    assert f"<mass>{AIRCRAFT_FUSELAGE_MASS_KG:.3f}</mass>" in model
    assert f"<launch_mass>{AIRCRAFT_LAUNCH_MASS_KG:.3f}</launch_mass>" in model
    assert f"<max_thrust_per_motor>{MAX_THRUST_PER_MOTOR_N:.4f}</max_thrust_per_motor>" in model
    assert f"<thrust_coefficient>{MOTOR_THRUST_COEFFICIENT:.10f}</thrust_coefficient>" in model
    assert f"<ground_brake_force>{GROUND_BRAKE_FORCE_N:.1f}</ground_brake_force>" in model
    assert f"<mass>{PAYLOAD_TOTAL_MASS_KG:.2f}</mass>" in model
    assert MOTOR_COUNT * MAX_THRUST_PER_MOTOR_N == pytest.approx(34.323275)
    assert MOTOR_THRUST_COEFFICIENT * MAX_MOTOR_SHAFT_SPEED_RAD_S**2 == pytest.approx(
        MAX_THRUST_PER_MOTOR_N
    )
    assert AIRCRAFT_EMPTY_MASS_KG == pytest.approx(2.5)
    assert "\"THR_MAX\": 100.0" in (ROOT / "src/cuadc_sim/config.py").read_text()
    assert f"<delay_s>{PAYLOAD_ACTUATOR_DELAY_S:.2f}</delay_s>" in model
    assert model.count('name="ribbon_') >= 6
    assert model.count('name="ribbon_joint_') == 6
    assert model.count('type="universal"') == 6
    assert model.count("<damping>0.05</damping>") == 12
    assert model.count("<velocity>8</velocity>") == 12
    assert model.count("<lower>-1.20</lower>") == 12
    assert model.count("<velocity_decay>") == 6
    assert '<collision name="c">' not in model
    assert 'name="airframe_contact" type="contact"' in model


def test_aircraft_empty_mass_sums_all_flight_model_links():
    import xml.etree.ElementTree as ET

    from cuadc_sim.config import AIRCRAFT_EMPTY_MASS_KG

    model = ET.parse(ROOT / "models/cuadc_twin/model.sdf").getroot().find("model")
    assert model is not None
    mass = sum(
        float(link.findtext("./inertial/mass"))
        for link in model.findall("link")
        if link.findtext("./inertial/mass") is not None
    )
    assert mass == pytest.approx(AIRCRAFT_EMPTY_MASS_KG)


def test_simulated_release_channel_matches_the_active_aircraft_profile():
    params = (ROOT / "config/ardupilot/plane.parm").read_text()
    assert "SERVO8_FUNCTION  0" in params
    # The checked-in flight profile uses the 0.10 gate required for panels
    # that are briefly clipped at the edge of the camera frame.  The Python
    # parameter model keeps 0.20 as its generic default for non-flight tests.
    assert "SCR_USER6        0.10" in params


def test_airframe_uses_pinned_official_ardupilot_visual_assets():
    model = (ROOT / "models/cuadc_twin/model.sdf").read_text()
    provenance = (ROOT / "OFFICIAL_ASSETS.md").read_text()
    assert "mini_talon_fuselage.dae" in model
    assert "mini_talon_left_wing.dae" in model
    assert "mini_talon_right_wing.dae" in model
    assert "mini_talon_left_aileron.dae" in model
    assert "mini_talon_right_aileron.dae" in model
    assert model.count("2.807017544") >= 10
    assert "25bc38ed8c6c0345840159a8cbc0b02781d52f3c" in provenance
    assert "ArduPilot/SITL_Models" in provenance


def test_down_camera_fov_matches_projection_intrinsics():
    import math

    from cuadc_sim.config import DOWN_CAMERA_HORIZONTAL_FOV_RAD

    model = (ROOT / "models/cuadc_twin/model.sdf").read_text()
    assert f"<horizontal_fov>{DOWN_CAMERA_HORIZONTAL_FOV_RAD:.7f}</horizontal_fov>" in model
    assert math.degrees(DOWN_CAMERA_HORIZONTAL_FOV_RAD) == 80.0


def test_world_uses_rule_arrow_wells_and_matching_physical_walls():
    import xml.etree.ElementTree as ET
    from cuadc_sim.config import SimParameters
    from cuadc_sim.scenario import generate_scenario, wells_to_sdf

    generated = wells_to_sdf(generate_scenario(SimParameters()))
    scene = ET.fromstring("<root>" + generated + "</root>")
    assert len(scene.findall("model")) == 8
    assert len(scene.findall(".//link[@name='target_board']")) == 6
    for model in scene.findall("model"):
        well = model.find("link[@name='well']")
        walls = [part for part in well.findall("collision") if part.get("name").startswith("well_wall_")]
        assert len(walls) == 5
        for collision in walls:
            visual = well.find("visual[@name='" + collision.get("name").removesuffix("_collision") + "']")
            assert collision.findtext("pose") == visual.findtext("pose")
            assert collision.findtext("geometry/box/size") == visual.findtext("geometry/box/size")
            assert float(collision.findtext("geometry/box/size").split()[2]) == 0.5
        assert well.findtext("collision[@name='well_floor_collision']/geometry/mesh/uri") == "well_floor.obj"
    assert "well_wall_ring" not in generated
    assert "<cylinder>" not in generated
    assert "well_arrow_shaft" not in generated
    assert generated.count("<size>0.600 0.600</size>") == 6
    world = (ROOT / "worlds/cuadc_world.sdf.in").read_text()
    assert "<max_step_size>0.002</max_step_size>" in world
    assert "Grass004_2K_Color.jpg" in world
    assert "<transparency>" not in world


def test_hand_launch_operator_model_is_removed():
    """手抛操作员模型已移除；发射物理仍由静锚点与插件承担。"""
    world = (ROOT / "worlds/cuadc_world.sdf.in").read_text()
    model = (ROOT / "models/cuadc_twin/model.sdf").read_text()

    assert "cuadc_human" not in world
    assert "hand_launch_operator" not in world
    assert "cuadc_human" not in model
    assert not (ROOT / "models/cuadc_human").exists()

    # 发射物理必须保留：静锚点、可分离关节、手抛插件与冲量参数。
    assert '<model name="hand_launcher">' in world
    assert "gz-sim-detachable-joint-system" in world
    assert '<plugin filename="CuadcHandLaunchSystem"' in model
    assert "<launch_mass>" in model
    assert "<impulse_duration>" in model

    # 不再向插件传人物动画参数，避免引用不存在的 actor。
    for tag in ("actor_name", "idle_animation", "launch_animation",
                "launch_animation_duration"):
        assert f"<{tag}>" not in model


def test_payload_release_is_a_single_latched_detachable_joint():
    model = (ROOT / "models/cuadc_twin/model.sdf").read_text()
    source = (ROOT / "plugins/CuadcPayloadReleaseSystem.cc").read_text()
    assert "gz-sim-detachable-joint-system" in model
    assert "this->releaseRequested.exchange(true)" in source
    assert "this->released.exchange(true)" in source
    assert "elapsed >= this->delaySeconds" in source
    assert "actuator_delay_s" in source
    assert "/cuadc/payload/impact" in model
    assert "impactRecorded.exchange(true)" in source
    assert "ContactSensorData" in source
    assert "velocity_sample" in source
    assert "release_position" in source
    assert "release_velocity_mps" in source
    assert "previousVelocity" in source
    assert "first payload impact atomically recorded" in source


def test_gazebo_is_the_only_external_physics_wind_source():
    world = (ROOT / "worlds/cuadc_world.sdf.in").read_text()
    patch = (ROOT / "patches/ardupilot_gazebo_airspeed.patch").read_text()
    assert "<linear_velocity>@@WIND_VECTOR@@</linear_velocity>" in world
    assert "<force_approximation_scaling_factor>@@GAZEBO_WIND_FORCE_SCALE@@</force_approximation_scaling_factor>" in world
    assert 'writer.Key("airspeed")' in patch
    assert "windSpdBdyA" in patch


def test_external_physics_rejects_nonfinite_samples_and_reports_origin():
    guard = (
        ROOT / "patches/ardupilot_gazebo_finite_state.patch"
    ).read_text()
    diagnostics = (
        ROOT / "plugins/CuadcAirframeDiagnosticsSystem.cc"
    ).read_text()
    assert "refusing non-finite / unbounded external physics state" in guard
    assert "finiteVector(statePosition)" in guard
    assert "CUADC physics health failure at link" in diagnostics
    assert "components::LinearVelocity" in diagnostics
    assert "components::AngularVelocity" in diagnostics


def test_supervisor_isolates_gazebo_discovery_and_refreshes_run_logs():
    source = (ROOT / "src/cuadc_sim/supervisor.py").read_text()
    assert 'self.env["GZ_PARTITION"] = "cuadc_garden_79"' in source
    assert 'self.env["GZ_IP"] = "127.0.0.1"' in source
    assert 'mode = "a" if name in self.started_log_names else "w"' in source
    assert '".deps/runtime/ardupilot-4.6.3"' in source
    assert '".deps/runtime/ardupilot-gazebo-garden"' in source
    assert '".deps/ardupilot/build' not in source
    assert "link.resolve(strict=False) == source.resolve()" in source
