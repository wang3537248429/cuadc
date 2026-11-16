from types import SimpleNamespace

from cuadc_sim.telemetry import Telemetry


def test_control_and_servo_diagnostics_are_retained():
    telemetry = Telemetry()
    telemetry.update_attitude(
        SimpleNamespace(
            roll=0.1,
            pitch=0.2,
            yaw=0.3,
            rollspeed=0.4,
            pitchspeed=0.5,
            yawspeed=0.6,
        )
    )
    telemetry.update_nav_controller(SimpleNamespace(nav_roll=12, nav_pitch=-4))
    telemetry.update_hud(SimpleNamespace(airspeed=16, groundspeed=15, throttle=42))
    telemetry.update_servos(
        SimpleNamespace(**{f"servo{index}_raw": 1000 + index for index in range(1, 9)})
    )
    assert telemetry.pitch_rate_rps == 0.5
    assert telemetry.target_pitch_deg == -4
    assert telemetry.airspeed_mps == 16
    assert telemetry.throttle_percent == 42
    assert telemetry.servo_pwm[:2] == [1001, 1002]
    assert telemetry.servo_pwm[8:] == [0] * 8
