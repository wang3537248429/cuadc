import math

import pytest

from cuadc_sim.ballistics import (
    along_track_error,
    crossed_release_line,
    solution_with_actuator_delay,
    solve_release,
)
from cuadc_sim.config import PAYLOAD_DRAG_RATE_S


def test_level_release_solution_matches_gazebo_linear_drag_model():
    result = solve_release(200, 0, agl_m=30, velocity_north_mps=23, velocity_east_mps=0)
    k = PAYLOAD_DRAG_RATE_S
    response = (1.0 - math.exp(-k * result.fall_time_s)) / k
    down = (
        9.80665 / k * result.fall_time_s
        - 9.80665 / k * response
    )
    assert down == pytest.approx(30.0)
    assert result.north_lead_m == pytest.approx(23.0 * response)
    assert result.release_north_m < 200
    assert result.release_east_m == 0


def test_wind_changes_release_lead_without_double_applying_gravity():
    calm = solve_release(200, 0, 30, 23, 0)
    tailwind = solve_release(200, 0, 30, 23, 0, wind_north_mps=8)
    assert tailwind.north_lead_m > calm.north_lead_m
    assert tailwind.fall_time_s == calm.fall_time_s


def test_zero_drag_retains_free_fall_solution():
    result = solve_release(
        200,
        0,
        agl_m=30,
        velocity_north_mps=23,
        velocity_east_mps=0,
        drag_rate_s=0,
    )
    assert result.fall_time_s == pytest.approx(math.sqrt(2 * 30 / 9.80665))


def test_ptk_actuator_delay_moves_command_upstream():
    base = solve_release(200, 0, 20, 20, 0, drag_rate_s=0)
    delayed = solution_with_actuator_delay(base, 20, 0, 0.12)
    assert delayed.release_north_m == pytest.approx(base.release_north_m - 2.4)
    assert delayed.north_lead_m == pytest.approx(base.north_lead_m + 2.4)


def test_along_track_sign_changes_after_crossing_release_point():
    before = along_track_error(100, 0, 120, 0, 20, 0)
    after = along_track_error(121, 0, 120, 0, 20, 0)
    assert before > 0
    assert after < 0


def test_release_crossing_requires_a_real_baseline_sample():
    assert not crossed_release_line(math.inf, -10)
    assert not crossed_release_line(-12, -10)
    assert not crossed_release_line(10, 4)
    assert crossed_release_line(4, -0.1)


def test_invalid_altitude_is_rejected():
    with pytest.raises(ValueError):
        solve_release(0, 0, 0, 10, 0)
