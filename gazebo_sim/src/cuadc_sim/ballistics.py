from __future__ import annotations

import math
from dataclasses import dataclass, replace

from .config import PAYLOAD_DRAG_RATE_S


@dataclass(frozen=True)
class ReleaseSolution:
    fall_time_s: float
    north_lead_m: float
    east_lead_m: float
    release_north_m: float
    release_east_m: float


def solve_release(
    target_north_m: float,
    target_east_m: float,
    agl_m: float,
    velocity_north_mps: float,
    velocity_east_mps: float,
    wind_north_mps: float = 0.0,
    wind_east_mps: float = 0.0,
    vertical_down_mps: float = 0.0,
    gravity_mps2: float = 9.80665,
    drag_rate_s: float = PAYLOAD_DRAG_RATE_S,
) -> ReleaseSolution:
    if agl_m <= 0:
        raise ValueError("AGL must be positive")
    if gravity_mps2 <= 0:
        raise ValueError("gravity must be positive")
    if drag_rate_s < 0:
        raise ValueError("drag rate must be non-negative")

    if drag_rate_s < 1e-9:
        # agl = v_down*t + 1/2*g*t^2, positive down.
        discriminant = vertical_down_mps**2 + 2.0 * gravity_mps2 * agl_m
        fall_time = (
            -vertical_down_mps + math.sqrt(discriminant)
        ) / gravity_mps2
        north_lead = velocity_north_mps * fall_time
        east_lead = velocity_east_mps * fall_time
    else:
        # Gazebo Garden WindEffects applies
        #   acceleration = k * (wind_velocity - link_velocity)
        # to every payload / ribbon link with enable_wind=true. Integrate that
        # exact first-order model so the companion release solver and Gazebo
        # physics use the same drag law.
        k = drag_rate_s
        terminal_down = gravity_mps2 / k

        def down_distance(seconds: float) -> float:
            response = -math.expm1(-k * seconds) / k
            return (
                terminal_down * seconds
                + (vertical_down_mps - terminal_down) * response
            )

        low, high = 0.0, max(1.0, math.sqrt(2.0 * agl_m / gravity_mps2))
        while down_distance(high) < agl_m:
            high *= 2.0
        for _ in range(64):
            middle = (low + high) / 2.0
            if down_distance(middle) < agl_m:
                low = middle
            else:
                high = middle
        fall_time = (low + high) / 2.0
        response = -math.expm1(-k * fall_time) / k
        north_lead = (
            wind_north_mps * fall_time
            + (velocity_north_mps - wind_north_mps) * response
        )
        east_lead = (
            wind_east_mps * fall_time
            + (velocity_east_mps - wind_east_mps) * response
        )
    return ReleaseSolution(
        fall_time_s=fall_time,
        north_lead_m=north_lead,
        east_lead_m=east_lead,
        release_north_m=target_north_m - north_lead,
        release_east_m=target_east_m - east_lead,
    )


def solution_with_actuator_delay(
    solution: ReleaseSolution,
    velocity_north_mps: float,
    velocity_east_mps: float,
    delay_s: float,
) -> ReleaseSolution:
    """Move the command point upstream by the measured command-to-release lag."""
    if delay_s < 0.0:
        raise ValueError("actuator delay cannot be negative")
    extra_north = velocity_north_mps * delay_s
    extra_east = velocity_east_mps * delay_s
    return replace(
        solution,
        north_lead_m=solution.north_lead_m + extra_north,
        east_lead_m=solution.east_lead_m + extra_east,
        release_north_m=solution.release_north_m - extra_north,
        release_east_m=solution.release_east_m - extra_east,
    )


def along_track_error(
    aircraft_north_m: float,
    aircraft_east_m: float,
    release_north_m: float,
    release_east_m: float,
    velocity_north_mps: float,
    velocity_east_mps: float,
) -> float:
    speed = math.hypot(velocity_north_mps, velocity_east_mps)
    if speed < 1e-6:
        return math.inf
    north_unit = velocity_north_mps / speed
    east_unit = velocity_east_mps / speed
    return (
        (release_north_m - aircraft_north_m) * north_unit
        + (release_east_m - aircraft_east_m) * east_unit
    )


def crossed_release_line(previous_error_m: float, current_error_m: float) -> bool:
    """Detect a real positive-to-negative crossing after a baseline sample exists."""
    return (
        math.isfinite(previous_error_m)
        and previous_error_m > 0.0
        and current_error_m <= 0.0
    )
