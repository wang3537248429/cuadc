"""Align telemetry to image exposure using sensor timestamps, never scene truth."""
from __future__ import annotations

import math


def align_to_frame(telemetry: dict, max_age_s: float = .75) -> dict:
    result=dict(telemetry)
    frame_sim=float(telemetry.get('camera_frame_time_s',0))
    sample_sim=float(telemetry.get('sample_time_s',0))
    frame_wall=float(telemetry.get('camera_frame_wall_time_s',0))
    sample_wall=float(telemetry.get('timestamp_s',0))
    if frame_sim>0 and sample_sim>0:
        delta=frame_sim-sample_sim
        attitude_time=float(telemetry.get('attitude_sample_time_s',0))
        attitude_delta=frame_sim-attitude_time if attitude_time>0 else 0.
    elif frame_wall>0 and sample_wall>0:
        # Arrival timestamps approximate USB capture/telemetry alignment;
        # transport and device latency still need physical calibration.
        delta=frame_wall-sample_wall
        attitude_time=float(telemetry.get('attitude_timestamp_s',0))
        attitude_delta=frame_wall-attitude_time if attitude_time>0 else 0.
    else:delta=attitude_delta=0.
    valid=all(math.isfinite(x) and abs(x)<=max_age_s for x in (delta,attitude_delta))
    result['camera_time_valid']=valid
    result['camera_position_alignment_s']=delta
    if not valid:return result
    for axis in ('north','east'):
        result[f'{axis}_m']=float(telemetry.get(f'{axis}_m',0))+float(telemetry.get(f'velocity_{axis}_mps',0))*delta
    result['relative_alt_m']=float(telemetry.get('relative_alt_m',0))-float(telemetry.get('velocity_down_mps',0))*delta
    for angle,rate in [('roll','roll_rate'),('pitch','pitch_rate'),('yaw','yaw_rate')]:
        result[f'{angle}_rad']=float(telemetry.get(f'{angle}_rad',0))+float(telemetry.get(f'{rate}_rps',0))*attitude_delta
    return result
