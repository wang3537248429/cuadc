"""实时局部预览：位置来自 YOLO 像素投影，图像始终取当前相机帧。"""
from __future__ import annotations

import math
import cv2
import numpy as np

from .frame_timing import align_to_frame


def target_details(frame, telemetry, tracks, horizontal_fov_rad, agl_offset_m=0):
    state = align_to_frame(telemetry)
    if not state.get('camera_time_valid', True):
        return None
    roll, pitch, yaw = (float(state.get(f'{axis}_rad', 0)) for axis in ('roll', 'pitch', 'yaw'))
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                             math.sin(pitch), math.cos(yaw), math.sin(yaw))
    rotation = np.array([[cp*cy, sr*sp*cy-cr*sy, cr*sp*cy+sr*sy],
                         [cp*sy, sr*sp*sy+cr*cy, cr*sp*sy-sr*cy], [-sp, sr*cp, cr*cp]])
    height, width = frame.shape[:2]
    focal = width/(2*math.tan(horizontal_fov_rad/2))
    tiles = []
    for track in tracks:
        ray = rotation.T @ np.array([track['north_m']-state.get('north_m', 0),
                                      track['east_m']-state.get('east_m', 0),
                                      state.get('relative_alt_m', 0)+agl_offset_m])
        if ray[2] <= 1:
            continue
        u, v = width/2+focal*ray[1]/ray[2], height/2-focal*ray[0]/ray[2]
        if not 0 <= u < width or not 0 <= v < height:
            continue
        radius = max(40, round(focal*1.1/ray[2]))
        x1, y1 = max(0, round(u)-radius), max(0, round(v)-radius)
        x2, y2 = min(width, round(u)+radius), min(height, round(v)+radius)
        crop = frame[y1:y2, x1:x2]
        if not crop.size:
            continue
        tile = cv2.resize(crop, (240, 240), interpolation=cv2.INTER_LINEAR)
        cv2.putText(tile, f'RAW {x2-x1}x{y2-y1}', (6, 20), cv2.FONT_HERSHEY_SIMPLEX,
                    .45, (10, 180, 240), 1, cv2.LINE_AA)
        tiles.append(tile)
        if len(tiles) == 3:
            break
    return np.hstack(tiles) if tiles else None
