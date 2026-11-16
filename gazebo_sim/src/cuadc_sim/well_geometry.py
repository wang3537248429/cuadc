"""2025 规则图 2/4 的天井轮廓，所有尺寸以米为单位。"""
from __future__ import annotations

import math


BODY_WIDTH_M = 1.0
BODY_LENGTH_M = 1.0
ARROW_SIDE_M = 1.0
WALL_HEIGHT_M = 0.5
# 规则未规定壁厚和底板厚度；以下为碰撞建模假设。
WALL_THICKNESS_M = 0.035
FLOOR_THICKNESS_M = 0.03
TARGET_PANEL_SIZE_M = 0.6


def footprint() -> tuple[tuple[float, float], ...]:
    half_width = BODY_WIDTH_M / 2
    half_length = BODY_LENGTH_M / 2
    head_height = math.sqrt(ARROW_SIDE_M**2 - half_width**2)
    return (
        (-half_width, -half_length), (half_width, -half_length),
        (half_width, half_length), (0.0, half_length + head_height),
        (-half_width, half_length),
    )


def wall_segments() -> list[tuple[float, float, float, float]]:
    """Return inward-offset wall centers, edge lengths and yaw angles."""
    points = footprint()
    segments = []
    for first, second in zip(points, points[1:] + points[:1]):
        dx, dy = second[0] - first[0], second[1] - first[1]
        length = math.hypot(dx, dy)
        inward = WALL_THICKNESS_M / (2 * length)
        segments.append((
            (first[0] + second[0]) / 2 - dy * inward,
            (first[1] + second[1]) / 2 + dx * inward,
            length, math.degrees(math.atan2(dy, dx)),
        ))
    return segments


def panel_wall_clearance(relative_heading_rad: float) -> float:
    """最小平面净距；靶板任意旋转都必须完全位于井壁内侧。"""
    half = TARGET_PANEL_SIZE_M/2
    c, s = math.cos(relative_heading_rad), math.sin(relative_heading_rad)
    corners = [(c*x-s*y, s*x+c*y) for x, y in ((-half,-half),(-half,half),(half,half),(half,-half))]
    points = footprint()
    return min(((second[0]-first[0])*(y-first[1])-(second[1]-first[1])*(x-first[0]))
               / math.dist(first, second)-WALL_THICKNESS_M
               for first, second in zip(points, points[1:]+points[:1]) for x,y in corners)


def visible_panel_fraction(camera_position, panel_heading_rad, arrow_heading_rad) -> float:
    """离线采集质检：对靶板网格发射视线，检查真实井壁遮挡。"""
    import numpy as np
    coordinates = np.linspace(-TARGET_PANEL_SIZE_M/2, TARGET_PANEL_SIZE_M/2, 21)
    xy = np.array(np.meshgrid(coordinates, coordinates)).reshape(2, -1).T
    c, s = math.cos(panel_heading_rad), math.sin(panel_heading_rad)
    xy = xy @ np.array([[c,s],[-s,c]])
    points = np.column_stack((xy, np.full(len(xy), FLOOR_THICKNESS_M+.004)))
    camera = np.asarray(camera_position, dtype=float).copy()
    c, s = math.cos(arrow_heading_rad), math.sin(arrow_heading_rad)
    to_well = np.array([[c,-s],[s,c]])
    points[:, :2] = points[:, :2] @ to_well
    camera[:2] = camera[:2] @ to_well
    visible = np.ones(len(points), dtype=bool)
    for x, y, length, yaw in wall_segments():
        c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
        rotation = np.array([[c,-s],[s,c]])
        origin = camera.copy()
        origin[:2] = (origin[:2]-[x,y]) @ rotation
        target = points.copy()
        target[:, :2] = (target[:, :2]-[x,y]) @ rotation
        directions = target-origin
        near = np.zeros(len(points))
        far = np.ones(len(points))
        for axis, (low, high) in enumerate(((-length/2,length/2),(-WALL_THICKNESS_M/2,WALL_THICKNESS_M/2),(0,WALL_HEIGHT_M))):
            delta = directions[:, axis]
            parallel = np.abs(delta)<1e-10
            safe_delta = np.where(parallel, 1, delta)
            a, b = (low-origin[axis])/safe_delta, (high-origin[axis])/safe_delta
            left, right = np.minimum(a,b), np.maximum(a,b)
            inside = low <= origin[axis] <= high
            left[parallel] = -np.inf if inside else np.inf
            right[parallel] = np.inf if inside else -np.inf
            near, far = np.maximum(near,left), np.minimum(far,right)
        visible &= ~((near<=far)&(near<.999999)&(far>0))
    return float(np.mean(visible))


def floor_mesh_obj() -> str:
    """A watertight solid floor; render and collision share the same mesh."""
    points = footprint()
    count = len(points)
    lines = ["# Generated CUADC arrow-shaped well floor (metres).", "o well_floor"]
    for height in (0.0, FLOOR_THICKNESS_M):
        lines.extend(f"v {x:.9f} {y:.9f} {height:.9f}" for x, y in points)
    faces = []
    for index in range(1, count - 1):
        faces.append((1, index + 2, index + 1))
        faces.append((count + 1, count + index + 1, count + index + 2))
    for index in range(count):
        first, second = index + 1, (index + 1) % count + 1
        faces.append((first, second, second + count))
        faces.append((first, second + count, first + count))
    vertices = [(x, y, z) for z in (0.0, FLOOR_THICKNESS_M) for x, y in points]
    for normal_index, face in enumerate(faces, 1):
        a, b, c = (vertices[index - 1] for index in face)
        u, v = tuple(bi-ai for ai, bi in zip(a, b)), tuple(ci-ai for ai, ci in zip(a, c))
        normal = (u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0])
        magnitude = math.sqrt(sum(value**2 for value in normal))
        lines.append("vn " + " ".join(f"{value/magnitude:.9f}" for value in normal))
        lines.append("f " + " ".join(f"{index}//{normal_index}" for index in face))
    return "\n".join(lines) + "\n"
