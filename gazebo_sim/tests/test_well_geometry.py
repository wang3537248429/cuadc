import math

import pytest

from cuadc_sim.well_geometry import footprint, floor_mesh_obj, panel_wall_clearance, visible_panel_fraction


def test_rotating_panel_never_intersects_any_wall():
    assert min(panel_wall_clearance(math.radians(angle)) for angle in range(360)) > .04


def test_offline_visibility_rejects_heavily_occluded_views():
    assert visible_panel_fraction((0,0,14),0,0) == 1
    assert visible_panel_fraction((18,0,14),0,0) < .6
    assert visible_panel_fraction((0,0,14),1.7,2.1) == 1


def test_rule_footprint_has_one_metre_base_sides_and_arrow_edges():
    points = footprint()
    assert len(points) == 5
    assert max(x for x, y in points) - min(x for x, y in points) == pytest.approx(1.0)
    for first, second in zip(points, points[1:] + points[:1]):
        assert math.dist(first, second) == pytest.approx(1.0)
    assert max(y for x, y in points) - min(y for x, y in points) == pytest.approx(1 + math.sqrt(3) / 2)


def test_floor_collision_mesh_is_watertight_and_has_outward_winding():
    from collections import Counter

    vertices, faces = [], []
    for row in floor_mesh_obj().splitlines():
        fields = row.split()
        if fields[0] == "v":
            vertices.append(tuple(map(float, fields[1:])))
        if fields[0] == "f":
            faces.append(tuple(int(value.split("//")[0]) - 1 for value in fields[1:]))
    edges = Counter()
    volume = 0.0
    for face in faces:
        a, b, c = (vertices[index] for index in face)
        volume += (a[0] * (b[1]*c[2]-b[2]*c[1]) - a[1] * (b[0]*c[2]-b[2]*c[0]) + a[2] * (b[0]*c[1]-b[1]*c[0])) / 6
        edges.update(tuple(sorted(pair)) for pair in zip(face, face[1:] + face[:1]))
    assert set(edges.values()) == {2}
    assert volume == pytest.approx((1 + math.sqrt(3)/4) * 0.03)
