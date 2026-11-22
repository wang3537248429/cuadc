from __future__ import annotations

import gzip
import hashlib
import math
from pathlib import Path
import subprocess
import sys

import pytest

from scripts.anonymize_mission import SITL_ALT, SITL_LAT, SITL_LON, anonymize
from scripts.check_local_resources import check_manifest
from scripts.privacy_audit import audit
from scripts.release_policy import public_files


def test_anonymized_route_preserves_geometry_commands_and_relative_altitude():
    # Arbitrary test origin, unrelated to the local flight site.
    latitude, longitude, altitude = 35.0, -100.0, 1234.5
    original = ("QGC WPL 110\n"
                "0 1 0 16 0 0 0 0 35.0 -100.0 1234.5 1\n"
                "1 0 3 22 20 0 0 0 35.001 -99.998 16 1\n"
                "2 0 3 183 8 1800 0 0 0 0 0 1\n")
    result = anonymize(original, latitude, longitude, altitude)
    before = [line.split() for line in original.splitlines()[1:]]
    after = [line.split() for line in result.splitlines()[1:]]
    assert [row[:8] + row[11:] for row in before] == [row[:8] + row[11:] for row in after]
    assert float(after[0][8]) == SITL_LAT
    assert float(after[0][9]) == SITL_LON
    assert float(after[0][10]) == SITL_ALT
    assert after[1][10] == before[1][10]
    assert after[2] == before[2]  # Servo command is not a geographic waypoint.
    old_north = float(before[1][8]) - latitude
    old_east = (float(before[1][9]) - longitude) * math.cos(math.radians(latitude))
    new_north = float(after[1][8]) - SITL_LAT
    new_east = (float(after[1][9]) - SITL_LON) * math.cos(math.radians(SITL_LAT))
    assert abs(new_north - old_north) < 1e-9
    assert abs(new_east - old_east) < 1e-9


def test_mission_anonymizer_refuses_overwrite(tmp_path):
    source, destination = tmp_path / "source.waypoints", tmp_path / "public.waypoints"
    source.write_text("QGC WPL 110\n")
    destination.write_text("keep existing mission")
    result = subprocess.run(
        [sys.executable, str(Path(__file__).resolve().parents[1] / "anonymize_mission.py"),
         str(source), str(destination), "--origin-lat", "35", "--origin-lon", "-100", "--origin-alt", "10"],
        capture_output=True, text=True,
    )
    assert result.returncode != 0
    assert destination.read_text() == "keep existing mission"


@pytest.mark.parametrize("name", ["saved.json", "arena_official_a.json"])
def test_nested_private_coordinates_have_no_filename_exemption(tmp_path, name):
    body = '{"field_layout": {"origin_lat_deg": "35.0", "origin_lon_deg": -100.0}}'
    (tmp_path / name).write_text(body)
    _, findings = audit(tmp_path)
    assert len(findings) == 2
    assert all("35.0" not in finding and "-100" not in finding for finding in findings)


def test_public_sitl_coordinates_remain_publishable(tmp_path):
    (tmp_path / "arena.json").write_text(
        '{"field_layout": {"origin_lat_deg": -35.363261, "origin_lon_deg": 149.165230}}')
    assert audit(tmp_path)[1] == []


def test_resource_policy_rejects_unlicensed_files_even_without_image_suffix(tmp_path):
    for name in ("assets/targets/reference/source.txt", "assets/targets/old_art.zip",
                 "datasets/samples/task1/data.yaml", "models/vision/task1_training.json",
                 "models/vision/training/results.csv", "models/vision/weight.copy"):
        path = tmp_path / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("restricted local content")
    allowed = tmp_path / "assets/targets/catalog.json"
    allowed.write_text("{}")
    assert public_files(tmp_path) == [Path("assets/targets/catalog.json")]


def test_resources_are_optional_only_when_the_entire_group_is_absent(tmp_path):
    manifest = tmp_path / "SHA256SUMS"
    digest = hashlib.sha256(b"known bytes").hexdigest()
    manifest.write_text(f"{digest}  one.dat\n{digest}  two.dat\n")
    assert not check_manifest(tmp_path, manifest)
    assert len(check_manifest(tmp_path, manifest, required=True)) == 2
    (tmp_path / "one.dat").write_bytes(b"known bytes")
    assert len(check_manifest(tmp_path, manifest)) == 1
    (tmp_path / "two.dat").write_bytes(b"known bytes")
    assert not check_manifest(tmp_path, manifest, required=True)
    (tmp_path / "two.dat").write_bytes(b"corrupt")
    assert len(check_manifest(tmp_path, manifest)) == 1


def test_model_manifest_accepts_a_verified_compressed_archive(tmp_path):
    payload = b"model bytes"
    (tmp_path / "task1_v4m_best.pt.gz").write_bytes(gzip.compress(payload))
    manifest = tmp_path / "MODEL_SHA256SUMS"
    manifest.write_text(
        f"{hashlib.sha256(payload).hexdigest()}  task1_v4m_best.pt\n"
    )
    assert not check_manifest(tmp_path, manifest, required=True)


@pytest.mark.parametrize("body", ["", "malformed", "0 " + "../private.dat", "0" * 64 + "  /tmp/private.dat"])
def test_bad_resource_manifests_fail_closed(tmp_path, body):
    manifest = tmp_path / "SHA256SUMS"
    manifest.write_text(body)
    assert check_manifest(tmp_path, manifest, required=True)


def test_resource_manifest_rejects_symlink_even_when_target_is_local(tmp_path):
    target = tmp_path / "target.dat"
    target.write_bytes(b"known bytes")
    (tmp_path / "linked.dat").symlink_to(target)
    manifest = tmp_path / "SHA256SUMS"
    manifest.write_text(f"{hashlib.sha256(target.read_bytes()).hexdigest()}  linked.dat\n")
    assert check_manifest(tmp_path, manifest, required=True)
