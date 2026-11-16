from __future__ import annotations

import gzip
import io
from pathlib import Path
import pickle
import tarfile
import zipfile

import pytest

from scripts.privacy_audit import audit, scan_text, scan_waypoints
from scripts.release_policy import public_files


@pytest.mark.parametrize("value", [
    "/" + "home/private-user/project",
    "gh" + "p_" + "a" * 36,
    "password=" + repr("unpublished-password"),
    "/dev/serial/by-id/usb-CubePilot_Controller_" + "A" * 24 + "-if00",
    "wx" + "id_" + "sensitive-user",
    "operator" + "@private.example.com",
])
def test_findings_do_not_echo_sensitive_values(value):
    findings = scan_text(value, "source.py")
    assert findings
    assert all(value not in finding for finding in findings)


def test_public_tree_refuses_secret_files(tmp_path):
    (tmp_path / ".env").write_text("hidden")
    _, findings = audit(tmp_path)
    assert any("凭据文件" in finding for finding in findings)


def test_generated_files_and_raw_training_data_are_never_exported(tmp_path):
    for name in (".private/secret.txt", "runs/flight.json", "eeprom.bin",
                 "onboard/config/onboard.local.yaml", "tools/vision_training/dataset/private.txt"):
        path = tmp_path / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("private data")
    (tmp_path / "README.md").write_text("公开说明")
    assert public_files(tmp_path) == [Path("README.md")]


def test_export_refuses_symlink_to_a_private_file(tmp_path):
    (tmp_path / "leak.py").symlink_to(tmp_path.parent / "private.txt")
    with pytest.raises(ValueError, match="非常规文件"):
        public_files(tmp_path)


def test_workspace_root_allowlist_does_not_export_unreviewed_material(tmp_path):
    (tmp_path / "README.md").write_text("公开说明")
    (tmp_path / "notes.txt").write_text("private notes")
    assert public_files(tmp_path, workspace=True) == [Path("README.md")]


def test_real_waypoint_coordinates_are_rejected():
    real = "QGC WPL 110\n0\t1\t0\t16\t0\t0\t0\t0\t31.2\t121.4\t0\t1\n"
    assert scan_waypoints(real, "example.waypoints")
    assert not scan_waypoints(real.replace("31.2", "0.1").replace("121.4", "0.1"), "example.waypoints")


def test_checkpoint_metadata_is_scanned_without_unpickling(tmp_path):
    data = io.BytesIO()
    with zipfile.ZipFile(data, "w") as archive:
        archive.writestr("archive/data.pkl", pickle.dumps({"path": "/" + "home/private-user/training"}))
    (tmp_path / "weights.pt.gz").write_bytes(gzip.compress(data.getvalue()))
    _, findings = audit(tmp_path)
    assert any("metadata@" in finding and "个人目录" in finding for finding in findings)
    assert all("private-user" not in finding for finding in findings)


def test_jpeg_comments_are_scanned_for_private_metadata(tmp_path):
    comment = b"source=/" + b"home/private-user/flight\n"
    jpeg = b"\xff\xd8\xff\xfe" + (len(comment) + 2).to_bytes(2, "big") + comment + b"\xff\xd9"
    (tmp_path / "frame.jpg").write_bytes(jpeg)
    _, findings = audit(tmp_path)
    assert any("jpeg-comment" in finding and "个人目录" in finding for finding in findings)
    assert all("private-user" not in finding for finding in findings)


def test_public_archive_has_no_history_or_local_owner_metadata(tmp_path):
    from scripts.build_release_packages import build

    archive = build("public", tmp_path)
    with tarfile.open(archive) as package:
        members = package.getmembers()
    assert all(member.isfile() for member in members)
    assert all(member.uid == member.gid == 0 and member.uname == member.gname == "" for member in members)
    assert all(member.mtime == 0 for member in members)
    names = [member.name for member in members]
    assert any("/ground_station/pyproject.toml" in name for name in names)
    assert not any(".private/" in name or "/.git/" in name or "eeprom.bin" in name for name in names)
