from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from cuadc_ground_station.tlog_reader import (  # noqa: E402
    default_tlog_root,
    mission_planner_log_candidates,
    newest_tlog,
)


def test_newest_tlog_is_selected_recursively(tmp_path: Path):
    first = tmp_path / "one.tlog"
    second = tmp_path / "nested/two.tlog"
    first.write_bytes(b"")
    second.parent.mkdir()
    second.write_bytes(b"")
    first.touch()
    second.touch()
    first_time = first.stat().st_mtime
    second_time = first_time + 2
    import os

    os.utime(second, (second_time, second_time))
    assert newest_tlog(tmp_path) == second


def test_windows_mission_planner_paths_include_onedrive_and_documents():
    candidates = mission_planner_log_candidates(
        home=Path("C:/Users/pilot"),
        environ={
            "USERPROFILE": "C:/Users/pilot",
            "OneDrive": "C:/Users/pilot/OneDrive",
            "LOCALAPPDATA": "C:/Users/pilot/AppData/Local",
        },
        platform_name="win32",
    )
    assert candidates[0] == Path(
        "C:/Users/pilot/OneDrive/Documents/Mission Planner/logs"
    )
    assert Path("C:/Users/pilot/Documents/Mission Planner/logs") in candidates
    assert Path("C:/Users/pilot/AppData/Local/Mission Planner/logs") in candidates


def test_default_root_chooses_the_directory_with_the_freshest_tlog(tmp_path: Path):
    old_root = tmp_path / "old/logs"
    fresh_root = tmp_path / "fresh/logs"
    old_root.mkdir(parents=True)
    fresh_root.mkdir(parents=True)
    old = old_root / "old.tlog"
    fresh = fresh_root / "fresh.tlog"
    old.write_bytes(b"")
    fresh.write_bytes(b"")
    import os

    os.utime(old, (10, 10))
    os.utime(fresh, (20, 20))
    assert default_tlog_root((old_root, fresh_root)) == fresh_root
