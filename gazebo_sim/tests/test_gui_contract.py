from __future__ import annotations

import os
from pathlib import Path

from cuadc_sim.gui import JsonMtimeCache


def test_gui_contract_is_light_camera_only_and_keeps_four_slots():
    root = Path(__file__).resolve().parents[1]
    source = (root / "src/cuadc_sim/gui.py").read_text(encoding="utf-8")
    assert "QTableWidget(4, 5)" in source
    assert "background:#ffffff" in source
    assert "飞行状态" not in source
    assert "公开航线与视觉估算" not in source
    assert "route_state.json" not in source


def test_json_cache_only_reloads_after_file_mtime_changes(tmp_path: Path):
    path = tmp_path / "state.json"
    path.write_text('{"frame": 1}', encoding="utf-8")
    cache = JsonMtimeCache(path)
    value, changed = cache.load()
    assert changed and value == {"frame": 1}
    assert cache.load() == ({"frame": 1}, False)

    previous_mtime_ns = path.stat().st_mtime_ns
    path.write_text('{"frame": 2, "updated": true}', encoding="utf-8")
    os.utime(path, ns=(previous_mtime_ns + 1_000_000, previous_mtime_ns + 1_000_000))
    value, changed = cache.load()
    assert changed and value == {"frame": 2, "updated": True}
