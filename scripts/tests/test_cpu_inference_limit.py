"""Capture must retain CPU time after YOLO's lazy predictor initialization."""
from importlib import import_module
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest


@pytest.mark.parametrize('project,package', [('gazebo_sim', 'cuadc_sim'), ('onboard', 'cuadc_onboard')])
def test_predictor_warmup_cannot_restore_oversized_cpu_pool(monkeypatch, tmp_path, project, package):
    root = Path(__file__).resolve().parents[2]
    monkeypatch.syspath_prepend(str(root / project / 'src'))
    vision = import_module(f'{package}.vision')
    pool = {'threads': 1, 'initialized': False}

    class Model:
        def predict(self, **_kwargs):
            if not pool['initialized']:
                pool.update(threads=8, initialized=True)
            return [SimpleNamespace(names={}, boxes=[])]

    monkeypatch.setitem(sys.modules, 'torch', SimpleNamespace(
        set_num_threads=lambda count: pool.update(threads=count)))
    monkeypatch.setitem(sys.modules, 'ultralytics', SimpleNamespace(YOLO=lambda _path: Model()))
    weights = tmp_path / 'local.pt'
    weights.write_bytes(b'local test model')
    engine = vision.VisionEngine({'task': 1, 'selected_group': 'A'}, weights=weights)
    assert engine.state.model_loaded
    assert pool['initialized'] and pool['threads'] == 2
    engine.process(np.zeros((320, 320, 3), dtype=np.uint8))
    assert pool['threads'] == 2
    assert engine.state.detections == []
