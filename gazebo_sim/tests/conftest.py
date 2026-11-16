from __future__ import annotations

from pathlib import Path
import os
import sys
import tempfile


# Imports bind several default paths. Set these before test modules import
# cuadc_sim, so unit tests cannot read or rewrite a developer's live flight.
_artifacts = tempfile.TemporaryDirectory(prefix="cuadc-tests-")
os.environ["CUADC_RUNTIME_DIR"] = str(Path(_artifacts.name) / "runtime")
os.environ["CUADC_RUNS_DIR"] = str(Path(_artifacts.name) / "runs")
os.environ['CUADC_ARENA_CONFIG']=str(Path(__file__).resolve().parents[1]/'config/arena.json')


SOURCE_ROOT = Path(__file__).resolve().parents[1] / "src"
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))
