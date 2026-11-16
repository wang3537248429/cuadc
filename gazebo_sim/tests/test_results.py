import json

from cuadc_sim.results import ResultStore, RunResult


def test_result_is_atomic_and_latest_is_retained(tmp_path):
    result = RunResult.create(9, 2, "B")
    result.status = "COMPLETED"
    result.selected_target = {"track_id": "T03", "value": 50, "guessed": False}
    store = ResultStore(result, tmp_path)
    path = store.save()
    assert json.loads(path.read_text())["status"] == "COMPLETED"
    latest = json.loads((tmp_path / "latest.json").read_text())
    assert latest["schema"] == "cuadc-run-result.v3"
    assert latest["selected_target"]["track_id"] == "T03"
