import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from cuadc_sim.config import SimParameters
from cuadc_sim.scenario import generate_scenario
from cuadc_sim.vision import (
    VisionEngine,
    correct_task2_rotation,
    pair_task2_digit_boxes,
    parse_task2_component,
    resolve_task_weights,
    save_vision_state,
    unrotate_bbox,
)


def test_local_reference_panels_are_complete_when_installed():
    root = Path(__file__).resolve().parents[1]
    if not (root / "assets/targets/reference").exists():
        pytest.skip("源码包不分发靶标图案；此项需要本地素材包")
    catalog = json.loads((root / "assets/targets/catalog.json").read_text())
    for target in catalog["task1"]:
        assert (root / f"assets/targets/reference/task1/{target['key']}.png").is_file()
    for value in range(100):
        assert (root / f"assets/targets/reference/task2/number-{value:02d}.png").is_file()


def test_target_catalog_describes_the_current_task2_digit_pair_model():
    root = Path(__file__).resolve().parents[1]
    catalog = json.loads(
        (root / "assets/targets/catalog.json").read_text(encoding="utf-8")
    )
    task2 = catalog["models"]["task2"]
    assert task2["weights"] == "task2_sim_digits_v1.pt"
    assert task2["classes"] == [
        *(f"tens_{digit}" for digit in range(10)),
        *(f"ones_{digit}" for digit in range(10)),
    ]


def test_repository_model_weights_are_resolved_from_standard_directory(tmp_path: Path):
    source = tmp_path / "models" / "vision"
    source.mkdir(parents=True)
    task1 = source / "task1_v4m_best.pt"
    task2 = source / "task2_sim_digits_v1.pt"
    task1.write_bytes(b"task1")
    task2.write_bytes(b"task2")
    assert resolve_task_weights(1, tmp_path) == task1.resolve()
    assert resolve_task_weights(2, tmp_path) == task2.resolve()


def test_missing_weight_reports_the_standard_repository_path(tmp_path: Path):
    expected = tmp_path / "models/vision/task1_v4m_best.pt"
    assert resolve_task_weights(1, tmp_path) == expected


def test_explicit_missing_weight_path_is_authoritative(monkeypatch, tmp_path: Path):
    missing = tmp_path / "deliberately-missing.pt"
    monkeypatch.setenv("CUADC_TASK1_WEIGHTS", str(missing))
    assert resolve_task_weights(1, tmp_path) == missing.resolve()


def test_task2_rotation_ambiguities_and_box_projection():
    assert correct_task2_rotation(16, 180) == 91
    assert correct_task2_rotation(41, 180) == 41
    assert correct_task2_rotation(16, 90) == 16
    box = (20.0, 40.0, 60.0, 80.0)
    assert unrotate_bbox(box, 90, 1280, 720) == (40.0, 660.0, 80.0, 700.0)
    assert unrotate_bbox(box, 180, 1280, 720) == (1220.0, 640.0, 1260.0, 680.0)
    assert unrotate_bbox(box, 270, 1280, 720) == (1200.0, 20.0, 1240.0, 60.0)


def test_missing_weights_warns_without_generating_truth_detections(tmp_path: Path):
    scenario = generate_scenario(SimParameters(task=1, group="A", seed=4)).to_dict()
    engine = VisionEngine(scenario, weights=tmp_path / "missing.pt")
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)
    annotated = engine.process(frame)
    assert annotated.shape == frame.shape
    assert engine.state.source == "MODEL_MISSING"
    assert "MODEL MISSING" in engine.state.banner
    assert engine.state.detections == []
    assert not hasattr(engine.state, "wells")


def test_task2_class_parser_preserves_00():
    scenario = generate_scenario(SimParameters(task=2, group="B", seed=1)).to_dict()
    engine = VisionEngine(scenario, weights=Path("/definitely/missing.pt"))
    assert engine._class_to_value("number-00") == 0
    assert engine._class_to_value("class_99") == 99


def test_task2_digit_yolo_boxes_pair_without_scenario_truth():
    assert parse_task2_component("tens_3") == ("tens", 3)
    assert parse_task2_component("ones_1") == ("ones", 1)
    assert parse_task2_component("31") is None
    paired = pair_task2_digit_boxes(
        [
            {"bbox": (100, 100, 130, 160), "role": "tens", "digit": 3, "confidence": 0.91},
            {"bbox": (140, 100, 170, 160), "role": "ones", "digit": 1, "confidence": 0.88},
            {"bbox": (500, 200, 530, 260), "role": "tens", "digit": 8, "confidence": 0.95},
            {"bbox": (540, 200, 570, 260), "role": "ones", "digit": 4, "confidence": 0.90},
        ]
    )
    assert [(item["value"], item["bbox"]) for item in paired] == [
        (31, (100, 100, 170, 160)),
        (84, (500, 200, 570, 260)),
    ]
    assert paired[0]["confidence"] == 0.88


def test_loaded_model_with_no_candidates_never_uses_truth(tmp_path: Path):
    weights = tmp_path / "present.pt"
    weights.write_bytes(b"test placeholder")
    scenario = generate_scenario(SimParameters(task=2, group="A", seed=9)).to_dict()
    engine = VisionEngine(scenario, weights=weights)

    class EmptyModel:
        def predict(self, **_kwargs):
            return [SimpleNamespace(names={0: "number-00"}, boxes=[])]

    engine.model = EmptyModel()
    engine.state.model_loaded = True
    engine.state.source = "YOLO"
    engine.state.banner = "YOLO LIVE"
    engine.last_inference = 0.0
    engine.process(
        np.zeros((720, 1280, 3), dtype=np.uint8),
        {"relative_alt_m": 30, "north_m": 200, "east_m": 0, "yaw_rad": 0},
    )
    assert engine.state.source == "YOLO"
    assert engine.state.detections == []
    assert not hasattr(engine.state, "wells")


def test_only_repeated_yolo_boxes_are_published_and_boxes_remain_visible(tmp_path: Path):
    engine = VisionEngine(
        {"task": 1, "selected_group": "A"},
        weights=tmp_path / "missing.pt",
    )
    box = SimpleNamespace(
        xyxy=np.array([[100.0, 100.0, 220.0, 220.0]]),
        conf=np.array([0.9]),
        cls=np.array([0]),
    )

    class BoxModel:
        def predict(self, **_kwargs):
            return [SimpleNamespace(names={0: "multirotor"}, boxes=[box])]

    engine.model = BoxModel()
    engine.inference_angles = (0,)
    engine.state.model_loaded = True
    engine.state.source = "YOLO"
    engine.state.banner = "YOLO LIVE"
    telemetry = {
        "relative_alt_m": 20,
        "north_m": 180,
        "east_m": 0,
        "roll_rad": 0,
        "pitch_rad": 0,
        "yaw_rad": 0,
    }
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)
    first = engine.process(frame, telemetry)
    assert first[100, 100].any()
    assert engine.state.detections == []
    assert len(engine.state.visible_detections) == 1
    between_inferences = engine.process(frame, telemetry)
    assert between_inferences[100, 100].any()
    assert engine.state.detections == []
    assert len(engine.state.visible_detections) == 1
    engine.last_inference = 0.0
    engine.process(frame, telemetry)
    assert len(engine.state.detections) == 1
    assert engine.state.detections[0]["frames"] == 2


def test_projection_uses_roll_pitch_and_yaw(tmp_path: Path):
    scenario = generate_scenario(SimParameters(task=1, group="A", seed=4)).to_dict()
    engine = VisionEngine(scenario, weights=tmp_path / "missing.pt")
    base = {
        "relative_alt_m": 30,
        "north_m": 100,
        "east_m": 20,
        "roll_rad": 0,
        "pitch_rad": 0,
        "yaw_rad": 0,
    }
    assert engine._project_pixel(640, 360, base, 1280, 720) == (100.0, 20.0)
    pitched = dict(base, pitch_rad=np.deg2rad(10))
    north, east = engine._project_pixel(640, 360, pitched, 1280, 720)
    assert north > 105.0
    assert east == 20.0
    turned = dict(pitched, yaw_rad=np.deg2rad(90))
    north, east = engine._project_pixel(640, 360, turned, 1280, 720)
    assert abs(north - 100.0) < 1e-6
    assert east > 25.0


def test_projection_aligns_camera_and_telemetry_by_measured_velocity(tmp_path: Path):
    engine = VisionEngine(
        {"task": 1, "selected_group": "A"},
        weights=tmp_path / "missing.pt",
        telemetry_alignment_s=0.2,
    )
    north, east = engine._project_pixel(
        640,
        360,
        {
            "relative_alt_m": 20,
            "north_m": 100,
            "east_m": 5,
            "velocity_north_mps": 14,
            "velocity_east_mps": -2,
        },
        1280,
        720,
    )
    assert abs(north - 102.80) < 0.01
    assert abs(east - 4.60) < 0.01


def test_projection_accepts_opencv_camera_intrinsics(tmp_path: Path):
    engine = VisionEngine(
        {"task": 1, "selected_group": "A"},
        weights=tmp_path / "missing.pt",
        camera_matrix=np.array(
            [[640.0, 0.0, 640.0], [0.0, 640.0, 360.0], [0.0, 0.0, 1.0]]
        ),
        distortion_coefficients=(0.0, 0.0, 0.0, 0.0, 0.0),
        telemetry_alignment_s=0.0,
    )
    telemetry = {
        "relative_alt_m": 20,
        "north_m": 100,
        "east_m": 5,
        "roll_rad": 0,
        "pitch_rad": 0,
        "yaw_rad": 0,
    }
    assert engine._project_pixel(640, 360, telemetry, 1280, 720) == (100.0, 5.0)
    north, east = engine._project_pixel(704, 360, telemetry, 1280, 720)
    assert north == pytest.approx(100.0)
    assert east == pytest.approx(7.0)


def test_scenario_truth_changes_do_not_change_missing_model_output(tmp_path: Path):
    first = generate_scenario(SimParameters(task=1, group="A", seed=1)).to_dict()
    second = generate_scenario(SimParameters(task=1, group="A", seed=99)).to_dict()
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)
    left = VisionEngine(first, weights=tmp_path / "missing.pt")
    right = VisionEngine(second, weights=tmp_path / "missing.pt")
    left.process(frame)
    right.process(frame)
    assert left.state.detections == right.state.detections == []
    assert left.state.source == right.state.source == "MODEL_MISSING"


def test_preview_frames_can_be_throttled_without_staling_vision_state(tmp_path: Path):
    engine = VisionEngine({"task": 1, "selected_group": "A"}, weights=tmp_path / "missing.pt")
    frame = np.zeros((24, 32, 3), dtype=np.uint8)
    engine.process(frame)

    save_vision_state(engine.state, frame, tmp_path, save_frame=False)

    assert (tmp_path / "vision_state.json").is_file()
    assert not (tmp_path / "vision_frame.jpg").exists()

    save_vision_state(engine.state, frame, tmp_path, save_frame=True)
    assert (tmp_path / "vision_frame.jpg").is_file()
