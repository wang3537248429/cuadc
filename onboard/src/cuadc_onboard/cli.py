from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import platform
import re
import time

import cv2
import numpy as np

from .camera import open_camera
from .camera_device import camera_candidates, is_auto_camera_device
from .config import DEFAULT_CONFIG, camera_angles_rad, load_config
from .runtime import main as runtime_main
from .router import main as router_main
from .vision import VisionEngine
from .mission import load_qgc_waypoints


def _camera_matrix(config):
    if config.camera.fx_px <= 0.0:
        return None
    return np.array(
        [
            [config.camera.fx_px, 0.0, config.camera.cx_px],
            [0.0, config.camera.fy_px, config.camera.cy_px],
            [0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def _check(config_path: Path, allow_non_pi: bool, skip_camera: bool) -> int:
    config = load_config(config_path)
    failures: list[str] = []
    warnings: list[str] = []
    architecture = platform.machine().lower()
    if architecture not in {"aarch64", "arm64"}:
        message = f"expected Raspberry Pi 64-bit ARM, found {architecture}"
        (warnings if allow_non_pi else failures).append(message)
    serial_device = config.telemetry_router.serial_device
    serial_path = Path(serial_device)
    resolved_serial_device = None
    if not serial_path.exists():
        failures.append(f"flight controller device is missing: {serial_device}")
    else:
        resolved_serial_device = str(serial_path.resolve(strict=True))
    camera_device = config.camera.device
    discovered_camera_devices = camera_candidates(camera_device)
    resolved_camera_device = None
    actual_camera_mode = None
    if not discovered_camera_devices:
        failures.append("no USB UVC capture camera was discovered")
    elif is_auto_camera_device(camera_device):
        first_camera = Path(discovered_camera_devices[0])
        resolved_camera_device = str(first_camera.resolve(strict=True))
        if len(discovered_camera_devices) > 1:
            warnings.append(
                "multiple USB cameras were discovered; auto mode will use the "
                "first camera that returns frames"
            )
    else:
        camera_path = Path(camera_device)
        if camera_device.isdigit():
            resolved_camera_device = camera_device
        elif not camera_path.exists():
            failures.append(f"USB camera device is missing: {camera_device}")
        else:
            resolved_camera_device = str(camera_path.resolve(strict=True))
    weights = config.vision.weights_path()
    if not weights.exists():
        failures.append(f"Task {config.vision.task} weights are missing: {weights}")
    if not skip_camera:
        try:
            capture = open_camera(config)
            ok, frame = capture.read()
            capture.release()
            if not ok or frame is None:
                failures.append("USB camera opened but did not return a frame")
            resolved_camera_device = getattr(
                capture, "resolved_device", resolved_camera_device
            )
            actual_camera_mode = {
                "device": getattr(capture, "device", camera_device),
                "identity": getattr(capture, "identity", None),
                "width": getattr(capture, "width", None),
                "height": getattr(capture, "height", None),
                "fps": getattr(capture, "fps", None),
                "pixel_format": getattr(capture, "pixel_format", None),
            }
        except Exception as exc:
            failures.append(str(exc))
    if config.camera.horizontal_fov_deg == 90.0:
        warnings.append(
            "camera FOV is still the 90-degree placeholder; calibrate it before target positioning"
        )
    if not config.camera.calibration_confirmed:
        warnings.append("camera calibration has not been confirmed")
    if not config.vision.validation_confirmed:
        warnings.append("real-image vision validation has not been confirmed")
    if config.release.drag_rate_s == 0.0:
        warnings.append(
            "payload drag is uncalibrated; solver currently uses the no-drag baseline"
        )
    if not config.release.ballistics_calibrated:
        warnings.append("real payload ballistics have not been confirmed")
    if not config.release.enabled:
        failures.append(
            "active-only package requires release.enabled=true; "
            "no flight-controller mission will be modified"
        )
    expected_mission_path = config.mission.expected_path()
    expected_mission_count = None
    expected_mission_hash = None
    if expected_mission_path is None:
        if config.mission.acceptance_mode == "pinned":
            failures.append("pinned mode requires mission.expected_file")
        else:
            warnings.append(
                "dynamic mission acceptance is enabled; the live Mission Planner "
                "route is not locked to a file hash"
            )
    else:
        expected_mission_count = len(load_qgc_waypoints(expected_mission_path))
        expected_mission_hash = hashlib.sha256(
            expected_mission_path.read_bytes()
        ).hexdigest()
    print(json.dumps({
        "config": str(config_path.resolve()),
        "architecture": architecture,
        "task": config.vision.task,
        "weights": str(weights),
        "release_enabled": config.release.enabled,
        "mission_acceptance_mode": config.mission.acceptance_mode,
        "mission_stable_readbacks": config.mission.stable_readbacks,
        "serial_device": serial_device,
        "resolved_serial_device": resolved_serial_device,
        "camera_device": camera_device,
        "camera_auto": is_auto_camera_device(camera_device),
        "camera_candidates": discovered_camera_devices,
        "resolved_camera_device": resolved_camera_device,
        "actual_camera_mode": actual_camera_mode,
        "expected_mission": (
            str(expected_mission_path) if expected_mission_path else None
        ),
        "expected_mission_sha256": expected_mission_hash,
        "expected_mission_count": expected_mission_count,
        "warnings": warnings,
        "failures": failures,
    }, ensure_ascii=False, indent=2))
    return 1 if failures else 0


def _benchmark(config_path: Path, samples: int) -> int:
    config = load_config(config_path)
    capture = open_camera(config)
    roll, pitch, yaw = camera_angles_rad(config.camera)
    engine = VisionEngine(
        {"task": config.vision.task, "selected_group": "REAL"},
        confidence=config.vision.confidence,
        weights=config.vision.weights_path(),
        inference_hz=1000.0,
        inference_angles=config.vision.angles,
        inference_size=config.vision.inference_size(),
        device=config.vision.device,
        horizontal_fov_rad=math.radians(config.camera.horizontal_fov_deg),
        telemetry_alignment_s=config.camera.telemetry_alignment_s,
        camera_roll_rad=roll,
        camera_pitch_rad=pitch,
        camera_yaw_rad=yaw,
        camera_matrix=_camera_matrix(config),
        distortion_coefficients=config.camera.distortion,
    )
    if engine.model is None:
        capture.release()
        print(engine.state.message)
        return 2
    telemetry = {
        "relative_alt_m": config.release.nominal_agl_m,
        "north_m": 0.0,
        "east_m": 0.0,
        "velocity_north_mps": config.release.nominal_airspeed_mps,
        "velocity_east_mps": 0.0,
        "roll_rad": 0.0,
        "pitch_rad": 0.0,
        "yaw_rad": 0.0,
    }
    durations: list[float] = []
    try:
        for index in range(samples + 1):
            ok, frame = capture.read()
            if not ok or frame is None:
                raise RuntimeError("USB camera did not return a benchmark frame")
            engine.last_inference = 0.0
            started = time.perf_counter()
            engine.process(frame, telemetry)
            duration = time.perf_counter() - started
            if index > 0:  # discard model/backend warm-up
                durations.append(duration)
    finally:
        capture.release()
    mean = sum(durations) / len(durations)
    print(json.dumps({
        "task": config.vision.task,
        "model": str(config.vision.weights_path()),
        "input_size": config.vision.inference_size(),
        "angles_per_frame": len(config.vision.angles),
        "samples": len(durations),
        "mean_seconds_per_camera_frame": mean,
        "effective_fps": 1.0 / mean,
        "individual_seconds": durations,
    }, ensure_ascii=False, indent=2))
    return 0


def expected_value_from_path(path: Path, task: int, task1_values: dict[str, int]) -> int | None:
    normalized = path.stem.lower()
    if task == 2:
        matches = re.findall(r"(?:number[-_]|task2[-_])(\d{2})(?:\D|$)", normalized)
        return int(matches[-1]) if matches else None
    for label, value in sorted(task1_values.items(), key=lambda item: len(item[0]), reverse=True):
        if label and not label.isdigit() and label in normalized:
            return value
    return None


def _validate_images(config_path: Path, image_dir: Path) -> int:
    config = load_config(config_path)
    roll, pitch, yaw = camera_angles_rad(config.camera)
    engine = VisionEngine(
        {"task": config.vision.task, "selected_group": "REAL"},
        confidence=config.vision.confidence,
        weights=config.vision.weights_path(),
        inference_hz=1000.0,
        inference_angles=config.vision.angles,
        inference_size=config.vision.inference_size(),
        device=config.vision.device,
        horizontal_fov_rad=math.radians(config.camera.horizontal_fov_deg),
        telemetry_alignment_s=config.camera.telemetry_alignment_s,
        camera_roll_rad=roll,
        camera_pitch_rad=pitch,
        camera_yaw_rad=yaw,
        camera_matrix=_camera_matrix(config),
        distortion_coefficients=config.camera.distortion,
    )
    if engine.model is None:
        print(engine.state.message)
        return 2
    paths = sorted(
        path
        for path in image_dir.rglob("*")
        if path.suffix.lower() in {".jpg", ".jpeg", ".png", ".bmp"}
    )
    telemetry = {
        "relative_alt_m": config.release.nominal_agl_m,
        "north_m": 0.0,
        "east_m": 0.0,
        "velocity_north_mps": config.release.nominal_airspeed_mps,
        "velocity_east_mps": 0.0,
        "roll_rad": 0.0,
        "pitch_rad": 0.0,
        "yaw_rad": 0.0,
    }
    records: list[dict] = []
    for path in paths:
        expected = expected_value_from_path(
            path, config.vision.task, engine.task1_values
        )
        if expected is None:
            continue
        frame = cv2.imread(str(path))
        if frame is None:
            continue
        engine.tracks = []
        engine.visible_boxes = []
        engine.visible_detections = []
        for _ in range(2):
            engine.last_inference = 0.0
            engine.process(frame, telemetry)
        predictions = [
            int(item["value"]) for item in engine.state.detections
        ]
        records.append(
            {
                "image": str(path),
                "expected": expected,
                "predictions": predictions,
                "pass": expected in predictions,
            }
        )
    passed = sum(bool(record["pass"]) for record in records)
    report = {
        "task": config.vision.task,
        "images_with_filename_labels": len(records),
        "passes": passed,
        "accuracy": passed / len(records) if records else 0.0,
        "records": records,
    }
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if records and passed == len(records) else 1


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="cuadc-onboard",
        description="CUADC Raspberry Pi real-aircraft companion service",
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    subparsers = parser.add_subparsers(dest="command", required=True)
    check_parser = subparsers.add_parser("check", help="run non-mutating hardware preflight checks")
    check_parser.add_argument("--allow-non-pi", action="store_true")
    check_parser.add_argument("--skip-camera", action="store_true")
    benchmark_parser = subparsers.add_parser("benchmark", help="benchmark the configured model on the USB camera")
    benchmark_parser.add_argument("--samples", type=int, default=5)
    validation_parser = subparsers.add_parser(
        "validate-images",
        help="validate filename-labelled real images (number-XX or class name)",
    )
    validation_parser.add_argument("image_dir", type=Path)
    subparsers.add_parser("run", help="run the real-aircraft companion service")
    subparsers.add_parser("router", help="run the flight-controller USB/local MAVLink router")
    subparsers.add_parser("show-config", help="validate and display the effective configuration")
    args = parser.parse_args(argv)
    if args.command == "check":
        return _check(args.config, args.allow_non_pi, args.skip_camera)
    if args.command == "benchmark":
        if args.samples < 1:
            parser.error("--samples must be positive")
        return _benchmark(args.config, args.samples)
    if args.command == "validate-images":
        return _validate_images(args.config, args.image_dir)
    config = load_config(args.config)
    if args.command == "show-config":
        print(json.dumps(asdict(config), ensure_ascii=False, indent=2))
        return 0
    if args.command == "run":
        return runtime_main(config)
    if args.command == "router":
        return router_main(config)
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
