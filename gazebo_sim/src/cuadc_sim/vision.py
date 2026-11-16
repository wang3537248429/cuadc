from __future__ import annotations

import json
import hashlib
import math
import os
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from .pixel_rois import panel_pixels_support, pixel_views
from .frame_timing import align_to_frame

from .config import (
    CAMERA_TELEMETRY_ALIGNMENT_S,
    DOWN_CAMERA_HORIZONTAL_FOV_RAD,
    ROOT,
    RUNTIME_DIR,
)
from .selection import Detection, aggregate_detections


WEIGHT_FILENAMES = {
    1: "task1_v4m_best.pt",
    2: "task2_sim_digits_v1.pt",
}
INFERENCE_ANGLES = (0,)
MIN_STABLE_FRAMES = 2
TASK2_ROT180_PAIRS = {
    1: 10,
    10: 1,
    6: 90,
    90: 6,
    8: 80,
    80: 8,
    9: 60,
    60: 9,
    16: 91,
    91: 16,
    18: 81,
    81: 18,
    19: 61,
    61: 19,
    66: 99,
    99: 66,
    68: 89,
    89: 68,
    86: 98,
    98: 86,
}


def resolve_task_weights(task: int, root: Path = ROOT) -> Path:
    """Resolve user-supplied weights without copying or modifying them."""
    if task not in WEIGHT_FILENAMES:
        raise ValueError(f"unsupported vision task: {task}")
    override = os.environ.get(f"CUADC_TASK{task}_WEIGHTS")
    filename = WEIGHT_FILENAMES[task]
    # An explicit path is authoritative even when it is absent. This makes a
    # configuration typo visible as MODEL MISSING instead of silently loading
    # a different copy, and provides a deterministic fallback acceptance mode.
    if override:
        return Path(override).expanduser().resolve()
    candidate = root / "models" / "vision" / filename
    if candidate.is_file():
        return candidate.resolve()
    # Keep the conventional path in diagnostics when bootstrap has not yet
    # unpacked the repository-provided model archives.
    return candidate


def correct_task2_rotation(value: int, angle_deg: int) -> int:
    """Apply the v3 model's documented 180-degree ambiguity correction."""
    if angle_deg == 180:
        return TASK2_ROT180_PAIRS.get(value, value)
    return value


def unrotate_bbox(
    bbox: tuple[float, float, float, float],
    angle_deg: int,
    original_width: int,
    original_height: int,
) -> tuple[float, float, float, float]:
    """Map a box from a right-angle-rotated image back to the camera frame."""
    x1, y1, x2, y2 = bbox
    if angle_deg == 0:
        return bbox
    if angle_deg == 90:
        return y1, original_height - x2, y2, original_height - x1
    if angle_deg == 180:
        return (
            original_width - x2,
            original_height - y2,
            original_width - x1,
            original_height - y1,
        )
    if angle_deg == 270:
        return original_width - y2, x1, original_width - y1, x2
    raise ValueError(f"unsupported right-angle rotation: {angle_deg}")


def rotate_frame(frame: np.ndarray, angle_deg: int) -> np.ndarray:
    if angle_deg == 0:
        return frame
    if angle_deg == 90:
        return cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    if angle_deg == 180:
        return cv2.rotate(frame, cv2.ROTATE_180)
    if angle_deg == 270:
        return cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    raise ValueError(f"unsupported right-angle rotation: {angle_deg}")


def parse_task2_component(class_name: str) -> tuple[str, int] | None:
    """Parse the simulator digit model's semantic board classes."""

    normalized = class_name.strip().lower()
    for role in ("tens", "ones"):
        prefix = f"{role}_"
        if normalized.startswith(prefix):
            suffix = normalized[len(prefix) :]
            if suffix.isdigit() and 0 <= int(suffix) <= 9:
                return role, int(suffix)
    return None


def pair_task2_digit_boxes(
    components: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    """Pair neighbouring YOLO tens/ones boxes into spatial target readings."""

    tens = [item for item in components if item.get("role") == "tens"]
    ones = [item for item in components if item.get("role") == "ones"]
    proposals: list[tuple[float, int, int]] = []
    for tens_index, left in enumerate(tens):
        lx1, ly1, lx2, ly2 = left["bbox"]
        left_center = ((lx1 + lx2) / 2.0, (ly1 + ly2) / 2.0)
        for ones_index, right in enumerate(ones):
            rx1, ry1, rx2, ry2 = right["bbox"]
            right_center = ((rx1 + rx2) / 2.0, (ry1 + ry2) / 2.0)
            distance = math.hypot(
                left_center[0] - right_center[0],
                left_center[1] - right_center[1],
            )
            scale = max(
                lx2 - lx1,
                ly2 - ly1,
                rx2 - rx1,
                ry2 - ry1,
                1.0,
            )
            if distance <= 3.5 * scale:
                proposals.append((distance / scale, tens_index, ones_index))

    used_tens: set[int] = set()
    used_ones: set[int] = set()
    paired: list[dict[str, Any]] = []
    for _score, tens_index, ones_index in sorted(proposals):
        if tens_index in used_tens or ones_index in used_ones:
            continue
        left, right = tens[tens_index], ones[ones_index]
        used_tens.add(tens_index)
        used_ones.add(ones_index)
        lx1, ly1, lx2, ly2 = left["bbox"]
        rx1, ry1, rx2, ry2 = right["bbox"]
        value = int(left["digit"]) * 10 + int(right["digit"])
        paired.append(
            {
                "bbox": (
                    min(lx1, rx1),
                    min(ly1, ry1),
                    max(lx2, rx2),
                    max(ly2, ry2),
                ),
                "value": value,
                # A valid reading requires both independent digit boxes.
                "confidence": min(
                    float(left["confidence"]), float(right["confidence"])
                ),
            }
        )
    return paired


@dataclass
class VisionState:
    schema: str = "cuadc-vision-state.v3"
    source: str = "PENDING"
    banner: str = "WAITING FOR GAZEBO CAMERA"
    model_path: str = ""
    model_loaded: bool = False
    model_sha256: str = ""
    frame_sha256: str = ""
    input_source: str = "camera_pixels"
    task: int = 1
    group: str = "A"
    frame_count: int = 0
    inference_count: int = 0
    camera_position_alignment_s: float = 0.0
    camera_time_valid: bool = True
    updated_at: float = 0.0
    detections: list[dict[str, Any]] = field(default_factory=list)
    visible_detections: list[dict[str, Any]] = field(default_factory=list)
    message: str = ""


class VisionEngine:
    def __init__(
        self,
        public_context: dict,
        confidence: float = 0.20,
        weights: Path | None = None,
        inference_hz: float = 3.0,
        inference_angles: tuple[int, ...] | None = None,
        inference_size: int | None = None,
        device: str = "cpu",
        horizontal_fov_rad: float = DOWN_CAMERA_HORIZONTAL_FOV_RAD,
        telemetry_alignment_s: float = CAMERA_TELEMETRY_ALIGNMENT_S,
        agl_offset_m: float = 0.0,
        camera_roll_rad: float = 0.0,
        camera_pitch_rad: float = 0.0,
        camera_yaw_rad: float = 0.0,
        camera_matrix: np.ndarray | None = None,
        distortion_coefficients: tuple[float, ...] | None = None,
    ):
        self.task = int(public_context["task"])
        self.group_name = str(public_context["selected_group"])
        self.confidence = confidence
        self.inference_period = 1.0 / max(0.1, inference_hz)
        # Pixel ROIs keep small targets readable. Full-frame fallback retains the
        # configured resolution; throughput must be verified on each device.
        self.inference_size = (
            inference_size
            if inference_size is not None
            else (960 if self.task == 2 else 640)
        )
        self.device = device
        self.horizontal_fov_rad = horizontal_fov_rad
        self.telemetry_alignment_s = telemetry_alignment_s
        self.agl_offset_m = agl_offset_m
        self.camera_roll_rad = camera_roll_rad
        self.camera_pitch_rad = camera_pitch_rad
        self.camera_yaw_rad = camera_yaw_rad
        self.camera_matrix = camera_matrix
        self.distortion_coefficients = (
            np.asarray(distortion_coefficients, dtype=float)
            if distortion_coefficients is not None
            else None
        )
        self.last_inference = 0.0
        # Match the active onboard profile: one unrotated inference per frame.
        self.inference_angles = (
            tuple(inference_angles)
            if inference_angles is not None
            else INFERENCE_ANGLES
        )
        self.tracks: list[Detection] = []
        self.visible_boxes: list[dict[str, Any]] = []
        self.visible_detections: list[Detection] = []
        self.model = None
        self.weights = weights or resolve_task_weights(self.task)
        self.state = VisionState(
            task=self.task,
            group=self.group_name,
            model_path=str(self.weights),
        )
        self.task1_values = self._task1_values()
        self._load_model()

    def _task1_values(self) -> dict[str, int]:
        manifest = json.loads((ROOT / "assets/targets/catalog.json").read_text(encoding="utf-8"))
        result: dict[str, int] = {}
        for item in manifest["task1"]:
            result[str(item["key"]).lower()] = int(item["value"])
            result[str(item["label"]).lower()] = int(item["value"])
            result[str(item["value"])] = int(item["value"])
        return result

    def _load_model(self) -> None:
        if not self.weights.exists():
            self.state.source = "MODEL_MISSING"
            self.state.banner = "MODEL MISSING / RECOGNITION DISABLED"
            self.state.message = f"weight file not found: {self.weights}"
            return
        try:
            from ultralytics import YOLO

            if self.device == "cpu":
                import torch
                torch.set_num_threads(2)
            self.model = YOLO(str(self.weights))
            if self.device == "cpu":
                # Ultralytics initializes its predictor lazily and resets the
                # Torch pool in select_device. Warm it up in this worker, then
                # reapply the cap so capture keeps CPU time during live use.
                self.model.predict(
                    source=np.zeros((320, 320, 3), dtype=np.uint8),
                    imgsz=320,
                    device="cpu",
                    verbose=False,
                )
                torch.set_num_threads(2)
            self.state.model_sha256 = hashlib.sha256(self.weights.read_bytes()).hexdigest()
            self.state.model_loaded = True
            self.state.source = "YOLO"
            self.state.banner = "YOLO LIVE"
            self.state.message = (
                f"YOLO inference on {self.device}: {self.inference_size}px, "
                f"angles={self.inference_angles}"
            )
        except Exception as exc:
            self.state.source = "MODEL_LOAD_FAILED"
            self.state.banner = "MODEL LOAD FAILED / RECOGNITION DISABLED"
            self.state.message = f"{type(exc).__name__}: {exc}"

    def _class_to_value(self, class_name: str) -> int | None:
        normalized = class_name.strip().lower().replace("number-", "")
        if self.task == 1:
            return self.task1_values.get(normalized)
        digits = "".join(character for character in normalized if character.isdigit())
        if not digits:
            return None
        value = int(digits[-2:])
        return value if 0 <= value <= 99 else None

    def _project_pixel(
        self,
        u: float,
        v: float,
        telemetry: dict,
        width: int,
        height: int,
    ) -> tuple[float, float] | None:
        """Project a down-camera pixel onto flat ground using full attitude.

        The ray is expressed in aircraft NED body axes (forward, right, down)
        and rotated with the MAVLink roll/pitch/yaw attitude.  Returning None
        for an above-horizon ray prevents extreme-bank projections from
        creating false ground tracks.
        """

        if not telemetry.get("camera_time_valid",True):
            return None
        agl = max(0.1, float(telemetry.get("relative_alt_m", 0.0)) + self.agl_offset_m)
        roll = float(telemetry.get("roll_rad", 0.0))
        pitch = float(telemetry.get("pitch_rad", 0.0))
        yaw = float(telemetry.get("yaw_rad", 0.0))
        if self.camera_matrix is not None:
            normalized = cv2.undistortPoints(
                np.array([[[u, v]]], dtype=np.float64),
                self.camera_matrix,
                self.distortion_coefficients,
            )[0, 0]
            horizontal_normalized = float(normalized[0])
            vertical_normalized = float(normalized[1])
        else:
            fx = width / (2.0 * math.tan(self.horizontal_fov_rad / 2.0))
            fy = fx
            horizontal_normalized = (u - width / 2.0) / fx
            vertical_normalized = (v - height / 2.0) / fy
        body_ray = np.array(
            [
                -vertical_normalized,
                horizontal_normalized,
                1.0,
            ],
            dtype=float,
        )
        # Apply the measured camera-to-body mounting offset before aircraft
        # attitude. Zero offsets preserve the simulator's exact body-down
        # camera convention.
        mcr, msr = math.cos(self.camera_roll_rad), math.sin(self.camera_roll_rad)
        mcp, msp = math.cos(self.camera_pitch_rad), math.sin(self.camera_pitch_rad)
        mcy, msy = math.cos(self.camera_yaw_rad), math.sin(self.camera_yaw_rad)
        camera_to_body = np.array(
            [
                [mcp * mcy, msr * msp * mcy - mcr * msy, mcr * msp * mcy + msr * msy],
                [mcp * msy, msr * msp * msy + mcr * mcy, mcr * msp * msy - msr * mcy],
                [-msp, msr * mcp, mcr * mcp],
            ],
            dtype=float,
        )
        body_ray = camera_to_body @ body_ray
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        body_to_ned = np.array(
            [
                [cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy],
                [cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy],
                [-sp, sr * cp, cr * cp],
            ],
            dtype=float,
        )
        ned_ray = body_to_ned @ body_ray
        if ned_ray[2] <= 1e-4:
            return None
        scale = agl / float(ned_ray[2])
        north = (
            float(telemetry.get("north_m", 0.0))
            + scale * float(ned_ray[0])
            + float(telemetry.get("velocity_north_mps", 0.0))
            * self.telemetry_alignment_s
        )
        east = (
            float(telemetry.get("east_m", 0.0))
            + scale * float(ned_ray[1])
            + float(telemetry.get("velocity_east_mps", 0.0))
            * self.telemetry_alignment_s
        )
        return north, east

    def process(self, frame_bgr: np.ndarray, telemetry: dict | None = None) -> np.ndarray:
        self.state.frame_count += 1
        telemetry = align_to_frame(telemetry or {})
        self.state.camera_position_alignment_s=float(telemetry.get("camera_position_alignment_s",0))
        self.state.camera_time_valid=bool(telemetry.get("camera_time_valid",True))
        annotated = frame_bgr.copy()
        now = time.monotonic()
        if self.model is None:
            cv2.putText(
                annotated,
                self.state.banner,
                (24, 48),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 0, 255),
                3,
                cv2.LINE_AA,
            )
        elif now - self.last_inference >= self.inference_period:
            self.last_inference = now
            self.state.inference_count += 1
            self.state.frame_sha256 = hashlib.sha256(frame_bgr.tobytes()).hexdigest()
            height, width = frame_bgr.shape[:2]
            box_candidates: list[dict[str, Any]] = []
            raw_detections: list[Detection] = []
            for view in pixel_views(frame_bgr, self.group_name, canonical=self.task==2):
                for angle in self.inference_angles:
                    inference_frame = rotate_frame(view.image, angle)
                    results = self.model.predict(
                        source=inference_frame,
                        # The task models were trained and calibrated against
                        # the 320 px ROI.  Keep that input size for task 1 as
                        # well as task 2; increasing only the ROI input changes
                        # the detector's small-panel calibration and reduces
                        # recall on the actual simulator camera.
                        imgsz=320 if view.roi else self.inference_size,
                        conf=self.confidence,
                        device=self.device,
                        verbose=False,
                    )
                    for result in results:
                        names = result.names
                        if result.boxes is None:
                            continue
                        predictions: list[dict[str, Any]] = []
                        task2_components: list[dict[str, Any]] = []
                        for box in result.boxes:
                            rotated_box = tuple(
                                float(component) for component in box.xyxy[0].tolist()
                            )
                            confidence = float(box.conf[0])
                            class_id = int(box.cls[0])
                            class_name = str(names[class_id])
                            component = (
                                parse_task2_component(class_name)
                                if self.task == 2
                                else None
                            )
                            if component is not None:
                                role, digit = component
                                task2_components.append(
                                    {
                                        "bbox": rotated_box,
                                        "role": role,
                                        "digit": digit,
                                        "confidence": confidence,
                                    }
                                )
                                continue
                            value = self._class_to_value(class_name)
                            if value is not None and self.task == 2:
                                value = correct_task2_rotation(value, angle)
                            if value is not None:
                                predictions.append(
                                    {
                                        "bbox": rotated_box,
                                        "value": value,
                                        "confidence": confidence,
                                        "label": class_name,
                                    }
                                )
                        # The simulator-specific Task 2 model detects the two
                        # physical number boards separately. Pairing happens only
                        # within this same real YOLO result; no scenario position,
                        # label, or expected value participates.
                        predictions.extend(pair_task2_digit_boxes(task2_components))
                        for prediction in predictions:
                            rotated_box = prediction["bbox"]
                            x1, y1, x2, y2 = unrotate_bbox(
                                rotated_box, angle, view.image.shape[1], view.image.shape[0]
                            )
                            x1, y1, x2, y2 = view.to_original((x1, y1, x2, y2))
                            if view.roi and not panel_pixels_support(frame_bgr, (x1,y1,x2,y2)):
                                continue
                            confidence = float(prediction["confidence"])
                            value = int(prediction["value"])
                            projection = self._project_pixel(
                                (x1 + x2) / 2.0,
                                (y1 + y2) / 2.0,
                                telemetry,
                                width,
                                height,
                            )
                            if projection is None:
                                continue
                            north, east = projection
                            label = (
                                f"{value:02d}@{angle}deg"
                                if self.task == 2
                                else str(prediction.get("label", value))
                            )
                            detection = Detection(
                                track_id="",
                                value=value,
                                confidence=confidence,
                                label=label,
                                north_m=north,
                                east_m=east,
                            )
                            raw_detections.append(detection)
                            box_candidates.append(
                                {
                                    "bbox": (x1, y1, x2, y2),
                                    "value": value,
                                    "label": label,
                                    "confidence": confidence,
                                    "north_m": north,
                                    "east_m": east,
                                }
                            )
            # Fuse all rotated views before adding history: "frames" means
            # distinct camera inference frames, never the number of rotations.
            self.visible_detections = aggregate_detections(raw_detections,prefer_frame_votes=False)
            for fused in self.visible_detections:
                self.tracks.append(
                    Detection(
                        track_id="",
                        value=fused.value,
                        confidence=fused.confidence,
                        frames=1,
                        label=fused.label,
                        north_m=fused.north_m,
                        east_m=fused.east_m,
                    )
                )
            if len(self.tracks) > 4000:
                self.tracks = self.tracks[-3000:]
            # Draw one real YOLO box per fused spatial target rather than four
            # overlapping boxes from the rotated views.
            visible_boxes: list[dict[str, Any]] = []
            for fused in self.visible_detections:
                matching = [
                    candidate
                    for candidate in box_candidates
                    if int(candidate["value"]) == fused.value
                    and math.hypot(
                        float(candidate["north_m"]) - fused.north_m,
                        float(candidate["east_m"]) - fused.east_m,
                    )
                    <= 8.0
                ]
                if not matching:
                    continue
                best = max(matching, key=lambda item: float(item["confidence"]))
                visible_boxes.append(
                    {
                        **best,
                        "label": fused.label,
                        "confidence": fused.confidence,
                    }
                )
            self.visible_boxes = visible_boxes
        # The camera publishes faster than the inference scheduler. Keep
        # the most recent genuine YOLO boxes visible between inference frames;
        # otherwise the next undecorated camera frame erases them almost
        # immediately and makes real detections look like hidden truth data.
        boxes_are_recent = (
            now - self.last_inference
            <= max(0.75, self.inference_period * 3.0)
        )
        if boxes_are_recent:
            for item in self.visible_boxes:
                x1, y1, x2, y2 = item["bbox"]
                label = str(item["label"])
                confidence = float(item["confidence"])
                north = float(item["north_m"])
                east = float(item["east_m"])
                cv2.rectangle(
                    annotated,
                    (int(x1), int(y1)),
                    (int(x2), int(y2)),
                    (40, 200, 70),
                    3,
                )
                cv2.putText(
                    annotated,
                    f"{label} {confidence:.2f}  N{north:.1f} E{east:.1f}",
                    (int(x1), max(18, int(y1) - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (40, 170, 60),
                    2,
                    cv2.LINE_AA,
                )
        else:
            self.visible_boxes = []
            self.visible_detections = []
        # The aircraft moves while a target remains fixed on the ground.  A
        # strict spatial cluster can split one real panel into several
        # one-frame tracks when attitude/GPS projection noise changes between
        # passes.  For Task 1 the YOLO class/value is the detector's identity;
        # fuse that class's history independently so projection drift cannot
        # erase a valid multi-frame observation.  This does not add a class or
        # coordinate from the scenario.  Task 2 keeps spatial grouping because
        # two digit boards can share component classes.
        if self.task == 1:
            by_value: dict[int, list[Detection]] = {}
            for item in self.tracks:
                by_value.setdefault(item.value, []).append(item)
            aggregated = []
            for history in by_value.values():
                aggregated.extend(
                    aggregate_detections(history, cluster_radius_m=40.0)
                )
            aggregated.sort(key=lambda item: item.track_id)
        else:
            aggregated = aggregate_detections(self.tracks, cluster_radius_m=15.0)
        self.state.detections = [
            asdict(item)
            for item in aggregated
            if item.frames >= MIN_STABLE_FRAMES
        ]
        self.state.visible_detections = [
            asdict(item) for item in self.visible_detections
        ]
        self.state.updated_at = time.time()
        return annotated


def load_json(path: Path, default: dict | None = None) -> dict:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError):
        return default or {}


def save_vision_state(
    state: VisionState,
    frame: np.ndarray,
    runtime_dir: Path = RUNTIME_DIR,
    save_frame: bool = True,
) -> None:
    runtime_dir.mkdir(parents=True, exist_ok=True)
    state_path = runtime_dir / "vision_state.json"
    state_tmp = state_path.with_suffix(".json.tmp")
    state_tmp.write_text(
        json.dumps(asdict(state), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    os.replace(state_tmp, state_path)
    if not save_frame:
        return
    success, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 88])
    if success:
        frame_path = runtime_dir / "vision_frame.jpg"
        frame_tmp = runtime_dir / "vision_frame.jpg.tmp"
        frame_tmp.write_bytes(encoded.tobytes())
        os.replace(frame_tmp, frame_path)
        if state.visible_detections:
            detection_path = runtime_dir / "last_detection_frame.jpg"
            detection_tmp = runtime_dir / "last_detection_frame.jpg.tmp"
            detection_tmp.write_bytes(encoded.tobytes())
            os.replace(detection_tmp, detection_path)
