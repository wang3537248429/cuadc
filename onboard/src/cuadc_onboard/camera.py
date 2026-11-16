from __future__ import annotations

from dataclasses import asdict
from collections import deque
import json
import os
from pathlib import Path
import threading
import time
from typing import Callable

import cv2
import numpy as np

from .config import OnboardConfig, camera_angles_rad
from .camera_device import (
    camera_candidates,
    camera_identity,
    is_auto_camera_device,
)
from .vision import VisionEngine


TelemetryProvider = Callable[[], dict]

# UVC cameras with automatic exposure can return several nearly black frames
# immediately after opening.  Warm up long enough for exposure/focus control
# to settle before handing the first frame to the vision worker.
CAMERA_WARMUP_FRAMES = 30


class OpenedCamera:
    def __init__(
        self,
        capture,
        *,
        device: str,
        resolved_device: str,
        width: int,
        height: int,
        fps: float,
        pixel_format: str,
        pending_frame,
    ):
        self.capture = capture
        self.device = device
        self.resolved_device = resolved_device
        self.identity = camera_identity(device)
        self.width = width
        self.height = height
        self.fps = fps
        self.pixel_format = pixel_format
        self._pending_frame = pending_frame

    def read(self):
        if self._pending_frame is not None:
            frame = self._pending_frame
            self._pending_frame = None
            return True, frame
        return self.capture.read()

    def release(self) -> None:
        self._pending_frame = None
        self.capture.release()

    def isOpened(self) -> bool:  # noqa: N802 - mirrors OpenCV's API
        return bool(self.capture.isOpened())

    def __getattr__(self, name):
        return getattr(self.capture, name)


class LatestFrameCamera:
    """Keep UVC capture running while inference consumes only the latest frame."""

    def __init__(self, source):
        self.source=source
        self.condition=threading.Condition()
        self.stopped=threading.Event()
        self.sequence=0
        self.delivered=0
        self.processing_skipped_frames=0
        self.captured_at=0.
        self.latest=None
        self.ended=False
        self.arrivals=deque(maxlen=120)
        self.reader=threading.Thread(target=self._capture,name='cuadc-uvc-capture',daemon=True)
        self.reader.start()

    def _capture(self):
        failures=0
        try:
            while not self.stopped.is_set():
                ok,frame=self.source.read()
                if not ok or frame is None:
                    failures+=1
                    if failures>=30:break
                    self.stopped.wait(.02)
                    continue
                failures=0
                with self.condition:
                    self.sequence+=1
                    self.latest=(frame,time.time())
                    self.arrivals.append(time.monotonic())
                    self.condition.notify_all()
        finally:
            with self.condition:
                self.ended=True
                self.condition.notify_all()

    def read(self):
        with self.condition:
            self.condition.wait_for(lambda:self.sequence>self.delivered or self.ended or self.stopped.is_set(),timeout=2.)
            if self.sequence<=self.delivered or self.latest is None:return False,None
            self.processing_skipped_frames+=max(0,self.sequence-self.delivered-1)
            self.delivered=self.sequence
            frame,self.captured_at=self.latest
            return True,frame

    def metrics(self):
        with self.condition:
            elapsed=self.arrivals[-1]-self.arrivals[0] if len(self.arrivals)>1 else 0.
            return {'camera_received_frames':self.sequence,
                    'camera_measured_fps':(len(self.arrivals)-1)/elapsed if elapsed>0 else 0.,
                    'inference_skipped_old_frames':self.processing_skipped_frames}

    def release(self):
        self.stopped.set()
        self.source.release()
        with self.condition:self.condition.notify_all()
        if threading.current_thread() is not self.reader:self.reader.join(timeout=2.)

    def __getattr__(self,name):
        return getattr(self.source,name)


def _decode_fourcc(value: float, fallback: str) -> str:
    encoded = int(value)
    decoded = "".join(chr((encoded >> (8 * index)) & 0xFF) for index in range(4))
    return decoded.strip("\x00 ") or fallback


def open_camera(config: OnboardConfig) -> OpenedCamera:
    configured = config.camera.device
    candidates = camera_candidates(configured)
    if not candidates:
        raise RuntimeError("no USB UVC capture camera was discovered")

    failures: list[str] = []
    for device in candidates:
        source: int | str = int(device) if device.isdigit() else device
        for requested_format in ("MJPG", "YUYV"):
            capture = cv2.VideoCapture(source, cv2.CAP_V4L2)
            capture.set(
                cv2.CAP_PROP_FOURCC,
                cv2.VideoWriter_fourcc(*requested_format),
            )
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, config.camera.width)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, config.camera.height)
            capture.set(cv2.CAP_PROP_FPS, config.camera.fps)
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if not capture.isOpened():
                capture.release()
                failures.append(f"{device}/{requested_format}: open failed")
                continue

            frame = None
            for _ in range(CAMERA_WARMUP_FRAMES):
                ok, candidate_frame = capture.read()
                if ok and candidate_frame is not None:
                    frame = candidate_frame
            if frame is None:
                capture.release()
                failures.append(f"{device}/{requested_format}: no frame")
                continue

            height, width = (int(value) for value in frame.shape[:2])
            if config.camera.calibration_confirmed and (
                width != config.camera.width or height != config.camera.height
            ):
                capture.release()
                failures.append(
                    f"{device}/{requested_format}: negotiated {width}x{height}, "
                    f"but calibrated mode requires {config.camera.width}x"
                    f"{config.camera.height}"
                )
                continue
            actual_fps = float(capture.get(cv2.CAP_PROP_FPS))
            if actual_fps <= 0.0:
                actual_fps = float(config.camera.fps)
            pixel_format = _decode_fourcc(
                capture.get(cv2.CAP_PROP_FOURCC), requested_format
            )
            try:
                resolved_device = str(Path(device).resolve(strict=True))
            except OSError:
                resolved_device = str(device)
            return OpenedCamera(
                capture,
                device=str(device),
                resolved_device=resolved_device,
                width=width,
                height=height,
                fps=actual_fps,
                pixel_format=pixel_format,
                pending_frame=frame,
            )

    mode = "auto-discovered" if is_auto_camera_device(configured) else "configured"
    detail = "; ".join(failures[-8:])
    raise RuntimeError(f"cannot open {mode} USB camera: {detail}")


class CameraVisionWorker(threading.Thread):
    """Read UVC frames and run YOLO without blocking MAVLink reception."""

    def __init__(
        self,
        config: OnboardConfig,
        telemetry_provider: TelemetryProvider,
        runtime_dir: Path,
    ):
        super().__init__(name="cuadc-usb-vision", daemon=True)
        self.config = config
        self.telemetry_provider = telemetry_provider
        self.runtime_dir = runtime_dir
        self.stop_event = threading.Event()
        self.ready_event = threading.Event()
        self.lock = threading.Lock()
        self.capture_lock = threading.Lock()
        self._capture = None
        self._snapshot: dict = {
            "source": "STARTING",
            "banner": "OPENING USB CAMERA / LOADING MODEL",
            "camera_configured_device": self.config.camera.device,
            "camera_auto": is_auto_camera_device(self.config.camera.device),
            "detections": [],
            "visible_detections": [],
            "updated_at": 0.0,
        }
        self.error: str | None = None
        self.frame_age_s = float("inf")

    def stop(self) -> None:
        self.stop_event.set()
        # V4L2 may be sleeping in a USB URB inside capture.read().  Releasing
        # the handle from the shutdown thread asks the driver to cancel that
        # I/O instead of leaving systemd to SIGKILL the whole service.
        with self.capture_lock:
            capture = self._capture
        if capture is not None:
            try:
                capture.release()
            except Exception:
                pass

    def _set_capture(self, capture) -> None:
        with self.capture_lock:
            self._capture = capture

    def _clear_capture(self, capture) -> None:
        with self.capture_lock:
            if self._capture is capture:
                self._capture = None

    def snapshot(self) -> dict:
        with self.lock:
            return json.loads(json.dumps(self._snapshot))

    def _publish(self, payload: dict) -> None:
        with self.lock:
            self._snapshot = payload

    def _write_outputs(self, state: dict, frame) -> None:
        self.runtime_dir.mkdir(parents=True, exist_ok=True)
        state_path = self.runtime_dir / "vision_state.json"
        state_tmp = state_path.with_suffix(".json.tmp")
        state_tmp.write_text(
            json.dumps(state, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        os.replace(state_tmp, state_path)
        if not self.config.camera.save_preview:
            return
        ok, encoded = cv2.imencode(
            ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 82]
        )
        if ok:
            frame_path = self.runtime_dir / "vision_frame.jpg"
            frame_tmp = self.runtime_dir / "vision_frame.jpg.tmp"
            frame_tmp.write_bytes(encoded.tobytes())
            os.replace(frame_tmp, frame_path)

    def run(self) -> None:
        capture = None
        try:
            roll, pitch, yaw = camera_angles_rad(self.config.camera)
            camera_matrix = None
            if self.config.camera.fx_px > 0.0:
                camera_matrix = np.array(
                    [
                        [self.config.camera.fx_px, 0.0, self.config.camera.cx_px],
                        [0.0, self.config.camera.fy_px, self.config.camera.cy_px],
                        [0.0, 0.0, 1.0],
                    ],
                    dtype=float,
                )
            engine = VisionEngine(
                {
                    "task": self.config.vision.task,
                    "selected_group": "REAL",
                },
                confidence=self.config.vision.confidence,
                weights=self.config.vision.weights_path(),
                inference_hz=self.config.vision.inference_hz,
                inference_angles=self.config.vision.angles,
                inference_size=self.config.vision.inference_size(),
                device=self.config.vision.device,
                horizontal_fov_rad=self.config.camera.horizontal_fov_deg
                * 3.141592653589793
                / 180.0,
                telemetry_alignment_s=self.config.camera.telemetry_alignment_s,
                agl_offset_m=self.config.camera.agl_offset_m,
                camera_roll_rad=roll,
                camera_pitch_rad=pitch,
                camera_yaw_rad=yaw,
                camera_matrix=camera_matrix,
                distortion_coefficients=self.config.camera.distortion,
            )
            if engine.model is None:
                raise RuntimeError(engine.state.message)
            last_output = 0.0
            failed_reads = 0
            while not self.stop_event.is_set():
                if capture is None:
                    try:
                        capture = LatestFrameCamera(open_camera(self.config))
                        self._set_capture(capture)
                        if self.stop_event.is_set():
                            self._clear_capture(capture)
                            capture.release()
                            capture = None
                            break
                        self.error = None
                        failed_reads = 0
                        self.ready_event.set()
                    except Exception as exc:
                        self.error = f"{type(exc).__name__}: {exc}"
                        self._publish(
                            {
                                "source": "VISION_WAITING_FOR_CAMERA",
                                "banner": "WAITING FOR USB CAMERA",
                                "message": self.error,
                                "camera_configured_device": self.config.camera.device,
                                "camera_auto": is_auto_camera_device(
                                    self.config.camera.device
                                ),
                                "detections": [],
                                "visible_detections": [],
                                "updated_at": time.time(),
                            }
                        )
                        self.ready_event.set()
                        self.stop_event.wait(self.config.camera.reconnect_s)
                        continue
                ok, frame = capture.read()
                captured_at = capture.captured_at
                if not ok or frame is None:
                    failed_reads += 1
                    if failed_reads >= 30:
                        self._clear_capture(capture)
                        capture.release()
                        capture = None
                        self.error = "USB camera returned 30 consecutive empty frames"
                        self._publish(
                            {
                                "source": "VISION_WAITING_FOR_CAMERA",
                                "banner": "USB CAMERA DISCONNECTED",
                                "message": self.error,
                                "camera_configured_device": self.config.camera.device,
                                "camera_auto": is_auto_camera_device(
                                    self.config.camera.device
                                ),
                                "detections": [],
                                "visible_detections": [],
                                "updated_at": time.time(),
                            }
                        )
                        self.stop_event.wait(self.config.camera.reconnect_s)
                        continue
                    time.sleep(0.02)
                    continue
                failed_reads = 0
                telemetry = self.telemetry_provider()
                telemetry["camera_frame_wall_time_s"]=captured_at
                navigation_ready = bool(
                    telemetry.get("position_valid")
                    and telemetry.get("home_valid")
                    and float(telemetry.get("relative_alt_m", 0.0)) >= 2.0
                )
                if navigation_ready:
                    annotated = engine.process(frame, telemetry)
                else:
                    annotated = frame
                    engine.state.frame_count += 1
                    engine.state.updated_at = captured_at
                    engine.state.message = (
                        "Waiting for HOME_POSITION and valid flight telemetry"
                    )
                state = asdict(engine.state)
                state["camera_connected"] = True
                state["camera_captured_at"] = captured_at
                state["camera_configured_device"] = self.config.camera.device
                state["camera_auto"] = is_auto_camera_device(
                    self.config.camera.device
                )
                state["camera_device"] = capture.device
                state["camera_resolved_device"] = capture.resolved_device
                state["camera_identity"] = capture.identity
                state["camera_width"] = capture.width
                state["camera_height"] = capture.height
                state["camera_fps"] = capture.fps
                state.update(capture.metrics())
                state["camera_pixel_format"] = capture.pixel_format
                state["navigation_ready"] = navigation_ready
                self.frame_age_s = max(0.0, time.time() - captured_at)
                self._publish(state)
                now = time.monotonic()
                if now - last_output >= 0.5:
                    self._write_outputs(state, annotated)
                    last_output = now
        except Exception as exc:
            self.error = f"{type(exc).__name__}: {exc}"
            self._publish(
                {
                    "source": "VISION_FAILED",
                    "banner": "USB CAMERA / MODEL FAILED",
                    "message": self.error,
                    "camera_configured_device": self.config.camera.device,
                    "camera_auto": is_auto_camera_device(
                        self.config.camera.device
                    ),
                    "detections": [],
                    "visible_detections": [],
                    "updated_at": time.time(),
                }
            )
            self.ready_event.set()
        finally:
            if capture is not None:
                self._clear_capture(capture)
                capture.release()
