from __future__ import annotations

import json
import os
import sys
import threading
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np

from .config import (RUNTIME_DIR, SIM_CAMERA_AGL_OFFSET_M, DOWN_CAMERA_HORIZONTAL_FOV_RAD,
                     SIM_VISION_INFERENCE_HZ)
from .camera_detail import target_details
from .vision import VisionEngine, load_json, save_vision_state
from .camera_stream import LatestFrameMailbox
from .camera_packet import pack_frame


class RosVisionNode:
    def __init__(self, confidence: float = 0.20):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import Image, CompressedImage
        from std_msgs.msg import Bool, String

        parameters = load_json(RUNTIME_DIR / "parameters.json")
        if not parameters:
            raise RuntimeError("parameters.json is missing; start the simulator first")
        public_context = {
            "task": int(parameters["task"]),
            "selected_group": str(parameters["group"]),
        }
        self.rclpy = rclpy
        cv2.setNumThreads(1)
        self.node = Node("cuadc_vision")
        self.public_context = public_context
        self.confidence = confidence
        self.compressed_message_type=CompressedImage
        self.preview_publisher=self.node.create_publisher(CompressedImage,'/cuadc/camera/compressed',
            QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.engine = None
        self.last_metrics_saved_at = 0.0
        self.arrivals = deque(maxlen=120)
        self.mailbox = LatestFrameMailbox()
        self.inference_stop = threading.Event()
        # Inference may be slower than 30 fps. Never back-pressure the camera
        # bridge or accumulate stale images while YOLO is working.
        # Large native RGB frames need reliable fragment retransmission.
        # Depth one and the latest-only mailbox still bound latency and memory.
        camera_qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE)
        self.node.create_subscription(Image, "/cuadc/camera/image", self._on_image, camera_qos)
        self.node.create_subscription(
            Bool, "/cuadc/payload/release_latched", self._on_payload_release, 10
        )
        self.node.create_subscription(
            String, "/cuadc/payload/impact", self._on_impact, 10
        )
        self.node.create_subscription(
            String, "/cuadc/airframe/touchdown", self._on_touchdown, 10
        )
        self.inference_thread = threading.Thread(
            target=self._infer, name="cuadc-yolo", daemon=True
        )
        self.inference_thread.start()

    def _on_image(self, message) -> None:
        image = np.frombuffer(message.data, dtype=np.uint8)
        expected = int(message.height) * int(message.step)
        if image.size < expected:
            return
        if message.encoding.lower() not in {"rgb8", "bgr8"}:
            return
        image = image[:expected].reshape(message.height, message.step)
        image = image[:, :message.width*3].reshape(message.height, message.width, 3)
        if message.encoding.lower() == "rgb8":
            frame = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        else:
            frame = image.copy()
        telemetry = load_json(RUNTIME_DIR / "telemetry.json")
        telemetry["camera_frame_time_s"]=message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        now = time.monotonic()
        self.arrivals.append(now)
        self.mailbox.put((frame, telemetry))
        # The live display follows capture, not the duration of YOLO inference.
        # Annotated inference frames remain in vision_frame.jpg independently.
        ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 94])
        if ok:
            jpeg=encoded.tobytes()
            temporary = RUNTIME_DIR / "camera_frame.jpg.tmp"
            temporary.write_bytes(jpeg)
            os.replace(temporary, RUNTIME_DIR / "camera_frame.jpg")
            temporary=RUNTIME_DIR/'camera_frame.packet.tmp'
            temporary.write_bytes(pack_frame(self.mailbox.sequence,telemetry['camera_frame_time_s'],jpeg))
            os.replace(temporary,RUNTIME_DIR/'camera_frame.packet')
            preview=self.compressed_message_type()
            preview.header=message.header
            preview.format='jpeg'
            preview.data=jpeg
            self.preview_publisher.publish(preview)
        tracks = list(self.engine.state.visible_detections) if self.engine is not None else []
        detail = target_details(frame, telemetry, tracks, DOWN_CAMERA_HORIZONTAL_FOV_RAD,
                                SIM_CAMERA_AGL_OFFSET_M)
        if detail is not None:
            ok, encoded = cv2.imencode('.jpg', detail, [cv2.IMWRITE_JPEG_QUALITY, 94])
            if ok:
                temporary = RUNTIME_DIR / 'camera_detail.jpg.tmp'
                temporary.write_bytes(encoded.tobytes())
                os.replace(temporary, RUNTIME_DIR / 'camera_detail.jpg')
        else:
            (RUNTIME_DIR / 'camera_detail.jpg').unlink(missing_ok=True)
        if now - self.last_metrics_saved_at >= .5:
            elapsed = self.arrivals[-1]-self.arrivals[0] if len(self.arrivals)>1 else 0
            metrics = {"received_frames": self.mailbox.sequence,
                       "arrival_fps": (len(self.arrivals)-1)/elapsed if elapsed>0 else 0,
                       "inference_skipped_old_frames": self.mailbox.skipped}
            temporary = RUNTIME_DIR / "camera_stream.json.tmp"
            temporary.write_text(json.dumps(metrics)+"\n")
            os.replace(temporary, RUNTIME_DIR / "camera_stream.json")
            self.last_metrics_saved_at = now

    def _infer(self):
        # Initialize Torch and run inference in the same worker. OpenMP thread
        # pool initialization must not hold up the ROS capture executor.
        self.engine = VisionEngine(self.public_context, confidence=self.confidence,
                                   agl_offset_m=SIM_CAMERA_AGL_OFFSET_M,
                                   inference_hz=SIM_VISION_INFERENCE_HZ)
        self.node.get_logger().info(self.engine.state.banner)
        while not self.mailbox.closed:
            delay = max(0., self.engine.inference_period-(time.monotonic()-self.engine.last_inference))
            if self.inference_stop.wait(delay):
                break
            sample = self.mailbox.take()
            if sample is None:
                continue
            frame, telemetry = sample
            try:
                annotated = self.engine.process(frame, telemetry)
                save_vision_state(self.engine.state, annotated)
            except Exception as exc:
                self.node.get_logger().error(f"YOLO inference failed: {exc}")
                self.mailbox.close()

    def close(self):
        self.inference_stop.set()
        self.mailbox.close()
        self.inference_thread.join(timeout=10)

    def _on_impact(self, message) -> None:
        if not (RUNTIME_DIR / "payload_released.flag").exists():
            return
        impact_path = RUNTIME_DIR / "impact.json"
        if impact_path.exists():
            return
        try:
            payload = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            return
        payload["timestamp"] = time.time()
        temporary = impact_path.with_suffix(".json.tmp")
        temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        os.replace(temporary, impact_path)

    def _on_payload_release(self, message) -> None:
        if not bool(message.data):
            return
        release_flag = RUNTIME_DIR / "payload_released.flag"
        if not release_flag.exists():
            release_flag.write_text(f"{time.time():.6f}\n", encoding="utf-8")

    def _on_touchdown(self, message) -> None:
        touchdown_path = RUNTIME_DIR / "touchdown.json"
        if touchdown_path.exists():
            return
        try:
            payload = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            return
        payload["timestamp"] = time.time()
        temporary = touchdown_path.with_suffix(".json.tmp")
        temporary.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        os.replace(temporary, touchdown_path)

    def spin(self) -> None:
        from rclpy.executors import ExternalShutdownException

        try:
            self.rclpy.spin(self.node)
        except ExternalShutdownException:
            pass


def run_gui_in_thread() -> threading.Thread | None:
    try:
        from .gui import run_gui
    except ImportError as exc:
        print(f"CUADC GUI unavailable: {exc}", file=sys.stderr)
        return None
    thread = threading.Thread(target=run_gui, name="cuadc-gui", daemon=True)
    thread.start()
    return thread


def main(confidence: float = 0.20, gui: bool = True) -> int:
    import faulthandler
    import signal
    import rclpy

    faulthandler.register(signal.SIGUSR1, all_threads=True)
    rclpy.init(args=None)
    service = RosVisionNode(confidence=confidence)
    # Qt normally owns the main thread. The CLI starts the service in a worker
    # when GUI is available; headless operation keeps ROS in the main thread.
    if gui:
        try:
            from .gui import run_gui
            ros_thread = threading.Thread(target=service.spin, name="cuadc-ros", daemon=True)
            ros_thread.start()
            try:
                return run_gui()
            finally:
                service.close()
                service.node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
        except ImportError as exc:
            print(f"PySide6 not installed; continuing headless: {exc}", file=sys.stderr)
    try:
        service.spin()
    except KeyboardInterrupt:
        pass
    finally:
        service.close()
        service.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
