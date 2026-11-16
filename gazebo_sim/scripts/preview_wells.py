#!/usr/bin/env python3
"""用独立 Gazebo 场景输出天井俯视和斜视，不启动飞控。"""
from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from cuadc_sim.scenario import Scenario, TargetGroup, Well, wells_to_sdf
from cuadc_sim.well_geometry import floor_mesh_obj


def camera(name: str, x: float, y: float, z: float) -> str:
    yaw = math.atan2(0.4 - y, -x) if x or y != 0.4 else 0.0
    pitch = math.atan2(z - 0.2, math.hypot(x, y - 0.4))
    return f'''<model name="{name}"><static>true</static>
      <pose>{x} {y} {z} 0 {pitch} {yaw}</pose><link name="camera">
      <sensor name="camera" type="camera"><always_on>true</always_on><update_rate>5</update_rate>
      <topic>/cuadc/preview/{name}</topic><camera><horizontal_fov>0.9</horizontal_fov>
      <image><width>1280</width><height>960</height><format>R8G8B8</format></image>
      <clip><near>0.05</near><far>100</far></clip></camera></sensor></link></model>'''


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / ".runtime/well-preview")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    runtime = ROOT / ".runtime"
    runtime.mkdir(exist_ok=True)
    (runtime / "well_floor.obj").write_text(floor_mesh_obj())
    groups = []
    for group, x, asset, value, heading in (
        ("A", -0.8, "assets/targets/reference/task1/fighter.png", 9, 0),
        ("B", 0.8, "assets/targets/reference/task2/number-94.png", 94, -15),
    ):
        well = Well(group, group, True, x, 0.0, heading, heading, value, str(value), asset)
        groups.append(TargetGroup(group, group, x, 0.0, (well,)))
    scenario = Scenario("preview", 1, 1, "A", tuple(groups))
    world = f'''<?xml version="1.0"?><sdf version="1.9"><world name="well_preview">
      <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
      <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
      <scene><ambient>0.45 0.45 0.45 1</ambient><background>0.7 0.8 0.9 1</background><shadows>true</shadows></scene>
      <light name="sun" type="directional"><pose>0 0 10 0 0 0</pose><diffuse>0.8 0.8 0.78 1</diffuse>
        <direction>-0.4 0.5 -1</direction><cast_shadows>true</cast_shadows></light>
      <model name="ground"><static>true</static><link name="ground"><visual name="ground">
        <geometry><plane><normal>0 0 1</normal><size>30 30</size></plane></geometry>
        <material><ambient>0.45 0.48 0.38 1</ambient><diffuse>0.65 0.68 0.55 1</diffuse>
          <pbr><metal><albedo_map>model://cuadc_world_assets/materials/textures/Grass004_2K_Color.jpg</albedo_map>
            <roughness>0.97</roughness><metalness>0</metalness></metal></pbr></material>
      </visual></link></model>
      <model name="practice_apron"><static>true</static><link name="pad"><visual name="concrete">
        <pose>0 0 0.005 0 0 0</pose><geometry><box><size>8 8 0.008</size></box></geometry>
        <material><ambient>0.65 0.65 0.65 1</ambient><diffuse>0.8 0.8 0.78 1</diffuse>
        <pbr><metal><albedo_map>model://cuadc_world_assets/materials/textures/concrete_synthetic.jpg</albedo_map>
        <roughness>0.97</roughness><metalness>0</metalness></metal></pbr></material>
      </visual></link></model>
      {wells_to_sdf(scenario)}
      {camera("top", 0, 0.4, 4.5)}
      {camera("angle", 2.8, -3.8, 2.8)}
    </world></sdf>'''
    world_path = runtime / "well_preview.sdf"
    world_path.write_text(world)
    env = os.environ.copy()
    env.update(GZ_PARTITION="cuadc_well_preview", GZ_IP="127.0.0.1", ROS_DOMAIN_ID="88",
               GZ_SIM_RESOURCE_PATH=str(ROOT / "models"))
    os.environ["ROS_DOMAIN_ID"] = "88"
    import cv2
    import numpy as np
    import rclpy
    from sensor_msgs.msg import Image

    processes, subscriptions, received = [], [], set()
    rclpy.init()
    node = rclpy.create_node("cuadc_well_preview_capture")
    started = time.monotonic()

    def save(message, name):
        if time.monotonic() - started < 8 or name in received:
            return
        frame = np.frombuffer(message.data, np.uint8).reshape(message.height, message.step)[:, :message.width * 3]
        frame = frame.reshape(message.height, message.width, 3)
        if message.encoding.lower() == "rgb8":
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        if not cv2.imwrite(str(args.output / f"{name}.png"), frame):
            raise RuntimeError("预览图片写入失败")
        received.add(name)

    try:
        with (runtime / "well-preview.log").open("w") as log:
            commands = [
                ["gz", "sim", "-s", "-r", str(world_path)],
                ["/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge", *[
                    f"/cuadc/preview/{name}@sensor_msgs/msg/Image[gz.msgs.Image" for name in ("top", "angle")
                ]],
            ]
            for command in commands:
                processes.append(subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
            for name in ("top", "angle"):
                subscriptions.append(node.create_subscription(Image, f"/cuadc/preview/{name}", lambda msg, name=name: save(msg, name), 1))
            while len(received) < 2 and time.monotonic() - started < 45:
                if any(process.poll() is not None for process in processes):
                    raise RuntimeError("预览进程退出，检查 .runtime/well-preview.log")
                rclpy.spin_once(node, timeout_sec=0.2)
            if len(received) < 2:
                raise TimeoutError("未收到两个相机的真实渲染画面")
    finally:
        for process in processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
        for process in processes:
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        node.destroy_node()
        rclpy.shutdown()
    print(f"[天井预览] 已输出俯视和斜视：{args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
