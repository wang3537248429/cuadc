from __future__ import annotations

import argparse
import ipaddress
import json
import os
import secrets
import shutil
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path

from .config import (
    FDM_PORT,
    GAZEBO_DIR,
    ORIGIN_ALT_M,
    ORIGIN_LAT_DEG,
    ORIGIN_LON_DEG,
    ROOT,
    RUNTIME_DIR,
    SITL_INSTANCE,
    SITL_TCP_PORT,
    SIM_CONFIG_DIR,
    WMS_PORT,
    SimParameters,
)
from .scenario import write_runtime
from .field_layout import load_field_layout


ARDUPILOT_RUNTIME = ROOT / ".deps/runtime/ardupilot-4.6.3"
ARDUPILOT_GAZEBO_RUNTIME = ROOT / ".deps/runtime/ardupilot-gazebo-garden"


class ProcessSupervisor:
    def __init__(self, headless: bool = False, seed: int | None = None,
                 gcs_host: str = '127.0.0.1', official_route: str | None = None,
                 preserve_uploaded_route: bool = False,
                 vision: bool = True):
        self.headless = headless
        self.vision_enabled = vision
        self.gcs_host=str(ipaddress.IPv4Address(gcs_host))
        self.official_route = official_route.upper() if official_route else None
        if self.official_route:
            route_config = SIM_CONFIG_DIR / f"arena_official_{self.official_route.lower()}.json"
            if not route_config.is_file():
                raise ValueError(f"official route configuration not found: {route_config}")
            os.environ.setdefault("CUADC_ARENA_CONFIG", str(route_config))
            try:
                route_data = json.loads(route_config.read_text(encoding="utf-8"))
                route_distance = float(
                    route_data.get("field_layout", {}).get("route_max_distance_m", 0.0)
                )
            except (OSError, json.JSONDecodeError, TypeError, ValueError) as exc:
                raise ValueError(f"invalid official route configuration: {route_config}") from exc
            # Keep 50 m of margin beyond the farthest checked-in waypoint.
            # The ordinary 300 m competition baseline remains unchanged for
            # all other simulator runs.
            boundary = max(300.0, route_distance + 50.0)
            os.environ["CUADC_MISSION_MAX_HOME_DISTANCE_M"] = f"{boundary:.1f}"
            # The official files are still uploaded unchanged by the ground
            # station.  The companion may replace only the four explicitly
            # reserved attack slots after YOLO has produced a complete target
            # fix.  Forcing RAW_REPLAY here made those slots unreachable and
            # silently degraded every official run to the fixed drop command.
        if preserve_uploaded_route:
            os.environ["CUADC_PRESERVE_UPLOADED_ROUTE"] = "1"
        self.randomize_tasks = seed is None
        self.seed = secrets.randbelow(1 << 24) if seed is None else seed
        if not 0 <= self.seed < (1 << 24):
            raise ValueError("seed must be within 0..16777215 for MAVLink float32")
        self.processes: dict[str, subprocess.Popen] = {}
        self.logs: dict[str, object] = {}
        self.started_log_names: set[str] = set()
        self.stopping = False
        self.runtime_python = (
            ROOT / ".venv/bin/python"
            if (ROOT / ".venv/bin/python").is_file()
            else Path(sys.executable)
        )
        self.env = os.environ.copy()
        self.env["PATH"] = os.pathsep.join(
            [str(ROOT / ".venv/bin"), self.env.get("PATH", "")]
        )
        self.env["CUADC_PROJECT_ROOT"] = str(ROOT)
        if preserve_uploaded_route:
            self.env["CUADC_PRESERVE_UPLOADED_ROUTE"] = "1"
        self.env["PYTHONPATH"] = ":".join(
            [
                str(ROOT / "src"),
                "/opt/ros/humble/lib/python3.10/site-packages",
                "/opt/ros/humble/local/lib/python3.10/dist-packages",
                self.env.get("PYTHONPATH", ""),
            ]
        )
        self.env["GZ_VERSION"] = "garden"
        self.env["OMP_NUM_THREADS"] = "2"
        self.env["MKL_NUM_THREADS"] = "2"
        self.env["OPENBLAS_NUM_THREADS"] = "1"
        # Keep this simulator isolated from any PX4 / Gazebo worlds already
        # running on the workstation.  Without a partition, the GUI discovers
        # every local world, renders unrelated models and receives duplicate
        # /clock and /stats publishers.
        self.env["GZ_PARTITION"] = "cuadc_garden_79"
        self.env["GZ_IP"] = "127.0.0.1"
        self.env["YOLO_CONFIG_DIR"] = str(RUNTIME_DIR / "ultralytics")
        self.env["TORCH_HOME"] = str(RUNTIME_DIR / "torch-cache")
        self.env["MPLCONFIGDIR"] = str(RUNTIME_DIR / "matplotlib")
        self.env["GZ_SIM_RESOURCE_PATH"] = ":".join(
            [
                str(GAZEBO_DIR / "models"),
                self.env.get("GZ_SIM_RESOURCE_PATH", ""),
            ]
        )
        self.env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = ":".join(
            [
                str(ROOT / ".runtime/plugins-build"),
                str(ARDUPILOT_GAZEBO_RUNTIME),
                "/usr/lib/x86_64-linux-gnu/gz-sim-7/plugins",
                self.env.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""),
            ]
        )

    def check(self) -> None:
        required = {
            "Gazebo Garden": shutil.which("gz"),
            "ROS 2": shutil.which("ros2"),
            "ROS/Gazebo bridge": Path(
                "/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge"
            ),
            "MAVProxy": shutil.which("mavproxy.py", path=self.env["PATH"]),
            "ArduPlane SITL": ARDUPILOT_RUNTIME / "arduplane",
            "ArduPilot Gazebo plugin": (
                ARDUPILOT_GAZEBO_RUNTIME / "libArduPilotPlugin.so"
            ),
            "CUADC hand-launch plugin": ROOT / ".runtime/plugins-build/libCuadcHandLaunchSystem.so",
            "CUADC twin-motor plugin": ROOT / ".runtime/plugins-build/libCuadcTwinMotorSystem.so",
            "CUADC scenario-state plugin": ROOT / ".runtime/plugins-build/libCuadcScenarioStateSystem.so",
        }
        missing = [name for name, path in required.items() if not path or not Path(path).exists()]
        if missing:
            raise RuntimeError(
                "missing runtime dependencies: "
                + ", ".join(missing)
                + ". Run the Gazebo package bootstrap.sh first."
            )
        version = subprocess.check_output(["gz", "sim", "--versions"], text=True).strip()
        if not version.startswith("7.9"):
            raise RuntimeError(f"Gazebo Garden 7.9 required, found {version}")
        for protocol, port in (
            (socket.SOCK_STREAM, SITL_TCP_PORT),
            (socket.SOCK_DGRAM, FDM_PORT),
            (socket.SOCK_STREAM, WMS_PORT),
        ):
            with socket.socket(socket.AF_INET, protocol) as probe:
                if protocol == socket.SOCK_STREAM:
                    probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                try:
                    probe.bind(("127.0.0.1", port))
                except OSError as exc:
                    raise RuntimeError(
                        f"private SITL port {port} is already in use; "
                        "stop the conflicting process before launch"
                    ) from exc

    def _start(self, name: str, command: list[str], cwd: Path = ROOT) -> None:
        log_dir = RUNTIME_DIR / "logs"
        log_dir.mkdir(parents=True, exist_ok=True)
        # Start each top-level run with clean logs, but append when this same
        # supervisor intentionally restarts the Gazebo/SITL side after a
        # disarmed scenario parameter change.
        mode = "a" if name in self.started_log_names else "w"
        log = (log_dir / f"{name}.log").open(mode, encoding="utf-8")
        self.started_log_names.add(name)
        self.logs[name] = log
        self.processes[name] = subprocess.Popen(
            command,
            cwd=cwd,
            env=self.env,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        print(f"[CUADC] started {name} (pid {self.processes[name].pid})", flush=True)

    def _stop(self, name: str, timeout: float = 8.0) -> None:
        process = self.processes.pop(name, None)
        if process:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            if process.poll() is None:
                try:
                    process.wait(timeout=timeout)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=2)
        log = self.logs.pop(name, None)
        if log:
            log.close()

    def start_gazebo_side(self) -> None:
        self._start(
            "gazebo",
            [
                "gz",
                "sim",
                "-s",
                "-r",
                "-v",
                "2",
                str(RUNTIME_DIR / "cuadc_world.sdf"),
            ],
        )
        if not self.headless:
            self._start("gazebo_gui", ["gz", "sim", "-g", "-v", "2"])
        time.sleep(2.0)
        bridge_command = [
            "/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge",
            "/cuadc/camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/cuadc/payload/release_latched@std_msgs/msg/Bool[gz.msgs.Boolean",
            "/cuadc/payload/impact@std_msgs/msg/String[gz.msgs.StringMsg",
            "/cuadc/airframe/touchdown@std_msgs/msg/String[gz.msgs.StringMsg",
        ]
        self._start("bridge", bridge_command)
        if self.vision_enabled:
            vision_command = [
                str(self.runtime_python),
                "-m",
                "cuadc_sim.cli",
                "vision",
            ]
            if self.headless:
                vision_command.append("--headless")
            self._start("vision", vision_command)
        else:
            print(
                "[CUADC] --no-vision：未启动 YOLO 视觉服务；相机画面仍由 Gazebo 发布。",
                flush=True,
            )

    def stop_gazebo_side(self) -> None:
        self._stop("vision")
        self._stop("bridge")
        self._stop("gazebo_gui")
        self._stop("gazebo")

    def start_autopilot_side(self) -> None:
        sitl_dir = RUNTIME_DIR / "sitl"
        sitl_dir.mkdir(parents=True, exist_ok=True)
        # ArduPilot invokes these helpers by basename on SIGFPE / SIGABRT.
        # Keeping the links in the private SITL directory makes any future
        # crash produce an actionable stack/core without touching the source
        # checkout or the user's separate ArduPilot workspace.
        script_dir = ARDUPILOT_RUNTIME
        for name in ("dumpstack.sh", "dumpcore.sh"):
            link = sitl_dir / name
            source = script_dir / name
            if link.is_symlink():
                if link.resolve(strict=False) == source.resolve():
                    continue
                link.unlink()
            elif link.exists():
                link.unlink()
            link.symlink_to(source)
        layout = load_field_layout(RUNTIME_DIR / "field_layout.json")
        ardupilot_command = [
            str(ARDUPILOT_RUNTIME / "arduplane"),
            "-S",
            "--model",
            "JSON",
            "--speedup",
            "1",
            "--slave",
            "0",
            "--defaults",
            str(SIM_CONFIG_DIR / "ardupilot/plane.parm"),
            "--home",
            f"{layout.origin_lat_deg},{layout.origin_lon_deg},{layout.origin_alt_m},{layout.initial_heading_deg}",
            "--sim-address=127.0.0.1",
            f"-I{SITL_INSTANCE}",
        ]
        if self.env.get("CUADC_SITL_GDB") == "1":
            ardupilot_command = [
                "gdb",
                "--batch",
                "-ex",
                "handle SIGPIPE nostop noprint pass",
                "-ex",
                "run",
                "-ex",
                "thread apply all bt full",
                "--args",
                *ardupilot_command,
            ]
        self._start("ardupilot", ardupilot_command, cwd=sitl_dir)
        time.sleep(1.0)
        self._start(
            "router",
            [
                "mavproxy.py",
                "--daemon",
                "--non-interactive",
                f"--master=tcp:127.0.0.1:{SITL_TCP_PORT}",
                f"--out=udp:{self.gcs_host}:14550",
                "--out=udp:127.0.0.1:14551",
            ],
            cwd=sitl_dir,
        )
        self._start(
            "companion",
            [
                str(self.runtime_python),
                "-m",
                "cuadc_sim.cli",
                "companion",
                "--endpoint",
                "udpin:0.0.0.0:14551",
            ],
        )

    def stop_autopilot_side(self) -> None:
        self._stop("companion")
        self._stop("router")
        self._stop("ardupilot")

    def restart_physics_stack(self) -> None:
        # A generated world can change the initial yaw by 180 degrees. Keeping
        # an already-aligned EKF alive across that discontinuity produced a
        # DCM yaw inconsistency and a SITL floating-point abort. Restart the
        # complete private stack so Gazebo truth and flight-controller state
        # always begin from the same physical initial condition. Parameters
        # and the uploaded mission remain in the private EEPROM file.
        self.stop_autopilot_side()
        self.stop_gazebo_side()
        self.start_gazebo_side()
        self.start_autopilot_side()

    def start(self) -> int:
        self.check()
        RUNTIME_DIR.mkdir(parents=True, exist_ok=True)
        (RUNTIME_DIR / 'task_policy.json').write_text(json.dumps({
            'randomize_each_upload': self.randomize_tasks,
            'fixed_seed': None if self.randomize_tasks else self.seed,
        })+'\n')
        for artifact in (
            "restart.request",
            "restart.started",
            "camera_frame.packet",
            "camera_frame.jpg",
            "camera_detail.jpg",
            "payload_released.flag",
            "impact.json",
            "telemetry.jsonl",
            "vision_state.json",
            "vision_frame.jpg",
            "last_detection_frame.jpg",
            "route_state.json",
        ):
            (RUNTIME_DIR / artifact).unlink(missing_ok=True)
        # The companion applies this one-shot request to the isolated SITL.
        # Later Mission Planner parameter changes remain authoritative.
        (RUNTIME_DIR / "startup_seed.json").write_text(json.dumps({"seed": self.seed}))
        write_runtime(SimParameters(seed=self.seed))
        print(f"[CUADC] 场景随机种子：{self.seed}（--seed 可复现）", flush=True)

        self._start(
            "wms",
            [str(self.runtime_python), "-m", "cuadc_sim.cli", "wms"],
        )
        self.start_gazebo_side()
        self.start_autopilot_side()
        print(
            "\n[CUADC] Mission Planner: choose UDP, port 14550.\n"
            "[CUADC] Upload a mission based on the Gazebo package config/missions template,\n"
            f"[CUADC] Local WMS: http://127.0.0.1:{WMS_PORT}/wms?\n"
            "[CUADC] then arm in AUTO. Logs: .runtime/logs/\n",
            flush=True,
        )
        if self.official_route:
            print(
                f"[CUADC] Official route profile {self.official_route}: "
                f"upload config/missions/official_route_{self.official_route.lower()}.waypoints; "
                "the uploaded route remains the flight-control source; YOLO may patch only its reserved attack slots.",
                flush=True,
            )

        while not self.stopping:
            time.sleep(0.5)
            restart = RUNTIME_DIR / "restart.request"
            if restart.exists():
                (RUNTIME_DIR/'restart.started').write_text(str(time.time())+'\n')
                print(
                    "[CUADC] disarmed parameter change: restarting Gazebo and "
                    "private ArduPlane stack",
                    flush=True,
                )
                restart.unlink(missing_ok=True)
                self.restart_physics_stack()
            for name, process in list(self.processes.items()):
                code = process.poll()
                if code is not None and name not in ("vision",):
                    raise RuntimeError(f"{name} exited unexpectedly with code {code}")
        return 0

    def shutdown(self, *_args) -> None:
        if self.stopping:
            return
        self.stopping = True
        pending = list(self.processes.items())
        for _, process in pending:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass

        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            if all(process.poll() is not None for _, process in pending):
                break
            time.sleep(0.1)

        for name, process in pending:
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                pass
            self.processes.pop(name, None)
            log = self.logs.pop(name, None)
            if log:
                log.close()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="CUADC Gazebo/SITL process supervisor")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--seed", type=int, help="指定场景种子；省略时每次启动随机生成")
    parser.add_argument('--gcs-host',default=os.environ.get('CUADC_GCS_HOST','127.0.0.1'),
                        help='Mission Planner 所在电脑的 IPv4；默认本机')
    parser.add_argument(
        "--official-route",
        choices=("A", "B", "a", "b"),
        help="使用公开 A/B 示例航线场地配置；仍需由 Mission Planner 上传对应航线",
    )
    parser.add_argument(
        "--preserve-uploaded-route",
        action="store_true",
        help="保留地面站上传的航点，不启用动态攻击段替换",
    )
    parser.add_argument(
        "--no-vision",
        action="store_true",
        help="不启动 YOLO 服务；用于没有本地模型/靶标资源的基础仿真",
    )
    args = parser.parse_args(argv)
    supervisor = ProcessSupervisor(
        headless=args.headless,
        seed=args.seed,
        gcs_host=args.gcs_host,
        official_route=args.official_route,
        preserve_uploaded_route=args.preserve_uploaded_route,
        vision=not args.no_vision,
    )
    signal.signal(signal.SIGINT, supervisor.shutdown)
    signal.signal(signal.SIGTERM, supervisor.shutdown)
    try:
        return supervisor.start()
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        print(f"[CUADC] fatal: {exc}", file=sys.stderr)
        return 2
    finally:
        supervisor.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
