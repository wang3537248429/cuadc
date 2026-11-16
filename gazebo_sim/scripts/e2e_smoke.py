#!/usr/bin/env python3
"""Upload the reference mission and exercise the live SITL/Gazebo loop.

Run this only against a simulator started by the Gazebo package run_sim.sh. It uses the
same UDP endpoint as Mission Planner, waits for companion validation, selects
AUTO and arms.  By default it stops after Gazebo confirms the physical hand
launch; ``--full`` waits for payload impact, landing and mission restoration.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time
import subprocess
import signal

from pymavlink import mavutil, mavwp


APP_ROOT = Path(__file__).resolve().parents[1]
ROOT = APP_ROOT if (APP_ROOT / "src/cuadc_sim").is_dir() else APP_ROOT.parent
SIM_CONFIG_DIR = APP_ROOT / "config"
sys.path.insert(0,str(APP_ROOT/'src'))
from cuadc_sim.mavlink_io import wait_autopilot_heartbeat
from cuadc_sim.config import RUNTIME_DIR, RUNS_DIR
RESULT_PATH = RUNS_DIR / "latest.json"
PARAMETERS_PATH = RUNTIME_DIR / "parameters.json"


def set_parameter(connection, name: str, value: float, timeout: float = 8.0) -> None:
    encoded = name.encode("ascii")
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        connection.mav.param_set_send(
            connection.target_system,
            connection.target_component,
            encoded,
            float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        )
        message = connection.recv_match(
            type="PARAM_VALUE", blocking=True, timeout=1.0
        )
        if message is None:
            continue
        parameter_id = message.param_id
        if isinstance(parameter_id, bytes):
            parameter_id = parameter_id.decode("ascii", errors="ignore")
        if str(parameter_id).rstrip("\x00") != name:
            continue
        if abs(float(message.param_value) - float(value)) <= 0.01:
            return
    raise TimeoutError(f"ArduPlane did not accept {name}={value}")


def wait_scenario_restart(expected: dict, changed_at: float, timeout: float = 50.0) -> None:
    """Wait until the supervisor has rebuilt physics for requested MP params."""

    deadline = time.monotonic() + timeout
    matched_since: float | None = None
    while time.monotonic() < deadline:
        try:
            parameters = json.loads(PARAMETERS_PATH.read_text(encoding="utf-8"))
        except (FileNotFoundError, json.JSONDecodeError):
            parameters = {}
        matches = all(parameters.get(key) == value for key, value in expected.items())
        fresh = (
            PARAMETERS_PATH.is_file()
            and PARAMETERS_PATH.stat().st_mtime >= changed_at
        )
        if matches and fresh:
            if matched_since is None:
                matched_since = time.monotonic()
            # The process supervisor needs about 6 s to replace Gazebo, SITL,
            # bridge, vision and companion after parameters.json is written.
            if time.monotonic() - matched_since >= 10.0:
                print(f"[E2E] scenario ready: {expected}")
                return
        else:
            matched_since = None
        time.sleep(0.5)
    raise TimeoutError(f"simulator did not restart for {expected}")


def _send_item(connection, item, integer: bool) -> None:
    if integer:
        global_frame = item.frame in {
            mavutil.mavlink.MAV_FRAME_GLOBAL,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT,
            mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT,
        }
        x = int(round(item.x * 1e7)) if global_frame else int(round(item.x))
        y = int(round(item.y * 1e7)) if global_frame else int(round(item.y))
        connection.mav.mission_item_int_send(
            connection.target_system,
            connection.target_component,
            item.seq,
            item.frame,
            item.command,
            item.current,
            item.autocontinue,
            item.param1,
            item.param2,
            item.param3,
            item.param4,
            x,
            y,
            item.z,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
        )
        return
    connection.mav.mission_item_send(
        connection.target_system,
        connection.target_component,
        item.seq,
        item.frame,
        item.command,
        item.current,
        item.autocontinue,
        item.param1,
        item.param2,
        item.param3,
        item.param4,
        item.x,
        item.y,
        item.z,
        mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
    )


def upload_mission(connection, path: Path, timeout: float = 30.0) -> int:
    loader = mavwp.MAVWPLoader()
    loader.load(str(path))
    while connection.recv_match(blocking=False) is not None:
        pass
    connection.mav.mission_clear_all_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
    )
    clear_ack = connection.recv_match(
        type="MISSION_ACK",
        blocking=True,
        timeout=5.0,
    )
    if (
        clear_ack is None
        or clear_ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED
    ):
        result = "timeout" if clear_ack is None else clear_ack.type
        raise RuntimeError(f"mission clear rejected with ACK {result}")
    connection.mav.mission_count_send(
        connection.target_system,
        connection.target_component,
        loader.count(),
        mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
    )
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        message = connection.recv_match(
            type=["MISSION_REQUEST", "MISSION_REQUEST_INT", "MISSION_ACK"],
            blocking=True,
            timeout=2.0,
        )
        if message is None:
            continue
        if message.get_type() == "MISSION_ACK":
            if message.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                raise RuntimeError(f"mission upload rejected with ACK {message.type}")
            connection.mav.mission_set_current_send(
                connection.target_system,
                connection.target_component,
                0,
            )
            return loader.count()
        sequence = int(message.seq)
        if not 0 <= sequence < loader.count():
            raise RuntimeError(f"autopilot requested invalid mission sequence {sequence}")
        _send_item(
            connection,
            loader.wp(sequence),
            message.get_type() == "MISSION_REQUEST_INT",
        )
    raise TimeoutError("timed out uploading mission")


def read_result() -> dict:
    try:
        return json.loads(RESULT_PATH.read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError):
        return {}


def wait_ready(
    uploaded_at: float,
    timeout: float = 30.0,
    require_dynamic_release: bool = False,
) -> None:
    deadline = time.monotonic() + timeout
    last_status = ""
    last_message = ""
    ready_since: float | None = None
    while time.monotonic() < deadline:
        result = read_result()
        status = str(result.get("status", ""))
        if status != last_status:
            print(f"[E2E] companion status: {status}")
            last_status = status
        last_message = str(result.get("message", ""))
        result_is_fresh = (
            RESULT_PATH.is_file()
            and RESULT_PATH.stat().st_mtime >= uploaded_at
        )
        generation=RUNTIME_DIR/'restart.started'
        if generation.exists() and RESULT_PATH.is_file():
            result_is_fresh = result_is_fresh and RESULT_PATH.stat().st_mtime>=generation.stat().st_mtime
        allowed_statuses = (
            {"READY"}
            if require_dynamic_release
            else {"READY", "READY_RAW_REPLAY", "READY_FIXED_FALLBACK"}
        )
        ready = status in allowed_statuses and result_is_fresh
        restart_pending = (RUNTIME_DIR / "restart.request").exists()
        if ready and not restart_pending:
            if ready_since is None:
                ready_since = time.monotonic()
            # A raw upload can request a HOME/world rebuild immediately after
            # validation. Keep the ground station connected until the fresh
            # companion has revalidated the EEPROM mission after that restart.
            if time.monotonic() - ready_since >= 8.0:
                if status == "READY_RAW_REPLAY":
                    print("[E2E] uploaded Mission Planner route accepted; waiting for dynamic-route validation")
                return
        else:
            ready_since = None
        time.sleep(0.5)
    raise TimeoutError(
        "companion did not validate the uploaded mission"
        + (f": {last_message}" if last_message else "")
    )


def wait_mode_and_arm(connection, timeout: float = 60.0) -> None:
    deadline = time.monotonic() + timeout
    auto_mode = connection.mode_mapping()["AUTO"]
    last_mode_request = 0.0
    last_arm_request = 0.0
    while time.monotonic() < deadline:
        now = time.monotonic()
        # ArduPlane can temporarily leave AUTO while EKF/GPS finishes its
        # initialisation. Keep requesting AUTO until it is both accepted and
        # armed; a mode request made too early must not be treated as final.
        if now - last_mode_request >= 1.0:
            connection.mav.mission_set_current_send(
                connection.target_system,
                connection.target_component,
                0,
            )
            connection.mav.set_mode_send(
                connection.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                auto_mode,
            )
            last_mode_request = now
        if now - last_arm_request >= 1.0:
            connection.mav.command_long_send(
                connection.target_system,
                connection.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1,
                0,
                0,
                0,
                0,
                0,
                0,
            )
            last_arm_request = now
        message = connection.recv_match(
            type=["HEARTBEAT", "STATUSTEXT", "COMMAND_ACK"],
            blocking=True,
            timeout=1.0,
        )
        if message is None:
            continue
        if message.get_type() == "STATUSTEXT":
            print(f"[E2E] ArduPlane: {message.text}")
            continue
        if message.get_type() == "COMMAND_ACK":
            print(
                f"[E2E] command ACK command={message.command} "
                f"result={message.result}"
            )
            continue
        heartbeat = message
        armed = bool(
            heartbeat.base_mode
            & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        )
        mode = mavutil.mode_string_v10(heartbeat)
        print(f"[E2E] mode={mode} armed={armed}")
        if mode != "AUTO":
            continue
        if armed:
            return
    raise TimeoutError("ArduPlane did not enter armed AUTO")


def wait_outcome(
    full: bool,
    timeout: float,
    require_dynamic_release: bool = False,
) -> None:
    deadline = time.monotonic() + timeout
    last_status = ""
    while time.monotonic() < deadline:
        result = read_result()
        status = str(result.get("status", ""))
        if status != last_status:
            print(f"[E2E] result status: {status}")
            last_status = status
        if status == "LAUNCH_FAILED":
            raise RuntimeError(str(result.get("message", "physical launch failed")))
        if not full and status in {
            "FLYING",
            "RESCAN",
            "ATTACK_ROUTE_ACTIVE",
            "PAYLOAD_RELEASED",
            "IMPACT_RECORDED",
            "COMPLETED",
        }:
            print("[E2E] physical launch confirmed")
            return
        if full and status == "COMPLETED":
            impact = result.get("impact")
            release = result.get("release")
            if not isinstance(impact, dict) or not isinstance(release, dict):
                raise RuntimeError("mission completed without release / impact records")
            if result.get("replay_completed") is not True:
                raise RuntimeError("mission completed without a replay-complete record")
            if (
                require_dynamic_release
                and release.get("source") != "DYNAMIC_BALLISTIC"
            ):
                raise RuntimeError(
                    "mission completed without primary dynamic ballistic release: "
                    f"source={release.get('source')}"
                )
            print(
                "[E2E] full mission completed: "
                f"replay={result.get('replay_completed')} "
                f"competition_rules={result.get('competition_rules_passed')} "
                f"competition_failures={result.get('competition_rule_failures', [])} "
                f"warnings={result.get('mission_warnings', [])}"
            )
            return
        time.sleep(0.5)
    target = "full mission completion" if full else "physical hand launch"
    raise TimeoutError(f"timed out waiting for {target}")


def main() -> int:
    sys.stdout.reconfigure(line_buffering=True)
    parser = argparse.ArgumentParser()
    parser.add_argument("--mission", type=Path)
    parser.add_argument("--task", type=int, choices=(1, 2))
    parser.add_argument("--group", choices=("A", "B"))
    parser.add_argument("--seed", type=int)
    parser.add_argument("--wind-speed", type=float)
    parser.add_argument("--wind-direction", type=float, default=90.0)
    parser.add_argument("--full", action="store_true")
    parser.add_argument('--record-video',type=Path,help='READY 后录制当前任务，避免跨场景代次拼接')
    parser.add_argument(
        "--require-dynamic-release",
        action="store_true",
        help="fail unless the primary online ballistic path commands release",
    )
    parser.add_argument("--timeout", type=float, default=300.0)
    parser.add_argument(
        "--arm-timeout",
        type=float,
        default=60.0,
        help="等待 ArduPlane 完成传感器初始化并进入 AUTO 解锁的秒数",
    )
    arguments = parser.parse_args()
    if arguments.seed is not None:
        (RUNTIME_DIR/'task_policy.json').write_text(json.dumps({
            'randomize_each_upload':False,'fixed_seed':arguments.seed})+'\n')

    connection = mavutil.mavlink_connection(
        "udpin:0.0.0.0:14550",
        # ArduPlane defaults SYSID_MYGCS to 255 and rejects mission writes
        # from other ground-station IDs unless explicitly configured. Match
        # Mission Planner so this smoke test exercises the supported path.
        source_system=255,
    )
    heartbeat = wait_autopilot_heartbeat(connection,timeout=20)
    if heartbeat is None:
        raise TimeoutError("no ArduPlane heartbeat on UDP 14550")
    print(
        f"[E2E] heartbeat system={connection.target_system} "
        f"component={connection.target_component}"
    )
    requested = {
        "SCR_USER1": arguments.task,
        "SCR_USER2": None if arguments.group is None else int(arguments.group == "B"),
        "SCR_USER3": arguments.seed,
        "SIM_WIND_SPD": arguments.wind_speed,
        "SIM_WIND_DIR": (
            arguments.wind_direction if arguments.wind_speed is not None else None
        ),
    }
    changes = {name: value for name, value in requested.items() if value is not None}
    if changes:
        expected = {}
        if arguments.task is not None:
            expected["task"] = arguments.task
        if arguments.group is not None:
            expected["group"] = arguments.group
        if arguments.seed is not None:
            expected["seed"] = arguments.seed
        if arguments.wind_speed is not None:
            expected["wind_speed_mps"] = float(arguments.wind_speed)
            expected["wind_from_deg"] = float(arguments.wind_direction) % 360.0
        try:
            current = json.loads(PARAMETERS_PATH.read_text(encoding="utf-8"))
        except (FileNotFoundError, json.JSONDecodeError):
            current = {}
        scenario_changed = any(
            current.get(key) != value for key, value in expected.items()
        )
        if scenario_changed:
            changed_at = time.time()
            for name, value in changes.items():
                set_parameter(connection, name, float(value))
            wait_scenario_restart(expected, changed_at)
            # Discard datagrams from the old SITL instance, then require a
            # fresh heartbeat before mission upload.
            while connection.recv_match(blocking=False) is not None:
                pass
            if wait_autopilot_heartbeat(connection,timeout=20) is None:
                raise TimeoutError("no heartbeat after scenario restart")
        else:
            print(f"[E2E] scenario already active: {expected}")

    group = arguments.group
    if group is None:
        try:
            group = json.loads(PARAMETERS_PATH.read_text(encoding="utf-8"))["group"]
        except (FileNotFoundError, KeyError, json.JSONDecodeError):
            group = "A"
    mission = arguments.mission or (
        SIM_CONFIG_DIR / "missions" / f"group_{str(group).lower()}.waypoints"
    )
    uploaded_at = time.time()
    for attempt in range(1, 4):
        try:
            count = upload_mission(connection, mission)
            break
        except (RuntimeError, TimeoutError) as exc:
            if attempt == 3:
                raise
            print(f"[E2E] transient upload failure ({exc}); retry {attempt + 1}/3")
            time.sleep(3.0)
    print(f"[E2E] uploaded {count} mission items")
    wait_ready(
        uploaded_at,
        require_dynamic_release=arguments.require_dynamic_release,
    )
    recorder=None
    succeeded=False
    try:
        if arguments.record_video:
            recorder=subprocess.Popen([sys.executable,str(Path(__file__).with_name('record_camera.py')),
                '--output',str(arguments.record_video),'--duration',str(arguments.timeout+60),
                '--result',str(RESULT_PATH)])
            time.sleep(.5)
        wait_mode_and_arm(connection, timeout=arguments.arm_timeout)
        wait_outcome(arguments.full,arguments.timeout,arguments.require_dynamic_release)
        succeeded=True
    finally:
        if recorder is not None:
            # COMPLETED may fail a stricter release assertion while the
            # recorder is already closing itself. Let it finish its report.
            if not succeeded and read_result().get('status') != 'COMPLETED' and recorder.poll() is None:
                recorder.send_signal(signal.SIGINT)
            try:recorder.wait(timeout=40)
            except subprocess.TimeoutExpired:
                recorder.send_signal(signal.SIGINT)
                recorder.wait(timeout=40)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
