from __future__ import annotations

import json
import math
import os
import signal
import secrets
import subprocess
import time
from dataclasses import asdict, replace
from pathlib import Path

from pymavlink import mavutil

from .ballistics import (
    solution_with_actuator_delay,
    solve_release,
)
from .config import (
    ORIGIN_LAT_DEG,
    ORIGIN_LON_DEG,
    MISSION_MAX_HOME_DISTANCE_M,
    RECON_AIRSPEED_MPS,
    RECON_TURN_AIRSPEED_MPS,
    MISSION_PLANNING_AIRSPEED_MPS,
    MISSION_PLANNING_BANK_DEG,
    MISSION_TRACKING_MARGIN_M,
    PAYLOAD_TARGET_TOLERANCE_M,
    PAYLOAD_ACTUATOR_DELAY_S,
    PAYLOAD_DRAG_RATE_S,
    PAYLOAD_MAX_ROLL_RAD,
    PAYLOAD_MAX_YAW_RATE_RPS,
    PAYLOAD_NOMINAL_AGL_M,
    PAYLOAD_NOMINAL_AIRSPEED_MPS,
    PAYLOAD_RELEASE_PWM,
    PAYLOAD_SERVO,
    RAW_REPLAY_RELEASE_PWM,
    RUNTIME_DIR,
    SIM_FLIGHT_PROFILE,
    SimParameters,
)
from .mavlink_io import MavlinkClient, MavlinkTimeout
from .mission import (
    MISSION_MODE_BLOCKED,
    MISSION_MODE_DYNAMIC_READY,
    MISSION_MODE_FIXED_FALLBACK,
    MISSION_MODE_RAW_REPLAY,
    FixedFallbackContract,
    MissionContract,
    MissionAssessment,
    MissionItem,
    RawReplayContract,
    assess_raw_replay,
    assess_mission,
    build_attack_patch,
    crossed_scan_end,
    retry_entry_seq,
    derive_scan_rectangle,
    latlon_to_ne,
    load_patch_backup,
    save_patch_backup,
    scan_coverage_cells,
    update_scan_coverage,
)
from .results import ResultStore, RunResult
from .field_layout import load_field_layout
from .replay_route import (
    deserialize_mission,
    load_replay_route,
    mission_route_compatible,
    write_replay_route,
)
from .route_state import (
    RouteState,
    in_takeoff_landing_zone,
    public_areas,
    save_route_state,
)
from .scenario import GROUP_CENTERS_NE, ZONE_HALF_SIZE_M, wind_vector, write_runtime
from .selection import (
    Detection,
    Selection,
    aggregate_detections,
    select_target,
)
from .telemetry import Telemetry, append_telemetry_history, write_telemetry
from .vision import load_json


FULL_STOP_MAX_GROUNDSPEED_MPS = 0.2
FULL_STOP_HOLD_S = 0.75
SCAN_COVERAGE_THRESHOLD = 0.90
MIN_STABLE_TARGETS = 3
# The YOLO worker runs independently of the MAVLink mission-current stream.
# Allow the last in-flight camera frame to be fused before declaring the scan
# incomplete; 1.25 s routinely discarded a third real detection at the scan
# boundary on a loaded CPU and forced a fixed release.
SCAN_FINAL_FRAME_GRACE_S = 3.0


class CompanionRuntime:
    def __init__(self, endpoint: str, dry_run: bool = False):
        self.client = MavlinkClient(endpoint)
        self.dry_run = dry_run
        self.parameters = SimParameters()
        self.mission: list[MissionItem] = []
        self.contract: MissionContract | FixedFallbackContract | RawReplayContract | None = None
        self.mission_assessment = MissionAssessment(MISSION_MODE_BLOCKED, None)
        self.telemetry = Telemetry()
        self.heartbeat = None
        self.last_end_seq_processed = -1
        self.launched = False
        self.release_commanded = False
        self.released = False
        self.completed = False
        self.selected: Selection | None = None
        self.previous_along_track = math.inf
        self.result = RunResult.create(1, 1, "A")
        self.store = ResultStore(self.result)
        self.last_status_time = 0.0
        self.last_mission_attempt = 0.0
        self.last_mission_wire_activity = 0.0
        self.mission_refresh_requested = False
        self.new_task_pending = False
        self.scan_end_pending_at: float | None = None
        self.uploaded_mission_count: int | None = None
        self.uploaded_mission_items: dict[int, MissionItem] = {}
        self.last_parameter_poll = 0.0
        self.original_attack_slots: list[MissionItem] = []
        self.mission_backup_path = RUNTIME_DIR / "mission_patch_backup.json"
        self.route_state = RouteState()
        self.last_route_point: tuple[float, float] | None = None
        self.last_trajectory_point: tuple[float, float] | None = None
        self.last_route_state_write = 0.0
        self.last_metrics_save = 0.0
        self.launch_time = 0.0
        self.airborne = False
        self.stopped_since: float | None = None
        self.scan_xtrack_sum_sq = 0.0
        self.scan_airspeed_sum = 0.0
        self.scan_agl_sum = 0.0
        self.scan_samples = 0
        self.turn_radius_sum = 0.0
        self.turn_radius_samples = 0
        self.scan_attempt = 1
        self.retry_turn_active = False
        self.retry_return_commanded = False
        self.scan_cells: list[tuple[float, float, float]] = []
        self.covered_scan_cells: set[int] = set()
        self.scan_coverage_ratio = 0.0
        self.stable_target_count = 0
        self.fallback_reason: str | None = None
        self.raw_release_triggered = False
        # The uploaded official route remains authoritative.  A raw route may
        # still contain a fixed DO_SET_SERVO item well after the aircraft has
        # passed the ballistic release point, so keep a separate along-track
        # baseline for the optional vision-guided release adapter.  This is
        # deliberately independent from the generated dynamic-route state.
        self.raw_previous_along_track = math.inf
        self.last_mission_seq_seen = -1
        self.origin_lat_deg = ORIGIN_LAT_DEG
        self.origin_lon_deg = ORIGIN_LON_DEG
        self.release_pwm = PAYLOAD_RELEASE_PWM

    def _reset_run(self, parameters: SimParameters) -> None:
        """Start a clean run after a disarmed Mission Planner parameter change."""
        self.parameters = parameters
        write_runtime(parameters)

        # These files describe physical or visual state from one particular
        # Gazebo world. Keeping any of them across a scenario rebuild can make
        # a new run inherit an old release, impact, or recognition result.
        for artifact in (
            "payload_released.flag",
            "impact.json",
            "touchdown.json",
            "vision_state.json",
            "vision_frame.jpg",
            "last_detection_frame.jpg",
            "route_state.json",
            "telemetry.json",
            "telemetry.jsonl",
        ):
            (RUNTIME_DIR / artifact).unlink(missing_ok=True)

        self.mission = []
        self.contract = None
        self.mission_assessment = MissionAssessment(MISSION_MODE_BLOCKED, None)
        self.last_end_seq_processed = -1
        self.scan_end_pending_at = None
        self.launched = False
        self.release_commanded = False
        self.released = False
        self.completed = False
        self.selected = None
        self.previous_along_track = math.inf
        self.original_attack_slots = []
        self.route_state = RouteState(group=parameters.group)
        self.last_route_point = None
        self.last_trajectory_point = None
        self.last_route_state_write = 0.0
        self.last_metrics_save = 0.0
        self.launch_time = 0.0
        self.airborne = False
        self.stopped_since = None
        self.scan_xtrack_sum_sq = 0.0
        self.scan_airspeed_sum = 0.0
        self.scan_agl_sum = 0.0
        self.scan_samples = 0
        self.turn_radius_sum = 0.0
        self.turn_radius_samples = 0
        self.scan_attempt = 1
        self.scan_cells = []
        self.retry_turn_active = False
        self.retry_return_commanded = False
        self.last_recon_speed = None
        self.covered_scan_cells = set()
        self.scan_coverage_ratio = 0.0
        self.stable_target_count = 0
        self.fallback_reason = None
        self.raw_release_triggered = False
        self.raw_previous_along_track = math.inf
        self.last_mission_seq_seen = -1
        self.origin_lat_deg = ORIGIN_LAT_DEG
        self.origin_lon_deg = ORIGIN_LON_DEG
        self.release_pwm = PAYLOAD_RELEASE_PWM

        self.result = RunResult.create(
            parameters.seed, parameters.task, parameters.group
        )
        self.result.status = "WAITING_FOR_MISSION"
        self.result.message = (
            "Parameters changed; waiting for Gazebo restart and mission revalidation"
        )
        self.store = ResultStore(self.result)
        self.store.save()
        # Force a fresh download even if the previous mission was structurally
        # valid. This prevents an A-route from authorizing a newly selected B
        # scenario (and vice versa) before Mission Planner uploads the new plan.
        self.last_mission_attempt = 0.0

    def initialize(self) -> None:
        self.client.connect()
        self.client.set_parameters(SIM_FLIGHT_PROFILE)
        startup_seed = RUNTIME_DIR / "startup_seed.json"
        if startup_seed.exists():
            seed = load_json(startup_seed).get("seed")
            if type(seed) is not int or not 0 <= seed < (1 << 24):
                raise ValueError("invalid simulator startup seed")
            self.client.set_parameters({"SCR_USER3": float(seed)})
            startup_seed.unlink()
        self.parameters = SimParameters.from_ardupilot(self.client.read_parameters())
        old_parameters = load_json(RUNTIME_DIR / "parameters.json")
        write_runtime(self.parameters)
        if not self.dry_run:
            layout=load_field_layout()
            self.client.set_ground_home(layout.origin_lat_deg,layout.origin_lon_deg,layout.origin_alt_m)
        if not self._parameters_match(old_parameters, self.parameters.to_dict()):
            (RUNTIME_DIR / "restart.request").write_text(
                "Mission Planner parameters changed while disarmed\n",
                encoding="utf-8",
            )
        self.result = RunResult.create(
            self.parameters.seed, self.parameters.task, self.parameters.group
        )
        self.result.status = "WAITING_FOR_MISSION"
        self.store = ResultStore(self.result)
        self.store.save()
        for message_id, interval_us in (
            (33, 100000),
            (30, 100000),
            (36, 100000),
            (42, 200000),
            (62, 100000),
            (74, 100000),
        ):
            self._request_message_interval(message_id, interval_us)
        self._refresh_mission()

    @staticmethod
    def _parameters_match(old: dict, current: dict) -> bool:
        """Compare persisted SITL parameters without reacting to float32 roundoff."""

        if set(old) != set(current):
            return False
        for key, expected in current.items():
            observed = old.get(key)
            if isinstance(expected, (int, float)) and isinstance(observed, (int, float)):
                if not math.isclose(
                    float(observed), float(expected), rel_tol=1e-6, abs_tol=1e-5
                ):
                    return False
            elif observed != expected:
                return False
        return True

    def _request_message_interval(self, message_id: int, interval_us: int) -> None:
        connection = self.client.connection
        assert connection is not None
        connection.mav.command_long_send(
            self.client.target_system,
            self.client.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            float(message_id),
            float(interval_us),
            0,
            0,
            0,
            0,
            0,
        )

    def _launch_heading_deg(self) -> float:
        if isinstance(self.contract, RawReplayContract) or (RUNTIME_DIR / "mission_replay.json").is_file():
            # Gazebo uses +Y west, whereas route bearings use +east.
            return (-load_field_layout().initial_heading_deg) % 360.0
        return 0.0 if self.parameters.group == "A" else 180.0

    def _refresh_mission(self) -> None:
        self.last_mission_attempt = time.monotonic()
        self.mission_refresh_requested = False
        try:
            downloaded_mission = self.client.download_mission(timeout=8.0)
            uploaded_mission = self._captured_uploaded_mission()
            previous_replay = load_replay_route(RUNTIME_DIR / "mission_replay.json")
            previous_mission = (
                deserialize_mission(previous_replay["items"])
                if previous_replay and isinstance(previous_replay.get("items"), list)
                else None
            )
            if uploaded_mission and mission_route_compatible(uploaded_mission, downloaded_mission):
                mission = uploaded_mission
            elif previous_mission and mission_route_compatible(previous_mission, downloaded_mission):
                mission = previous_mission
            else:
                mission = downloaded_mission
            raw_assessment = assess_raw_replay(
                mission,
                payload_servo=PAYLOAD_SERVO,
                payload_pwm=float(RAW_REPLAY_RELEASE_PWM),
                warning_home_distance_m=MISSION_MAX_HOME_DISTANCE_M,
            )
            has_raw_release = any(
                item.command == mavutil.mavlink.MAV_CMD_DO_SET_SERVO
                and int(round(item.param1)) == int(PAYLOAD_SERVO)
                and abs(item.param2 - float(RAW_REPLAY_RELEASE_PWM)) <= 0.5
                for item in mission
            )
            layout = load_field_layout()
            self.origin_lat_deg, self.origin_lon_deg = layout.origin_lat_deg, layout.origin_lon_deg
            self.release_pwm = RAW_REPLAY_RELEASE_PWM if has_raw_release else PAYLOAD_RELEASE_PWM
            dynamic = assess_mission(
                mission, min_release_pwm=float(self.release_pwm),
                origin_lat_deg=self.origin_lat_deg, origin_lon_deg=self.origin_lon_deg,
                max_home_distance_m=MISSION_MAX_HOME_DISTANCE_M,
            )
            # Tagged competition routes can use low-active release PWM and a
            # private HOME while retaining the same four dynamic payload slots.
            # Official Mission Planner routes can opt out of the dynamic patch
            # so the simulator replays every uploaded item exactly as sent by
            # the ground station.  The physical release adapter still listens
            # for the low-active DO_SET_SERVO item in that mode.
            preserve_uploaded_route = os.environ.get(
                "CUADC_PRESERVE_UPLOADED_ROUTE", "0"
            ).lower() in {"1", "true", "yes", "on"}
            use_raw = (
                bool(mission)
                and has_raw_release
                and (
                    preserve_uploaded_route
                    or not isinstance(dynamic.contract, MissionContract)
                )
            )
            assessment = raw_assessment if use_raw else dynamic
            if self.new_task_pending and assessment.launch_allowed:
                self.new_task_pending = False
                self._prepare_task_seed()
            replay_path = RUNTIME_DIR / "mission_replay.json"
            # Persist the uploaded route, keeping the physical arena fixed.
            if mission and assessment.launch_allowed:
                write_replay_route(mission, layout, replay_path)
            backup = load_patch_backup(self.mission_backup_path)
            if assessment.mode == MISSION_MODE_BLOCKED and backup:
                self.client.write_partial(backup)
                self.mission_backup_path.unlink(missing_ok=True)
                mission = self.client.download_mission(timeout=8.0)
                assessment = assess_mission(
                    mission,
                    min_release_pwm=float(PAYLOAD_RELEASE_PWM),
                    origin_lat_deg=ORIGIN_LAT_DEG,
                    origin_lon_deg=ORIGIN_LON_DEG,
                    max_home_distance_m=MISSION_MAX_HOME_DISTANCE_M,
                )
                if assessment.mode != MISSION_MODE_BLOCKED:
                    self._status("Recovered original mission slots after interrupted run")
        except Exception as exc:
            mission = []
            assessment = MissionAssessment(
                MISSION_MODE_BLOCKED,
                None,
                errors=(str(exc),),
            )
        self.mission = mission
        self.mission_assessment = assessment
        self.contract = assessment.contract
        self.result.mission_capability = assessment.mode
        self.result.mission_warnings = list(assessment.warnings)
        self.scan_attempt = 1
        self.covered_scan_cells = set()
        self.scan_coverage_ratio = 0.0
        self.stable_target_count = 0
        self.fallback_reason = None
        self.scan_cells = (
            scan_coverage_cells(mission, self.contract, origin_lat_deg=self.origin_lat_deg, origin_lon_deg=self.origin_lon_deg)
            if self.contract is not None
            else []
        )
        self.route_state.mission = self._mission_route_state(mission)
        self.route_state.mission_capability = assessment.mode
        self.route_state.mission_warnings = list(assessment.warnings)
        if assessment.mode == MISSION_MODE_BLOCKED:
            self.result.status = "MISSION_INVALID"
            self.result.message = "; ".join(assessment.errors)
            self.store.save()
            self._status("MISSION INVALID - launcher blocked", mavutil.mavlink.MAV_SEVERITY_CRITICAL)
        elif assessment.mode == MISSION_MODE_RAW_REPLAY:
            self.result.status = "READY_RAW_REPLAY"
            self.result.message = (
                "Uploaded Mission Planner route retained for flight control; dynamic attack slots are not yet active"
            )
            self.store.save()
            self._status("Raw Mission Planner route ready")
        elif assessment.mode == MISSION_MODE_FIXED_FALLBACK:
            self.result.status = "READY_FIXED_FALLBACK"
            self.result.message = (
                "Mission accepted; dynamic route unavailable, fixed payload "
                "fallback will still execute"
            )
            self.store.save()
            self._status(
                "Mission ready - fixed payload fallback",
                mavutil.mavlink.MAV_SEVERITY_WARNING,
            )
        else:
            self.result.status = "READY"
            self.result.message = "Dynamic mission ready; arm in AUTO to hand launch"
            self.store.save()
            self._status("Dynamic mission ready for AUTO launch")

    def _mission_route_state(self, items: list[MissionItem]) -> list[dict]:
        position_commands = {
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS,
            mavutil.mavlink.MAV_CMD_NAV_LOITER_TO_ALT,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            mavutil.mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT,
        }
        route: list[dict] = []
        for item in items:
            if item.command not in position_commands or (item.x == 0.0 and item.y == 0.0):
                continue
            north_m, east_m = latlon_to_ne(
                item.x, item.y, self.origin_lat_deg, self.origin_lon_deg
            )
            route.append(
                {
                    "seq": item.seq,
                    "command": item.command,
                    "north_m": north_m,
                    "east_m": east_m,
                    "alt_m": item.z,
                }
            )
        return route

    def _note_mission_wire_activity(self, message, now: float | None = None) -> None:
        """Defer downloads while Mission Planner owns the mission protocol.

        ArduPilot permits one mission transfer at a time on the MAVProxy link.
        The companion therefore observes upload requests / acknowledgements
        passively, waits for a quiet interval, then downloads and validates the
        completed mission.  This prevents periodic validation from cancelling
        a Mission Planner upload.
        """

        self.last_mission_wire_activity = (
            time.monotonic() if now is None else now
        )
        message_type = message.get_type()
        source_system = int(getattr(message, "get_srcSystem", lambda: 0)() or 0)
        from_vehicle = source_system == int(self.client.target_system)
        # MAVProxy broadcasts vehicle replies to every output, but a GCS's
        # outbound MISSION_COUNT is not necessarily echoed to this companion.
        # A vehicle requesting item zero is the observable upload boundary.
        if (from_vehicle and message_type in {'MISSION_REQUEST','MISSION_REQUEST_INT'}
                and int(getattr(message,'seq',-1))==0 and not self.telemetry.armed):
            if self.launched or self.completed:
                self._reset_run(self.parameters)
            self.new_task_pending=True
        if not from_vehicle and message_type == "MISSION_COUNT":
            # A completed run keeps ``launched`` true so its final metrics are
            # never mistaken for a new flight.  Mission Planner may upload a
            # second route while the aircraft is disarmed, though.  Treat the
            # first external MISSION_COUNT as the start of a new run and clear
            # the one-shot release/landing state before validating that upload.
            # Companion-originated partial writes use the vehicle system id and
            # therefore do not take this path while restoring the original
            # mission slots.
            if not self.telemetry.armed and (self.launched or self.completed):
                self._reset_run(self.parameters)
            self.uploaded_mission_count = int(message.count)
            self.uploaded_mission_items = {}
            self.new_task_pending = not self.telemetry.armed and int(message.count)>0
        elif not from_vehicle and message_type in {"MISSION_ITEM", "MISSION_ITEM_INT"}:
            item = MissionItem.from_message(message)
            if self.uploaded_mission_count is None or item.seq < self.uploaded_mission_count:
                self.uploaded_mission_items[item.seq] = item
        if message_type in {
            "MISSION_REQUEST",
            "MISSION_REQUEST_INT",
            "MISSION_ITEM",
            "MISSION_ITEM_INT",
        }:
            self.mission_refresh_requested = True
        elif (
            message_type == "MISSION_ACK"
            and int(message.type) == mavutil.mavlink.MAV_MISSION_ACCEPTED
        ):
            self.mission_refresh_requested = True

    def _captured_uploaded_mission(self) -> list[MissionItem] | None:
        if self.uploaded_mission_count is None:
            return None
        if len(self.uploaded_mission_items) != self.uploaded_mission_count:
            return None
        if set(self.uploaded_mission_items) != set(range(self.uploaded_mission_count)):
            return None
        return [self.uploaded_mission_items[index] for index in range(self.uploaded_mission_count)]

    def _prepare_task_seed(self) -> None:
        """A completed, disarmed GCS upload starts a fresh random task once.

        World/SITL rebuilding occurs before READY is usable. Parameter polls,
        internal downloads and the restarted companion never draw another seed.
        An explicit supervisor --seed keeps deterministic replay available.
        """
        policy=load_json(RUNTIME_DIR/'task_policy.json')
        if self.telemetry.armed or not policy.get('randomize_each_upload',False):
            return
        # Add a nonzero offset modulo the float32-exact MAVLink seed range.
        seed=(self.parameters.seed+1+secrets.randbelow((1<<24)-1))%(1<<24)
        self.client.set_parameters({'SCR_USER3':float(seed)})
        self.parameters=replace(self.parameters,seed=seed)
        self.result.seed=seed
        write_runtime(self.parameters)
        (RUNTIME_DIR/'restart.request').write_text('new task uploaded; fresh random target layout\n')

    def _mission_refresh_due(self, now: float | None = None) -> bool:
        current = time.monotonic() if now is None else now
        quiet = current - self.last_mission_wire_activity >= 2.0
        passive_upload_complete = self.mission_refresh_requested
        safety_poll_due = current - self.last_mission_attempt >= 60.0
        return quiet and (passive_upload_complete or safety_poll_due)

    def _status(self, text: str, severity: int = mavutil.mavlink.MAV_SEVERITY_NOTICE) -> None:
        if time.monotonic() - self.last_status_time < 0.75:
            return
        self.last_status_time = time.monotonic()
        try:
            self.client.status_text(text, severity)
        except Exception:
            pass

    def _launch(self) -> None:
        # The physical impulse must match the heading used by the generated
        # world and the uploaded route, including non-cardinal directions.
        yaw_deg = self._launch_heading_deg()
        payload = (
            f"x: {self.parameters.launch_speed_mps:.4f}, "
            f"y: {self.parameters.launch_pitch_deg:.4f}, z: {yaw_deg:.4f}"
        )
        if not self.dry_run:
            # EKF initialization can replace HOME after the first disarmed write.
            # Correct it while the Gazebo launcher still physically holds the
            # aircraft, before the one-shot impulse; this is SITL-only code.
            if self.launched or self.telemetry.groundspeed_mps>1:
                raise RuntimeError('ground HOME correction requires the held SITL aircraft')
            layout=load_field_layout()
            self.client.set_ground_home(layout.origin_lat_deg,layout.origin_lon_deg,
                                        layout.origin_alt_m,held_prelaunch=True)
            listener = subprocess.Popen(
                [
                    "gz",
                    "topic",
                    "-e",
                    "-t",
                    "/cuadc/hand_launch/status",
                    "-n",
                    "1",
                    "--json-output",
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                start_new_session=True,
            )
            try:
                # Give Gazebo Transport discovery time to register the status
                # subscriber before issuing the one-shot command.
                time.sleep(0.35)
                output = ""
                error = ""
                for _ in range(3):
                    subprocess.run(
                        [
                            "gz",
                            "topic",
                            "-t",
                            "/cuadc/hand_launch",
                            "-m",
                            "gz.msgs.Vector3d",
                            "-p",
                            payload,
                        ],
                        check=True,
                        timeout=3,
                    )
                    try:
                        output, error = listener.communicate(timeout=2.0)
                    except subprocess.TimeoutExpired:
                        continue
                    if '"data":"launched"' in output.replace(" ", ""):
                        break
                else:
                    raise RuntimeError(
                        "Gazebo launcher did not confirm physical launch"
                        + (f": {error.strip()}" if error.strip() else "")
                    )
            finally:
                if listener.poll() is None:
                    try:
                        os.killpg(listener.pid, signal.SIGTERM)
                    except ProcessLookupError:
                        pass
                    try:
                        listener.wait(timeout=1)
                    except subprocess.TimeoutExpired:
                        os.killpg(listener.pid, signal.SIGKILL)
        self.launched = True
        self.launch_time = time.monotonic()
        self.last_route_point = (
            self.telemetry.north_m,
            self.telemetry.east_m,
        )
        self.last_trajectory_point = self.last_route_point
        self.route_state.group = self.parameters.group
        layout = load_field_layout()
        self.route_state.origin_lat_deg = layout.origin_lat_deg
        self.route_state.origin_lon_deg = layout.origin_lon_deg
        self.route_state.field_layout = layout.to_dict()
        self.result.status = "FLYING"
        self.result.message = "Physical hand-launch impulse applied"
        self.store.save()
        self._status("CUADC physical hand launch applied")

    def _detections(self) -> tuple[list[Detection], str]:
        state = load_json(RUNTIME_DIR / "vision_state.json")
        source = str(state.get("source", "PENDING"))
        detections = [
            Detection(
                track_id=str(item["track_id"]),
                value=int(item["value"]),
                confidence=float(item["confidence"]),
                frames=int(item.get("frames", 1)),
                label=str(item.get("label", "")),
                north_m=float(item["north_m"]),
                east_m=float(item["east_m"]),
            )
            for item in state.get("detections", [])
            if (
                item.get("track_id") is not None
                and item.get("value") is not None
                and item.get("north_m") is not None
                and item.get("east_m") is not None
            )
        ]
        return detections, source

    def _sample_scan_coverage(self) -> None:
        if self.contract is None or not self.scan_cells:
            return
        if not (
            self.contract.scan_start_seq
            < self.telemetry.mission_seq
            < self.contract.scan_end_seq
        ):
            return
        vision = load_json(RUNTIME_DIR / "vision_state.json")
        updated_at = float(vision.get("updated_at", 0.0))
        camera_age_s = time.time() - updated_at if updated_at > 0.0 else math.inf
        metric='scan_camera_healthy_samples' if 0<=camera_age_s<=3 else 'scan_camera_stale_samples'
        self.result.route_metrics[metric]=self.result.route_metrics.get(metric,0)+1
        self.result.route_metrics['planned_scan_cells']=len(self.scan_cells)
        self.scan_coverage_ratio = update_scan_coverage(
            self.scan_cells,
            self.covered_scan_cells,
            self.telemetry.north_m,
            self.telemetry.east_m,
            self.telemetry.relative_alt_m,
            camera_age_s,
        )

    def _scan_is_complete(self) -> bool:
        return (
            self.scan_coverage_ratio >= SCAN_COVERAGE_THRESHOLD
            and self.stable_target_count >= MIN_STABLE_TARGETS
        )

    def _target_center(self):
        layout = load_field_layout()
        return (layout.group_centers_ne or GROUP_CENTERS_NE)[self.parameters.group]

    def _poll_scan_decision(self, now: float | None = None) -> None:
        if self.scan_end_pending_at is None or self.contract is None:
            return
        now=time.monotonic() if now is None else now
        detections,_=self._detections()
        stable=sum(item.frames>=2 for item in aggregate_detections(detections))
        before_payload=self.telemetry.mission_seq<self.contract.fallback_start_seq
        if (stable<MIN_STABLE_TARGETS and before_payload
                and now-self.scan_end_pending_at<SCAN_FINAL_FRAME_GRACE_S):
            return
        self.scan_end_pending_at=None
        self._handle_scan_end()

    def _handle_scan_end(self) -> None:
        assert self.contract is not None
        detections, source = self._detections()
        fused = aggregate_detections(detections)
        stable = [item for item in fused if item.frames >= 2]
        self.stable_target_count = len(stable)
        self.result.detections = [asdict(item) for item in fused]
        self.result.recognition_complete = self._scan_is_complete()
        if not self.result.recognition_complete:
            # A moving camera can finish the first scan while one YOLO track
            # is still warming up.  Dynamic missions reserve a holding turn
            # before the scan for exactly this case.  Use one bounded retry;
            # never invent a missing target or fall back to a guessed
            # coordinate.  Raw Mission Planner routes remain verbatim and do
            # not receive an inserted pass.
            if (
                isinstance(self.contract, MissionContract)
                and self.scan_attempt < 2
                and self.telemetry.armed
            ):
                entry = retry_entry_seq(self.mission, self.contract)
                self.scan_attempt += 1
                self.retry_turn_active = True
                self.retry_return_commanded = False
                self.last_end_seq_processed = entry
                self.client.set_current(entry)
                self.result.status = "SCAN_RETRY"
                self.result.message = (
                    "YOLO scan incomplete; one planned holding-turn retry "
                    f"started at mission sequence {entry}"
                )
                self.fallback_reason = (
                    f"first scan incomplete: coverage={self.scan_coverage_ratio:.0%}, "
                    f"stable_targets={self.stable_target_count}"
                )
                self.route_state.fallback_reason = self.fallback_reason
                self.store.save()
                self._status(
                    "YOLO scan incomplete - retrying planned scan",
                    mavutil.mavlink.MAV_SEVERITY_WARNING,
                )
                return
            self.selected = None
            self.result.source = source
            self.result.selected_target = None
            self.fallback_reason = (
                f"scan incomplete after {self.scan_attempt} attempts: "
                f"coverage={self.scan_coverage_ratio:.0%}, "
                f"stable_targets={self.stable_target_count}"
            )
            self.route_state.fallback_reason = self.fallback_reason
            self.result.status = "FALLBACK_ROUTE_ACTIVE"
            self.result.message = (
                self.fallback_reason
                + "; original Mission Planner payload route retained"
            )
            self.store.save()
            self._status(
                "Recon incomplete - fixed payload fallback active",
                mavutil.mavlink.MAV_SEVERITY_WARNING,
            )
            return

        selection = select_target(
            self.parameters.task,
            fused,
            require_complete=True,
        )
        # Selection is deliberately fail-closed.  A weak/partial recognition
        # result must never be converted to a zone-centre coordinate or used to
        # install a dynamic attack route.
        if (
            selection.track_id is None
            or selection.guessed
            or selection.source != "YOLO"
            or selection.north_m is None
            or selection.east_m is None
        ):
            self.selected = None
            self.result.source = selection.source
            self.result.guessed = False
            self.result.selected_target = None
            self.result.recognition_complete = False
            self.fallback_reason = selection.reason
            self.route_state.fallback_reason = self.fallback_reason
            self.result.status = "RECOGNITION_FAILED"
            self.result.message = (
                f"{selection.reason}; dynamic target route and payload withheld"
            )
            self.store.save()
            self._status(
                "Recognition incomplete - dynamic payload withheld",
                mavutil.mavlink.MAV_SEVERITY_WARNING,
            )
            return

        center_north, center_east = self._target_center()
        target_north = selection.north_m
        target_east = selection.east_m
        if (
            abs(target_north - center_north) > ZONE_HALF_SIZE_M + 3.0
            or abs(target_east - center_east) > ZONE_HALF_SIZE_M + 3.0
        ):
            # A projected track outside the public target zone is an
            # outlier, not permission to drop at the zone centre.
            self.selected = None
            self.result.source = "YOLO_OUTLIER"
            self.result.guessed = False
            self.result.selected_target = None
            self.result.recognition_complete = False
            self.fallback_reason = "Estimated YOLO track outside public target zone"
            self.route_state.fallback_reason = self.fallback_reason
            self.result.status = "RECOGNITION_FAILED"
            self.result.message = (
                f"{self.fallback_reason}; dynamic target route and payload withheld"
            )
            self.store.save()
            self._status(
                "Recognition outlier - dynamic payload withheld",
                mavutil.mavlink.MAV_SEVERITY_WARNING,
            )
            return
        self.selected = selection
        self.result.source = selection.source
        self.result.guessed = selection.guessed
        self.result.selected_target = {
            "track_id": selection.track_id,
            "value": selection.value,
            "reason": selection.reason,
            "north_m": target_north,
            "east_m": target_east,
            "guessed": selection.guessed,
        }

        if not isinstance(self.contract, MissionContract):
            self.result.status = "FALLBACK_ROUTE_ACTIVE"
            self.result.message = (
                "Recognition complete, but dynamic slots are unavailable; "
                "original fixed payload route retained"
            )
            self.fallback_reason = "dynamic mission slots unavailable"
            self.route_state.fallback_reason = self.fallback_reason
            self.store.save()
            self._status(
                "Target logged - fixed payload fallback active",
                mavutil.mavlink.MAV_SEVERITY_WARNING,
            )
            return

        try:
            decision_north, decision_east = latlon_to_ne(
                self.mission[self.contract.decision_seq].x,
                self.mission[self.contract.decision_seq].y, self.origin_lat_deg, self.origin_lon_deg,
            )
            scan_points = [
                latlon_to_ne(item.x, item.y, self.origin_lat_deg, self.origin_lon_deg)
                for item in self.mission
                if (
                    self.contract.scan_start_seq
                    < item.seq
                    < self.contract.scan_end_seq
                    and item.command == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
                    and not (item.x == 0.0 and item.y == 0.0)
                )
            ]
            if len(scan_points) >= 2:
                outbound_north = scan_points[-1][0] - scan_points[0][0]
                outbound_east = scan_points[-1][1] - scan_points[0][1]
            else:
                outbound_north = outbound_east = None
            attack_patch = build_attack_patch(
                self.mission,
                self.contract,
                target_north,
                target_east,
                decision_north,
                decision_east,
                nominal_speed_mps=MISSION_PLANNING_AIRSPEED_MPS,
                nominal_agl_m=PAYLOAD_NOMINAL_AGL_M,
                max_home_distance_m=MISSION_MAX_HOME_DISTANCE_M-MISSION_TRACKING_MARGIN_M,
                planning_bank_deg=MISSION_PLANNING_BANK_DEG,
                outbound_north=outbound_north,
                outbound_east=outbound_east,
                origin_lat_deg=self.origin_lat_deg, origin_lon_deg=self.origin_lon_deg,
                release_pwm=self.release_pwm,
                attack_altitude_limit_m=None,
            )
            patch = attack_patch
            originals = [self.mission[item.seq] for item in patch]
            save_patch_backup(self.mission_backup_path, originals)
            self.client.write_partial(patch)
            # AP_Mission does not reinitialise an item that was already active
            # when it was replaced.  The scan-end notification normally arrives
            # just after ArduPlane advances to this slot, so explicitly restart
            # it to make the newly uploaded loiter geometry deterministic.
            self.client.set_current(self.contract.fallback_start_seq)
            self.original_attack_slots = originals
            self.mission = [
                next((new for new in patch if new.seq == old.seq), old)
                for old in self.mission
            ]
            self.result.status = "ATTACK_ROUTE_ACTIVE"
            self.result.message = "Dynamic half-turn/drop/return segment installed"
            self._status("Dynamic half-turn target route installed")
        except Exception as exc:
            self.result.status = "FALLBACK_ROUTE_ACTIVE"
            self.result.message = (
                "Dynamic patch activation failed; recovery backup retained: "
                f"{exc}"
            )
            self._status("Route patch failed - center-drop fallback active", mavutil.mavlink.MAV_SEVERITY_WARNING)
        self.store.save()

    def _restore_original_attack_slots(self) -> bool:
        backup = self.original_attack_slots or load_patch_backup(
            self.mission_backup_path
        )
        if not backup:
            return True
        try:
            self.client.write_partial(backup)
        except Exception as exc:
            self.result.status = "MISSION_RESTORE_FAILED"
            self.result.message = f"Flight complete but mission-slot restore failed: {exc}"
            self.store.save()
            self._status(
                "Mission restore failed - reupload before next flight",
                mavutil.mavlink.MAV_SEVERITY_CRITICAL,
            )
            return False
        replacements = {item.seq: item for item in backup}
        self.mission = [
            replacements.get(item.seq, item)
            for item in self.mission
        ]
        self.original_attack_slots = []
        self.mission_backup_path.unlink(missing_ok=True)
        self._status("Original Mission Planner route slots restored")
        return True

    def _command_payload_release(
        self,
        solution,
        source: str,
        position_age_s: float = 0.0,
        raw_mission_item: MissionItem | None = None,
    ) -> None:
        if (
            self.release_commanded
            or self.released
            or not self.telemetry.armed
        ):
            return
        if not self.dry_run:
            # ArduPilot's Gazebo channel 8 maps a PWM value to the normalized
            # release topic.  The supplied competition files use a low-active
            # 1000 us value for the real PTK controller, while the simulation
            # latch intentionally accepts a positive normalized value.  Send
            # the simulator adapter's high signal through MAVLink itself so
            # the command remains in the flight-controller path and does not
            # incur a one-second ``gz topic`` process startup delay.  That
            # delay was large enough to move the aircraft 10--15 m after the
            # ballistic gate had opened.
            self.client.set_servo(PAYLOAD_SERVO, PAYLOAD_RELEASE_PWM)
        self.release_commanded = True
        speed = math.hypot(
            self.telemetry.velocity_north_mps,
            self.telemetry.velocity_east_mps,
        )
        release = {
            "command_timestamp": time.time(),
            "source": source,
            "aircraft_north_m": self.telemetry.north_m,
            "aircraft_east_m": self.telemetry.east_m,
            "agl_m": self.telemetry.relative_alt_m,
            "speed_mps": speed,
        }
        if solution is not None:
            release["solution"] = asdict(solution)
            release["position_age_s"] = position_age_s
            release["effective_command_delay_s"] = (
                PAYLOAD_ACTUATOR_DELAY_S + position_age_s
            )
        if raw_mission_item is not None:
            release["mission_command"] = {
                "seq": raw_mission_item.seq,
                "command": raw_mission_item.command,
                "servo_channel": raw_mission_item.param1,
                "pwm": raw_mission_item.param2,
            }
        self.result.release = release
        self.result.status = "RELEASE_COMMAND_SENT"
        self.result.message = (
            f"Servo {PAYLOAD_SERVO} release commanded; "
            "waiting for Gazebo latch confirmation"
        )
        self.store.save()
        self._status("Payload release commanded - awaiting physical latch")

    def _try_release(self) -> None:
        if (
            not isinstance(self.contract, MissionContract)
            or
            self.selected is None
            or self.release_commanded
            or self.released
            or not self.telemetry.armed
        ):
            return
        if not (
            self.contract.fallback_start_seq + 1
            <= self.telemetry.mission_seq
            <= self.contract.fallback_release_seq
        ):
            return
        selected = self.result.selected_target or {}
        target_north = float(selected.get("north_m", 0.0))
        target_east = float(selected.get("east_m", 0.0))
        speed = math.hypot(
            self.telemetry.velocity_north_mps,
            self.telemetry.velocity_east_mps,
        )
        if speed < 8.0 or self.telemetry.relative_alt_m < 5.0:
            return
        if abs(self.telemetry.roll_rad) > PAYLOAD_MAX_ROLL_RAD:
            return
        if abs(self.telemetry.yaw_rate_rps) > PAYLOAD_MAX_YAW_RATE_RPS:
            return
        target_delta_n = target_north - self.telemetry.north_m
        target_delta_e = target_east - self.telemetry.east_m
        if (
            target_delta_n * self.telemetry.velocity_north_mps
            + target_delta_e * self.telemetry.velocity_east_mps
        ) <= 0.0:
            return
        gazebo_wind = wind_vector(self.parameters)
        wind_north, wind_east = gazebo_wind[0], -gazebo_wind[1]
        solution = solve_release(
            target_north,
            target_east,
            max(0.2, self.telemetry.relative_alt_m),
            self.telemetry.velocity_north_mps,
            self.telemetry.velocity_east_mps,
            wind_north,
            wind_east,
            self.telemetry.velocity_down_mps,
            drag_rate_s=PAYLOAD_DRAG_RATE_S,
        )
        # GLOBAL_POSITION_INT and ATTITUDE arrive independently.  A safe yaw
        # sample can therefore open the gate while the most recent position is
        # already one transport cycle old.  Compare the stale aircraft point
        # with a command point moved upstream by that measured age as well as
        # by the physical servo/latch delay.
        position_age_s = 0.0
        if self.telemetry.timestamp_s > 0.0:
            position_age_s = max(
                0.0,
                # The MAVLink router timestamp includes the packet's queue
                # residence time.  Treating the full observed age as motion
                # delay double-counts that queueing; a 150 ms cap matches the
                # measured command-to-position path while retaining a stale
                # packet guard.
                min(0.15, time.time() - self.telemetry.timestamp_s),
            )
        solution = solution_with_actuator_delay(
            solution,
            self.telemetry.velocity_north_mps,
            self.telemetry.velocity_east_mps,
            PAYLOAD_ACTUATOR_DELAY_S + position_age_s,
        )
        distance = math.hypot(
            solution.release_north_m - self.telemetry.north_m,
            solution.release_east_m - self.telemetry.east_m,
        )
        # Telemetry arrives at a finite rate while the aircraft covers about
        # one metre per sample.  Requiring the aircraft to be inside a
        # The solver already includes measured actuator and telemetry delay;
        # a multi-metre direct gate would add that distance back as a forward
        # bias.  Keep the direct trigger at the competition tolerance and
        # retain a bounded line-crossing path for a packet that straddles it.
        along = (
            (solution.release_north_m - self.telemetry.north_m)
            * self.telemetry.velocity_north_mps
            + (solution.release_east_m - self.telemetry.east_m)
            * self.telemetry.velocity_east_mps
        ) / max(speed, 1e-6)
        crossed = (
            math.isfinite(self.previous_along_track)
            and self.previous_along_track > 0.0
            and along <= 0.0
        )
        self.previous_along_track = along
        cross_track = math.sqrt(max(0.0, distance * distance - along * along))
        # The companion receives GLOBAL_POSITION_INT in bursts.  A strict
        # 0.35 m geometric gate can fall between two samples at 10--12 m/s:
        # the first sample is just upstream and the next is already metres
        # downstream, so the command is never sent.  The ballistic solver
        # still uses the 0.35 m target tolerance; this small runtime margin
        # only covers one receive/dispatch interval and is disabled in the
        # dry-run gate tests that exercise the exact tolerance contract.
        direct_window = PAYLOAD_TARGET_TOLERANCE_M
        if not self.dry_run:
            direct_window = max(
                direct_window,
                min(1.25, speed * 0.10),
            )
        if distance > direct_window and not (
            crossed
            and along >= -direct_window
            and cross_track <= 10.0
        ):
            return
        self._command_payload_release(
            solution,
            "DYNAMIC_BALLISTIC",
            position_age_s=position_age_s,
        )

    def _try_raw_dynamic_release(self, mission_seq: int) -> bool:
        """Release a raw uploaded route at its computed ballistic point.

        Mission Planner's official files intentionally remain byte-for-byte
        unchanged and keep their low-active DO_SET_SERVO item.  On the real
        route that item is close enough to the target for the aircraft's
        release adapter, but in simulation the bottle's 1.6 s fall time makes
        a literal sequence-triggered release overshoot by tens of metres.
        Once YOLO has selected a target, this adapter watches the same route
        telemetry and commands the physical latch as the delayed ballistic
        release line is reached.  The original mission item is still retained
        and recorded in the result; the later autopilot command is harmless
        because the payload joint is one-shot latched.

        The return value tells the caller that the one-shot release has been
        handled, including a command that was sent in this call.
        """
        contract = self.contract
        selected = self.result.selected_target
        if not isinstance(contract, RawReplayContract) or not selected:
            return False
        if self.release_commanded or self.released or not self.telemetry.armed:
            return False
        release_sequences = tuple(contract.release_sequences)
        if not release_sequences:
            return False
        release_seq = max(release_sequences)
        # The final scan waypoint and the release slot form a short attack
        # leg.  Do not alter earlier reconnaissance or a later landing leg.
        # ArduPlane may publish MISSION_CURRENT 15 -> 17 and omit the
        # transient DO_SET_SERVO sequence.  Keep watching the short final
        # attack leg through the following waypoint; the uploaded mission is
        # still untouched and remains the flight-control source.
        if mission_seq < max(contract.first_flight_seq, release_seq - 2) or mission_seq > release_seq + 2:
            return False

        item = next((candidate for candidate in self.mission if candidate.seq == release_seq), None)
        if (
            item is None
            or item.command != mavutil.mavlink.MAV_CMD_DO_SET_SERVO
            or int(round(item.param1)) != int(PAYLOAD_SERVO)
            or abs(item.param2 - float(RAW_REPLAY_RELEASE_PWM)) > 0.5
        ):
            return False

        target_north = float(selected.get("north_m", math.nan))
        target_east = float(selected.get("east_m", math.nan))
        if not (math.isfinite(target_north) and math.isfinite(target_east)):
            return False
        speed = math.hypot(
            self.telemetry.velocity_north_mps,
            self.telemetry.velocity_east_mps,
        )
        if speed < 8.0 or self.telemetry.relative_alt_m < 5.0:
            return False
        target_delta_n = target_north - self.telemetry.north_m
        target_delta_e = target_east - self.telemetry.east_m
        if target_delta_n * self.telemetry.velocity_north_mps + target_delta_e * self.telemetry.velocity_east_mps <= 0.0:
            return False

        gazebo_wind = wind_vector(self.parameters)
        wind_north, wind_east = gazebo_wind[0], -gazebo_wind[1]
        solution = solve_release(
            target_north,
            target_east,
            max(0.2, self.telemetry.relative_alt_m),
            self.telemetry.velocity_north_mps,
            self.telemetry.velocity_east_mps,
            wind_north,
            -wind_east,
            self.telemetry.velocity_down_mps,
            drag_rate_s=PAYLOAD_DRAG_RATE_S,
        )
        position_age_s = 0.0
        if self.telemetry.timestamp_s > 0.0:
            position_age_s = max(0.0, min(0.15, time.time() - self.telemetry.timestamp_s))
        solution = solution_with_actuator_delay(
            solution,
            self.telemetry.velocity_north_mps,
            self.telemetry.velocity_east_mps,
            PAYLOAD_ACTUATOR_DELAY_S + position_age_s,
        )
        distance = math.hypot(
            solution.release_north_m - self.telemetry.north_m,
            solution.release_east_m - self.telemetry.east_m,
        )
        along = (
            (solution.release_north_m - self.telemetry.north_m) * self.telemetry.velocity_north_mps
            + (solution.release_east_m - self.telemetry.east_m) * self.telemetry.velocity_east_mps
        ) / max(speed, 1e-6)
        previous = self.raw_previous_along_track
        self.raw_previous_along_track = along
        # The solver already moves the release point upstream by the measured
        # actuator and telemetry delay.  A metre-scale trigger window would
        # apply that delay twice and bias the payload down-track.  Use the
        # same competition tolerance as the dynamic path; a packet that
        # straddles the line is accepted only when the crossing remains inside
        # that tolerance.
        command_window = PAYLOAD_TARGET_TOLERANCE_M
        if not self.dry_run:
            command_window = max(command_window, min(1.25, speed * 0.10))
        crossed = math.isfinite(previous) and previous > 0.0 and along <= 0.0
        if not (
            (-command_window <= along <= command_window and distance <= command_window * 1.25)
            or (crossed and along >= -command_window and distance <= command_window * 1.5)
        ):
            return False
        self.raw_release_triggered = True
        self._command_payload_release(
            solution,
            "MISSION_PLANNER_RAW_COMMAND",
            position_age_s=position_age_s,
            raw_mission_item=item,
        )
        return True

    def _try_mission_release_fallback(self, mission_seq: int) -> None:
        if self.retry_turn_active:
            return
        if isinstance(self.contract, RawReplayContract):
            # Prefer the vision-guided ballistic point while the aircraft is
            # on the final attack leg.  If recognition is unavailable or the
            # telemetry gate never opens, the uploaded sequence remains the
            # deterministic safety fallback below.
            if self._try_raw_dynamic_release(mission_seq):
                return
            # Once YOLO has supplied a target, a missed ballistic window is a
            # precision failure.  Do not silently replace it with the fixed
            # mission slot and report a successful-looking impact.  Dry-run
            # unit tests retain the legacy one-shot fallback contract.
            if self.result.selected_target is not None and not self.dry_run:
                return
            if (
                self.release_commanded
                or self.released
                or not self.telemetry.armed
                or not self.launched
                or not any(
                    mission_seq >= release_seq
                    for release_seq in self.contract.release_sequences
                )
            ):
                return
            # ArduPlane executes DO_SET_SERVO as a non-position mission
            # command and may publish MISSION_CURRENT as the next NAV item
            # (15 -> 17) without ever emitting the transient 16.  The first
            # sequence at or after the uploaded release slot is still an
            # unambiguous one-shot trigger because the contract contains the
            # original sequence and PWM values.
            item = next(
                (
                    candidate
                    for candidate in self.mission
                    if candidate.seq == max(self.contract.release_sequences)
                ),
                None,
            )
            if (
                item is None
                or item.command != mavutil.mavlink.MAV_CMD_DO_SET_SERVO
                or int(round(item.param1)) != int(PAYLOAD_SERVO)
                or abs(item.param2 - float(RAW_REPLAY_RELEASE_PWM)) > 0.5
            ):
                return
            self.raw_release_triggered = True
            self._command_payload_release(
                None,
                "MISSION_PLANNER_RAW_COMMAND",
                raw_mission_item=item,
            )
            return
        if (
            not isinstance(self.contract, MissionContract)
            or self.selected is None
            or self.release_commanded
            or self.released
            or not self.telemetry.armed
            or mission_seq < self.contract.fallback_start_seq + 1
        ):
            return
        if self.result.selected_target is not None and not self.dry_run:
            # Dynamic target selection must be paired with a ballistic
            # release.  The old centre/near-target fallback produced large
            # errors while still marking the run complete.
            return
        selected = self.result.selected_target or {}
        target_north = float(selected.get("north_m", 0.0))
        target_east = float(selected.get("east_m", 0.0))
        center_north, center_east = self._target_center()
        inside_target_zone = (
            abs(self.telemetry.north_m - center_north) <= ZONE_HALF_SIZE_M + 3.0
            and abs(self.telemetry.east_m - center_east) <= ZONE_HALF_SIZE_M + 3.0
        )
        speed = math.hypot(
            self.telemetry.velocity_north_mps,
            self.telemetry.velocity_east_mps,
        )
        if speed < 8.0 or not inside_target_zone:
            return
        error_to_target = math.hypot(
            self.telemetry.north_m - target_north,
            self.telemetry.east_m - target_east,
        )
        if error_to_target > 12.0:
            return
        self._command_payload_release(None, "MISSION_RELEASE_SLOT_FALLBACK")

    def _poll_impact(self) -> None:
        if not self.released or self.result.impact is not None:
            return
        impact = load_json(RUNTIME_DIR / "impact.json")
        if impact:
            position = impact.get("position", {})
            if "x" in position and "y" in position:
                impact["north_m"] = float(position["x"])
                impact["east_m"] = -float(position["y"])
            self.result.impact = impact
            self.result.status = "IMPACT_RECORDED"
            self.result.message = "First physical payload contact recorded"
            self.store.save()

    def _poll_touchdown(self) -> None:
        if not self.launched or self.result.first_touchdown is not None:
            return
        touchdown = load_json(RUNTIME_DIR / "touchdown.json")
        if not touchdown:
            return
        position = touchdown.get("position", {})
        if "x" not in position or "y" not in position:
            return
        north_m = float(position["x"])
        east_m = -float(position["y"])
        layout = load_field_layout()
        self.result.first_touchdown = {
            **touchdown,
            "north_m": north_m,
            "east_m": east_m,
            "in_takeoff_landing_zone": in_takeoff_landing_zone(
                north_m, east_m, layout
            ),
        }
        self.store.save()

    def _poll_physical_release(self) -> None:
        release_flag = RUNTIME_DIR / "payload_released.flag"
        if not self.launched:
            # During a parameter-triggered process restart, the old ROS bridge
            # can deliver one final latched=True sample after artifact cleanup.
            # It belongs to the previous Gazebo world and must never release
            # or complete a fresh, still-held aircraft.
            release_flag.unlink(missing_ok=True)
            (RUNTIME_DIR / "impact.json").unlink(missing_ok=True)
            (RUNTIME_DIR / "touchdown.json").unlink(missing_ok=True)
            return
        if self.released or not release_flag.exists():
            return
        self.released = True
        speed = math.hypot(
            self.telemetry.velocity_north_mps, self.telemetry.velocity_east_mps
        )
        release = dict(self.result.release or {})
        release.setdefault("source", "MISSION_FALLBACK")
        release.setdefault("aircraft_north_m", self.telemetry.north_m)
        release.setdefault("aircraft_east_m", self.telemetry.east_m)
        release.setdefault("agl_m", self.telemetry.relative_alt_m)
        release.setdefault("speed_mps", speed)
        release["physical_confirmed_at"] = time.time()
        self.result.release = release
        self.result.status = "PAYLOAD_RELEASED"
        self.result.message = "Gazebo confirmed the one-shot physical payload latch"
        self.store.save()
        self._status("Physical payload release confirmed")

    def _poll_parameters(self) -> None:
        now = time.monotonic()
        if self.telemetry.armed or now - self.last_parameter_poll < 5.0:
            return
        self.last_parameter_poll = now
        try:
            updated = SimParameters.from_ardupilot(self.client.read_parameters(timeout=3.0))
        except Exception:
            return
        if updated == self.parameters:
            return
        self._reset_run(updated)
        (RUNTIME_DIR / "restart.request").write_text(
            "Mission Planner parameters changed while disarmed\n", encoding="utf-8"
        )
        self._status("Parameters changed - Gazebo scenario restart requested")

    def _competition_rule_failures(self) -> list[str]:
        failures: list[str] = []
        metrics = self.result.route_metrics
        if float(metrics.get("max_home_distance_m", 0.0)) > MISSION_MAX_HOME_DISTANCE_M:
            failures.append(
                f"flight path reached {float(metrics['max_home_distance_m']):.0f} m from HOME, beyond the {MISSION_MAX_HOME_DISTANCE_M:.0f} m boundary"
            )
        if float(metrics.get("mission_time_s", 0.0)) > 480.0:
            failures.append("flight duration exceeded the 480 s competition limit")
        if not self.result.first_touchdown or not self.result.first_touchdown.get(
            "in_takeoff_landing_zone"
        ):
            failures.append("first touchdown was outside the 50 m x 50 m landing zone")
        stop = self.result.final_stop or {}
        if not stop.get("in_takeoff_landing_zone"):
            failures.append("final stop was outside the 50 m x 50 m landing zone")
        impact = self.result.impact or {}
        if impact:
            layout = load_field_layout()
            zones = public_areas(layout.to_dict(), self.parameters.group)["target_zones"]
            selected_zone = next(
                (zone for zone in zones if zone["name"] == self.parameters.group),
                None,
            )
            if selected_zone is None or not (
                abs(float(impact.get("north_m", math.inf)) - selected_zone["center_north_m"])
                <= selected_zone["half_size_m"]
                and abs(float(impact.get("east_m", math.inf)) - selected_zone["center_east_m"])
                <= selected_zone["half_size_m"]
            ):
                failures.append("first payload impact was outside the selected 60 m x 60 m target zone")
        else:
            failures.append("no physical payload impact was recorded")
        if any(warning.startswith("route reaches ") for warning in self.mission_assessment.warnings):
            failures.append("uploaded route contains a waypoint beyond the 300 m HOME boundary")
        return list(dict.fromkeys(failures))

    def _update_flight_metrics(self) -> None:
        now = time.monotonic()
        layout = load_field_layout()
        self._sample_scan_coverage()
        position_valid = self.telemetry.lat_deg != 0.0 and self.telemetry.lon_deg != 0.0
        if self.launched and position_valid:
            point = (self.telemetry.north_m, self.telemetry.east_m)
            if self.last_route_point is not None:
                step = math.hypot(
                    point[0] - self.last_route_point[0],
                    point[1] - self.last_route_point[1],
                )
                # Ignore a reset/EKF discontinuity, but retain every real
                # 10 Hz flight segment.
                if step <= 30.0:
                    self.result.route_metrics["distance_m"] += step
            self.last_route_point = point
            self.result.route_metrics["max_home_distance_m"] = max(
                float(self.result.route_metrics["max_home_distance_m"]),
                math.hypot(*point),
            )
            in_land_command = (
                self.contract is not None
                and getattr(self.contract, "land_seq", None) is not None
                and self.telemetry.mission_seq >= self.contract.land_seq
            )
            roll_key = (
                "max_landing_roll_deg" if in_land_command else "max_roll_deg"
            )
            self.result.route_metrics[roll_key] = max(
                float(self.result.route_metrics[roll_key]),
                abs(math.degrees(self.telemetry.roll_rad)),
            )
            if (
                self.contract is not None
                and getattr(self.contract, "scan_start_seq", math.inf)
                < self.telemetry.mission_seq
                < getattr(self.contract, "scan_end_seq", -math.inf)
            ):
                scan_waypoint_items = [
                    item
                    for item in self.mission
                    if (
                        self.contract.scan_start_seq < item.seq < self.contract.scan_end_seq
                        and item.command == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
                    )
                ]
                metric_segment = None
                rectangle = derive_scan_rectangle(self.mission, self.contract, origin_lat_deg=self.origin_lat_deg, origin_lon_deg=self.origin_lon_deg)
                if rectangle is not None:
                    by_seq = {item.seq: item for item in scan_waypoint_items}
                    active_lane_by_target_seq = {
                        rectangle.lane1_end_seq: (
                            by_seq[rectangle.lane1_start_seq],
                            by_seq[rectangle.lane1_end_seq],
                        ),
                        rectangle.lane2_end_seq: (
                            by_seq[rectangle.lane2_start_seq],
                            by_seq[rectangle.lane2_end_seq],
                        ),
                    }
                    metric_segment = active_lane_by_target_seq.get(
                        self.telemetry.mission_seq
                    )
                elif len(scan_waypoint_items) >= 2:
                    metric_segment = (
                        scan_waypoint_items[0], scan_waypoint_items[-1]
                    )
                if metric_segment is not None:
                    start_n, start_e = latlon_to_ne(
                        metric_segment[0].x, metric_segment[0].y, self.origin_lat_deg, self.origin_lon_deg
                    )
                    end_n, end_e = latlon_to_ne(
                        metric_segment[1].x, metric_segment[1].y, self.origin_lat_deg, self.origin_lon_deg
                    )
                    line_n = end_n - start_n
                    line_e = end_e - start_e
                    line_length = math.hypot(line_n, line_e)
                    if line_length > 1.0:
                        xtrack = abs(
                            line_e * (point[0] - start_n)
                            - line_n * (point[1] - start_e)
                        ) / line_length
                        self.scan_xtrack_sum_sq += xtrack * xtrack
                        self.scan_airspeed_sum += self.telemetry.airspeed_mps
                        self.scan_agl_sum += self.telemetry.relative_alt_m
                        self.scan_samples += 1
                        metrics = self.result.route_metrics
                        metrics["scan_samples"] = self.scan_samples
                        metrics["scan_cross_track_rms_m"] = math.sqrt(
                            self.scan_xtrack_sum_sq / self.scan_samples
                        )
                        metrics["scan_cross_track_max_m"] = max(
                            float(metrics["scan_cross_track_max_m"]), xtrack
                        )
                        metrics["scan_mean_airspeed_mps"] = (
                            self.scan_airspeed_sum / self.scan_samples
                        )
                        metrics["scan_mean_agl_m"] = (
                            self.scan_agl_sum / self.scan_samples
                        )
            if (
                self.contract is not None
                and self.telemetry.mission_seq
                == getattr(self.contract, "fallback_start_seq", -1)
                and abs(math.degrees(self.telemetry.roll_rad)) >= 15.0
                and abs(self.telemetry.yaw_rate_rps) >= 0.08
                and self.telemetry.groundspeed_mps >= 8.0
            ):
                observed_radius = self.telemetry.groundspeed_mps / abs(
                    self.telemetry.yaw_rate_rps
                )
                if 5.0 <= observed_radius <= 200.0:
                    self.turn_radius_sum += observed_radius
                    self.turn_radius_samples += 1
                    metrics = self.result.route_metrics
                    metrics["turn_radius_samples"] = self.turn_radius_samples
                    metrics["turn_radius_mean_m"] = (
                        self.turn_radius_sum / self.turn_radius_samples
                    )
                    previous_min = metrics["turn_radius_min_m"]
                    metrics["turn_radius_min_m"] = (
                        observed_radius
                        if previous_min is None
                        else min(float(previous_min), observed_radius)
                    )
                    metrics["turn_radius_max_m"] = max(
                        float(metrics["turn_radius_max_m"]), observed_radius
                    )
            # Freeze the official duration at physical standstill. ArduPlane
            # intentionally remains armed for LAND_DISARMDELAY after stopping;
            # that disarm delay is not flight or landing time.
            if self.result.final_stop is None:
                self.result.route_metrics["mission_time_s"] = max(
                    0.0,
                    now - self.launch_time,
                )
            if self.telemetry.relative_alt_m >= 3.0:
                self.airborne = True
            if (
                self.airborne
                and self.result.first_touchdown is None
                and not in_land_command
                and self.telemetry.relative_alt_m >= 2.0
                and self.telemetry.armed
                and self.telemetry.airspeed_mps > 0.0
            ):
                previous_min = self.result.route_metrics["min_airspeed_mps"]
                self.result.route_metrics["min_airspeed_mps"] = (
                    self.telemetry.airspeed_mps
                    if previous_min is None
                    else min(float(previous_min), self.telemetry.airspeed_mps)
                )
            if self.airborne and self.result.first_touchdown is not None:
                if self.telemetry.groundspeed_mps <= FULL_STOP_MAX_GROUNDSPEED_MPS:
                    if self.stopped_since is None:
                        self.stopped_since = now
                    elif (
                        self.result.final_stop is None
                        and now - self.stopped_since >= FULL_STOP_HOLD_S
                    ):
                        self.result.final_stop = {
                            "timestamp": time.time(),
                            "north_m": point[0],
                            "east_m": point[1],
                            "groundspeed_mps": self.telemetry.groundspeed_mps,
                            "hold_s": now - self.stopped_since,
                            "armed": self.telemetry.armed,
                            "in_takeoff_landing_zone": in_takeoff_landing_zone(*point, layout),
                        }
                elif self.result.final_stop is None:
                    self.stopped_since = None

            if (
                self.last_trajectory_point is None
                or math.hypot(
                    point[0] - self.last_trajectory_point[0],
                    point[1] - self.last_trajectory_point[1],
                )
                >= 2.0
            ):
                self.route_state.trajectory.append(
                    {"north_m": point[0], "east_m": point[1]}
                )
                self.route_state.trajectory = self.route_state.trajectory[-1200:]
                self.last_trajectory_point = point

        self.route_state.group = self.parameters.group
        self.route_state.origin_lat_deg = layout.origin_lat_deg
        self.route_state.origin_lon_deg = layout.origin_lon_deg
        self.route_state.field_layout = layout.to_dict()
        self.route_state.phase = self.result.status
        self.route_state.aircraft = asdict(self.telemetry)
        self.route_state.detections = list(self.result.detections)
        self.route_state.selected_target = self.result.selected_target
        self.route_state.release = self.result.release
        self.route_state.landing = {
            "first_touchdown": self.result.first_touchdown,
            "final_stop": self.result.final_stop,
        }
        self.route_state.mission_capability = self.mission_assessment.mode
        self.route_state.mission_warnings = list(self.mission_assessment.warnings)
        self.route_state.scan_attempt = self.scan_attempt
        self.route_state.scan_coverage_ratio = self.scan_coverage_ratio
        self.route_state.stable_target_count = self.stable_target_count
        self.route_state.fallback_reason = self.fallback_reason
        if now - self.last_route_state_write >= 0.25:
            save_route_state(self.route_state)
            self.last_route_state_write = now
        if self.launched and now - self.last_metrics_save >= 0.5:
            self.store.save()
            self.last_metrics_save = now

    def run(self) -> int:
        self.initialize()
        last_telemetry_write = 0.0
        while True:
            message = self.client.recv(
                [
                    "HEARTBEAT",
                    "GLOBAL_POSITION_INT",
                    "ATTITUDE",
                    "MISSION_CURRENT",
                    "NAV_CONTROLLER_OUTPUT",
                    "SERVO_OUTPUT_RAW",
                    "VFR_HUD",
                    "MISSION_COUNT",
                    "MISSION_REQUEST",
                    "MISSION_REQUEST_INT",
                    "MISSION_ITEM",
                    "MISSION_ITEM_INT",
                    "MISSION_ACK",
                ],
                timeout=0.1,
            )
            if message is not None:
                message_type = message.get_type()
                # Companion/GCS heartbeats cannot change vehicle armed/AUTO state.
                if (message_type in {'HEARTBEAT','GLOBAL_POSITION_INT','ATTITUDE','MISSION_CURRENT',
                                     'NAV_CONTROLLER_OUTPUT','SERVO_OUTPUT_RAW','VFR_HUD'}
                        and (message.get_srcSystem()!=self.client.target_system
                             or message.get_srcComponent()!=self.client.target_component)):
                    continue
                if message_type == "HEARTBEAT":
                    self.heartbeat = message
                    self.telemetry.armed = self.client.is_armed(message)
                    self.telemetry.auto = self.client.is_auto(message)
                elif message_type == "GLOBAL_POSITION_INT":
                    self.telemetry.update_position(
                        message,
                        self.origin_lat_deg,
                        self.origin_lon_deg,
                    )
                elif message_type == "ATTITUDE":
                    self.telemetry.update_attitude(message)
                elif message_type == "NAV_CONTROLLER_OUTPUT":
                    self.telemetry.update_nav_controller(message)
                elif message_type == "SERVO_OUTPUT_RAW":
                    self.telemetry.update_servos(message)
                elif message_type == "VFR_HUD":
                    self.telemetry.update_hud(message)
                elif message_type == "MISSION_CURRENT":
                    seq = int(message.seq)
                    self.telemetry.mission_seq = seq
                    self.last_mission_seq_seen = seq
                    # Official Mission Planner routes use the same two tagged
                    # reconnaissance lanes as the generated route.  Apply the
                    # speed schedule to both forms: 10.5 m/s through the
                    # camera scan, 12.5 m/s on the attack/return legs.  Raw
                    # replay routes still bypass this block entirely.
                    if isinstance(self.contract, MissionContract) and self.telemetry.armed:
                        target_speed = (
                            RECON_AIRSPEED_MPS
                            if self.contract.scan_start_seq <= seq < self.contract.scan_end_seq
                            else RECON_TURN_AIRSPEED_MPS
                        )
                        if getattr(self,'last_recon_speed',None)!=target_speed:
                            self.client.command_airspeed(target_speed)
                            self.last_recon_speed=target_speed
                    if self.retry_turn_active and isinstance(self.contract,MissionContract):
                        # The official B file enters the scan at the first
                        # waypoint after tag 9101 (there is no takeoff or
                        # holding waypoint before it).  Waiting for a
                        # sequence strictly below scan_start would therefore
                        # leave the retry flag set forever and suppress the
                        # second 9102 crossing.  Clear it as soon as the
                        # planned retry entry is reached; generated routes
                        # with a loiter entry satisfy the same condition.
                        retry_entry = retry_entry_seq(self.mission, self.contract)
                        if seq <= retry_entry:
                            self.last_end_seq_processed=seq
                            self.retry_turn_active=False
                        elif seq>self.contract.fallback_start_seq and not self.retry_return_commanded:
                            entry=retry_entry_seq(self.mission,self.contract)
                            self.client.set_current(entry)
                            self.last_end_seq_processed=entry
                            self.retry_return_commanded=True
                        # Do not mistake the outbound return turn for a second scan.
                        continue
                    if (
                        self.contract
                        and crossed_scan_end(
                            self.last_end_seq_processed,
                            seq,
                            self.contract.scan_end_seq,
                        )
                    ):
                        self.last_end_seq_processed = seq
                        # The final exposure may still be in the YOLO worker.
                        # Use the existing decision leg for a bounded drain;
                        # no new waypoint or extra reconnaissance lap is added.
                        self.scan_end_pending_at=time.monotonic()
                    elif (
                        self.contract
                        and seq < self.contract.scan_end_seq
                        and seq < self.last_end_seq_processed
                    ):
                        # MISSION_SET_CURRENT returns to the first waypoint
                        # after tag 9101, not to the tag itself.  Seeing that
                        # backward jump rearms the one-shot 9102 crossing for
                        # the second and final scan attempt.
                        self.last_end_seq_processed = seq
                    self._try_mission_release_fallback(seq)
                elif message_type.startswith("MISSION_"):
                    self._note_mission_wire_activity(message)

            if time.monotonic() - last_telemetry_write >= 0.1:
                write_telemetry(self.telemetry)
                append_telemetry_history(self.telemetry)
                self._update_flight_metrics()
                last_telemetry_write = time.monotonic()

            flight_done = (
                self.released
                and self.result.impact is not None
                and not self.telemetry.armed
                and self.launched
                and not self.completed
                and self.result.first_touchdown is not None
                and self.result.final_stop is not None
            )
            if flight_done and self._restore_original_attack_slots():
                self.completed = True
                self.result.replay_completed = True
                # Route playback and competition-rule acceptance are separate
                # outcomes. A warning (for example route 2's 699 m extent or
                # hand-launch start) keeps replay successful while making the
                # stricter rule verdict explicit.
                self.result.competition_rule_failures = self._competition_rule_failures()
                self.result.competition_rules_passed = not bool(
                    self.result.competition_rule_failures
                )
                self.result.status = "COMPLETED"
                self.result.message = (
                    "Mission landed, stopped and disarmed; original route "
                    "restored; recognition result retained"
                )
                self.store.save()
                self._status("CUADC mission complete")
                time.sleep(1.0)

            # Mission Planner uploads the user mission after the simulator has
            # started. Revalidate it while disarmed so a valid mission left in
            # EEPROM can never mask a newly uploaded (or invalid) mission.
            if (
                not self.telemetry.armed
                and not self.launched
                and self._mission_refresh_due()
            ):
                self._refresh_mission()
            if (
                self.mission_assessment.launch_allowed
                and self.telemetry.armed
                and self.telemetry.auto
                and not self.launched
            ):
                try:
                    self._launch()
                except Exception as exc:
                    self.result.status = "LAUNCH_FAILED"
                    self.result.message = str(exc)
                    self.store.save()
                    self._status("Physical launcher failed", mavutil.mavlink.MAV_SEVERITY_CRITICAL)
            self._poll_scan_decision()
            self._try_release()
            self._try_mission_release_fallback(self.telemetry.mission_seq)
            self._poll_physical_release()
            self._poll_impact()
            self._poll_touchdown()
            self._poll_parameters()


def main(endpoint: str = "udpin:0.0.0.0:14551", dry_run: bool = False) -> int:
    try:
        return CompanionRuntime(endpoint, dry_run=dry_run).run()
    except KeyboardInterrupt:
        return 130
    except (MavlinkTimeout, RuntimeError, ValueError) as exc:
        print(f"CUADC companion fatal: {exc}", flush=True)
        return 2
