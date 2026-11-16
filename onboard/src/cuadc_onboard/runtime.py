from __future__ import annotations

from dataclasses import asdict, replace
import json
import math
import os
from pathlib import Path
import signal
import threading
import time

from pymavlink import mavutil

from .ballistics import (
    ReleaseSolution,
    along_track_error,
    crossed_release_line,
    solve_release,
)
from .mavlink_io import MavlinkClient, MavlinkTimeout
from .mission import (
    DO_SET_SERVO,
    MISSION_MODE_BLOCKED,
    JUMP_TAG,
    MISSION_MODE_CONTRACT_WARNING,
    MISSION_MODE_PASSTHROUGH,
    MISSION_MODE_PAYLOAD,
    MISSION_MODE_RECON_ONLY,
    MISSION_MODE_DYNAMIC_READY,
    MISSION_MODE_FIXED_FALLBACK,
    FixedFallbackContract,
    MissionContract,
    MissionAssessment,
    MissionItem,
    ReconContract,
    assess_mission,
    build_attack_patch,
    classify_mission_contract,
    compare_mission_reference,
    crossed_scan_end,
    retry_entry_seq,
    derive_scan_rectangle,
    is_shadow_neutralization,
    neutralize_release_item,
    latlon_to_ne,
    load_qgc_waypoints,
    load_patch_backup,
    mission_fingerprint,
    save_patch_backup,
    scan_coverage_cells,
    update_scan_coverage,
    validate_mission,
)
from .selection import Detection, Selection, select_target
from .telemetry import Telemetry
from .precision import PrecisionBudget, estimate_impact_error

from .camera import CameraVisionWorker
from .config import OnboardConfig
from .service import SystemdWatchdog, rotate_file


ONBOARD_STATUS_CODES = {
    "STARTING": 0,
    "WAITING_FOR_NAVIGATION": 1,
    "READY": 2,
    "VISION_FAILED": 3,
    "MISSION_INVALID": 4,
    "PAYLOAD_SERVO_INVALID": 5,
    "FATAL": 6,
    "RECON_SELECTION": 7,
    "FALLBACK_ROUTE_ACTIVE": 8,
    "ATTACK_ROUTE_ACTIVE": 9,
    "RELEASE_COMMAND_SENT": 10,
    "RELEASE_OUTPUT_OBSERVED": 11,
    "MISSION_RESTORE_FAILED": 12,
    "FLIGHT_COMPLETE": 13,
    "FLIGHT_CONTROLLER_HEALTH_FAULT": 14,
    "MISSION_PASSTHROUGH": 15,
    "MISSION_CONTRACT_WARNING": 16,
    "MISSION_UPDATE_PENDING": 17,
    "SCAN_RETRY": 18,
}


# ArduPilot normally repeats pre-arm messages on an approximately 30-second
# cadence.  Keep transient flight-controller health faults active slightly
# longer than that cadence so a continuously reported fault cannot flicker
# back to READY between messages.  CrashDump is intentionally stronger: once
# observed it remains latched for this process lifetime and can only be cleared
# by restarting after the dump has been retrieved and the aircraft inspected.
FLIGHT_CONTROLLER_HEALTH_WARNING_TTL_S = 45.0
STATUSTEXT_FRAGMENT_LENGTH = 50
STATUSTEXT_FRAGMENT_TTL_S = 5.0
STATUSTEXT_MAX_FRAGMENTS = 4
SCAN_COVERAGE_THRESHOLD = 0.90
MIN_STABLE_TARGETS = 3


def atomic_write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)


def solution_with_actuator_delay(
    solution: ReleaseSolution,
    velocity_north_mps: float,
    velocity_east_mps: float,
    delay_s: float,
) -> ReleaseSolution:
    if delay_s < 0.0:
        raise ValueError("actuator delay cannot be negative")
    extra_north = velocity_north_mps * delay_s
    extra_east = velocity_east_mps * delay_s
    return replace(
        solution,
        north_lead_m=solution.north_lead_m + extra_north,
        east_lead_m=solution.east_lead_m + extra_east,
        release_north_m=solution.release_north_m - extra_north,
        release_east_m=solution.release_east_m - extra_east,
    )


def release_gate_reasons(
    config: OnboardConfig,
    telemetry: Telemetry,
    contract: MissionContract | FixedFallbackContract | ReconContract | None,
    heartbeat_age_s: float,
    position_age_s: float,
    attitude_age_s: float,
    airspeed_age_s: float,
    gps_fix_type: int,
    satellites_visible: int,
    gps_hdop: float,
    servo_config_valid: bool,
    home_valid: bool,
    target_north_m: float,
    target_east_m: float,
    flight_controller_health_ok: bool = True,
) -> list[str]:
    reasons: list[str] = []
    release = config.release
    mission = config.mission
    if not release.enabled:
        reasons.append("release disabled by configuration")
    if release.precision_required:
        if not release.precision_calibrated:
            reasons.append("precision release has not been calibrated")
        if telemetry.relative_alt_m>0:
            budget=PrecisionBudget.from_release(release)
            estimate=estimate_impact_error(telemetry.relative_alt_m,telemetry.airspeed_mps,
                                           release.actuator_delay_s,release.drag_rate_s,budget)
            if estimate['r95_m']>release.target_tolerance_m:
                reasons.append("predicted 95% impact error exceeds target tolerance")
    if contract is None:
        reasons.append("mission contract is not valid")
    elif isinstance(contract, FixedFallbackContract):
        reasons.append("fixed-fallback mission disables primary dynamic release")
    elif isinstance(contract, ReconContract):
        reasons.append("recon-only mission permanently disables release")
    elif not (
        contract.fallback_start_seq + 1
        <= telemetry.mission_seq
        <= contract.fallback_release_seq
    ):
        reasons.append("aircraft is outside the patched attack segment")
    if not telemetry.armed:
        reasons.append("aircraft is disarmed")
    if mission.require_auto and not telemetry.auto:
        reasons.append("flight mode is not AUTO")
    if heartbeat_age_s > config.flight_controller.telemetry_timeout_s:
        reasons.append("heartbeat is stale")
    if position_age_s > config.flight_controller.telemetry_timeout_s:
        reasons.append("position is stale")
    if attitude_age_s > config.flight_controller.telemetry_timeout_s:
        reasons.append("attitude is stale")
    if airspeed_age_s > config.flight_controller.telemetry_timeout_s:
        reasons.append("airspeed is stale")
    if not home_valid:
        reasons.append("HOME_POSITION is unavailable")
    if mission.require_3d_gps and gps_fix_type < 3:
        reasons.append("GPS fix is below 3D")
    if satellites_visible == 255 or satellites_visible < mission.min_satellites:
        reasons.append("too few GPS satellites")
    if not math.isfinite(gps_hdop) or gps_hdop > mission.max_hdop:
        reasons.append("GPS HDOP is outside the release gate")
    if not servo_config_valid:
        reasons.append("payload servo configuration is not verified")
    if not flight_controller_health_ok:
        reasons.append("flight controller reports an unresolved health fault")
    if not release.min_agl_m <= telemetry.relative_alt_m <= release.max_agl_m:
        reasons.append("relative altitude is outside the release gate")
    if not release.min_airspeed_mps <= telemetry.airspeed_mps <= release.max_airspeed_mps:
        reasons.append("airspeed is outside the release gate")
    if abs(math.degrees(telemetry.roll_rad)) > release.max_roll_deg:
        reasons.append("roll angle is outside the release gate")
    if abs(math.degrees(telemetry.yaw_rate_rps)) > release.max_yaw_rate_dps:
        reasons.append("yaw rate is outside the release gate")
    target_delta_n = target_north_m - telemetry.north_m
    target_delta_e = target_east_m - telemetry.east_m
    closing_dot = (
        target_delta_n * telemetry.velocity_north_mps
        + target_delta_e * telemetry.velocity_east_mps
    )
    if closing_dot <= 0.0:
        reasons.append("ground track is not closing on the selected target")
    if math.hypot(target_north_m, target_east_m) > mission.max_home_distance_m:
        reasons.append("selected target is outside the home safety boundary")
    return reasons


class OnboardRuntime:
    """Real-aircraft executive; contains no Gazebo or SITL control path."""

    def __init__(self, config: OnboardConfig):
        self.config = config
        self.runtime_dir = config.runtime.path()
        self.runtime_dir.mkdir(parents=True, exist_ok=True)
        self.client = MavlinkClient(
            config.flight_controller.endpoint,
            baud=config.flight_controller.baud,
        )
        self.telemetry = Telemetry()
        self.telemetry_lock = threading.Lock()
        self.home_lat_deg = 0.0
        self.home_lon_deg = 0.0
        self.home_alt_m = 0.0
        self.home_valid = False
        self.gps_fix_type = 0
        self.satellites_visible = 0
        self.gps_hdop = math.inf
        self.last_heartbeat = -math.inf
        self.last_position = -math.inf
        self.last_attitude = -math.inf
        self.last_airspeed = -math.inf
        self.last_status_write = 0.0
        self.last_mavlink_report = -math.inf
        self.last_mission_refresh = -math.inf
        self.last_status_text = -math.inf
        self.last_home_request = -math.inf
        self.mission: list[MissionItem] = []
        self.contract: MissionContract | FixedFallbackContract | ReconContract | None = None
        self.mission_assessment = MissionAssessment(MISSION_MODE_BLOCKED, None)
        self.expected_mission_path = (
            config.mission.expected_path()
            if config.mission.acceptance_mode == "pinned"
            else None
        )
        self.expected_mission = (
            load_qgc_waypoints(self.expected_mission_path)
            if self.expected_mission_path is not None
            else []
        )
        self.original_slots: list[MissionItem] = []
        self.backup_path = self.runtime_dir / "mission_patch_backup.json"
        self.last_end_seq_processed = -1
        self.selected: Selection | None = None
        self.release_commanded = False
        self.release_observed = False
        self.previous_along_track = math.inf
        self.ever_armed = False
        self.restore_pending = False
        self.scan_attempt = 1
        self.retry_turn_active = False
        self.retry_return_commanded = False
        self.scan_cells: list[tuple[float, float, float]] = []
        self.scan_rectangle = None
        self.covered_scan_cells: set[int] = set()
        self.scan_coverage_ratio = 0.0
        self.stable_target_count = 0
        self.scan_fallback_reason: str | None = None
        self.pending_mission_fingerprint: str | None = None
        self.pending_mission_readbacks = 0
        self.accepted_mission_fingerprint: str | None = None
        self.accepted_semantic_fingerprint: str | None = None
        self.stop_requested = False
        self.watchdog = SystemdWatchdog()
        self.wind_north_mps = 0.0
        self.wind_east_mps = 0.0
        self.wind_samples = 0
        self.servo_config_valid = False
        self.servo_configuration: dict = {}
        self.last_servo_check = -math.inf
        self.flight_controller_health_issues: dict[str, dict] = {}
        self._flight_controller_status_text_fragments: dict[tuple[int, int, int], dict] = {}
        self._nominal_status = "STARTING"
        self._nominal_message = "Connecting to ArduPlane"
        self.vision = CameraVisionWorker(
            config,
            self.telemetry_snapshot,
            self.runtime_dir,
        )
        self.state: dict = {
            "schema": "cuadc-onboard-state.v1",
            "mode": "REAL_AIRCRAFT",
            "status": "STARTING",
            "message": "Connecting to ArduPlane",
            "task": config.vision.task,
            "release_enabled": config.release.enabled,
            "release_commanded": False,
            "release_observed": False,
            "selected_target": None,
            "release": None,
            "mission_errors": [],
            "mission_mode": "UNKNOWN",
            "mission_warnings": [],
            "scan_attempt": 1,
            "scan_coverage_ratio": 0.0,
            "scan_rectangle": None,
            "stable_target_count": 0,
            "scan_fallback_reason": None,
            "flight_controller_health": {
                "status": "HEALTHY",
                "release_blocked": False,
                "crash_dump_detected": False,
                "warning_ttl_s": FLIGHT_CONTROLLER_HEALTH_WARNING_TTL_S,
                "issues": [],
            },
            "mission_reference": {
                "acceptance_mode": config.mission.acceptance_mode,
                "file": (
                    str(self.expected_mission_path)
                    if self.expected_mission_path is not None
                    else None
                ),
                "sha256": config.mission.expected_sha256 or None,
                "expected_count": len(self.expected_mission),
                "matched": False if self.expected_mission else None,
                "last_checked_at": None,
            },
            "started_at": time.time(),
        }

    def telemetry_snapshot(self) -> dict:
        with self.telemetry_lock:
            payload = asdict(self.telemetry)
        now = time.monotonic()
        payload.update(
            {
                "home_valid": self.home_valid,
                "position_valid": now - self.last_position
                <= self.config.flight_controller.telemetry_timeout_s,
                "gps_fix_type": self.gps_fix_type,
                "satellites_visible": self.satellites_visible,
                "gps_hdop": (
                    self.gps_hdop if math.isfinite(self.gps_hdop) else None
                ),
            }
        )
        return payload

    def _set_status(self, status: str, message: str) -> None:
        self._nominal_status = status
        self._nominal_message = message
        self._sync_flight_controller_health_state()

    @staticmethod
    def _decoded_status_text_fragment(message) -> str:
        text = getattr(message, "text", "")
        if isinstance(text, bytes):
            text = text.decode("utf-8", errors="replace")
        return str(text).rstrip("\x00")[:STATUSTEXT_FRAGMENT_LENGTH]

    @classmethod
    def _decoded_status_text(cls, message) -> str:
        return cls._decoded_status_text_fragment(message).strip()

    def _message_is_from_target_autopilot(self, message) -> bool:
        try:
            return bool(
                int(message.get_srcSystem()) == int(self.client.target_system)
                and int(message.get_srcComponent())
                == mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
                and int(message.get_srcComponent())
                == int(self.client.target_component)
            )
        except (AttributeError, TypeError, ValueError):
            return False

    @staticmethod
    def _classify_flight_controller_health_text(
        text: str,
        severity: int,
    ) -> tuple[str, bool] | None:
        """Return ``(code, latched)`` for release-blocking FC messages.

        STATUSTEXT is also used for routine mode and calibration notices, so
        severity alone is not sufficient for warnings.  All PreArm/Arm
        failures and ERROR-or-higher messages are safety relevant; WARNING
        messages are included only when they name a core flight-health system.
        """

        folded = " ".join(text.casefold().split())
        compact = folded.replace(" ", "")
        if "crashdumpdatadetected" in compact:
            return "CRASH_DUMP_DETECTED", True
        if folded.startswith("prearm:"):
            return "PREARM_FAILURE", False
        if folded.startswith("arm:"):
            return "ARMING_FAILURE", False
        if severity <= mavutil.mavlink.MAV_SEVERITY_ERROR:
            return "FLIGHT_CONTROLLER_ERROR", False
        health_terms = (
            "ahrs",
            "ekf",
            "gps",
            "compass",
            "gyro",
            "accelerometer",
            "barometer",
            "battery failsafe",
            "failsafe",
            "watchdog",
        )
        if (
            severity <= mavutil.mavlink.MAV_SEVERITY_WARNING
            and any(term in folded for term in health_terms)
        ):
            return "FLIGHT_CONTROLLER_WARNING", False
        return None

    def _record_flight_controller_health_text(
        self,
        text: str,
        severity: int,
    ) -> None:
        text = text.strip()[: STATUSTEXT_FRAGMENT_LENGTH * STATUSTEXT_MAX_FRAGMENTS]
        if not text:
            return
        classification = self._classify_flight_controller_health_text(
            text,
            severity,
        )
        if classification is None:
            return
        code, latched = classification
        now_monotonic = time.monotonic()
        now_wall = time.time()
        key = f"{code}:{' '.join(text.casefold().split())}"
        previous = self.flight_controller_health_issues.get(key)
        self.flight_controller_health_issues[key] = {
            "code": code,
            "text": text,
            "severity": severity,
            "latched": latched or bool(previous and previous["latched"]),
            "first_seen_at": (
                previous["first_seen_at"] if previous is not None else now_wall
            ),
            "last_seen_at": now_wall,
            "_last_seen_monotonic": now_monotonic,
        }
        self._sync_flight_controller_health_state(now_monotonic)

    def _expire_flight_controller_status_text_fragments(self, now: float) -> None:
        expired = [
            key
            for key, pending in self._flight_controller_status_text_fragments.items()
            if now - float(pending["updated_at"]) > STATUSTEXT_FRAGMENT_TTL_S
        ]
        expired_pending = []
        for key in expired:
            pending = self._flight_controller_status_text_fragments.pop(key, None)
            if pending is not None:
                expired_pending.append(pending)
        for pending in expired_pending:
            # A lost continuation packet must not make a safety-relevant first
            # fragment disappear.  Classify the partial text after the short
            # reassembly window; benign fragments remain ignored.
            self._record_flight_controller_health_text(
                "".join(pending["fragments"]),
                int(pending["severity"]),
            )

    def _handle_flight_controller_status_text(self, message) -> None:
        if not self._message_is_from_target_autopilot(message):
            return
        now = time.monotonic()
        self._expire_flight_controller_status_text_fragments(now)
        fragment = self._decoded_status_text_fragment(message)
        if not fragment:
            return
        try:
            severity = int(message.severity)
        except (AttributeError, TypeError, ValueError):
            severity = mavutil.mavlink.MAV_SEVERITY_NOTICE
        try:
            message_id = int(getattr(message, "id", 0))
            chunk_seq = int(getattr(message, "chunk_seq", 0))
            source_system = int(message.get_srcSystem())
            source_component = int(message.get_srcComponent())
        except (AttributeError, TypeError, ValueError):
            message_id = 0
            chunk_seq = 0
            source_system = int(self.client.target_system)
            source_component = int(self.client.target_component)

        # MAVLink 1 and unchunked MAVLink 2 messages use id=0.
        if message_id == 0:
            self._record_flight_controller_health_text(fragment, severity)
            return
        if not 0 <= chunk_seq < STATUSTEXT_MAX_FRAGMENTS:
            return

        key = (source_system, source_component, message_id)
        if chunk_seq == 0:
            pending = {
                "fragments": [],
                "severity": severity,
                "updated_at": now,
            }
            self._flight_controller_status_text_fragments[key] = pending
        else:
            pending = self._flight_controller_status_text_fragments.get(key)
            if pending is None:
                return
        fragments = pending["fragments"]
        if chunk_seq != len(fragments):
            self._flight_controller_status_text_fragments.pop(key, None)
            # UDP loss/reordering must fail safe for a safety-relevant prefix
            # already received in-order.  Do not silently discard that prefix.
            self._record_flight_controller_health_text(
                "".join(fragments),
                int(pending["severity"]),
            )
            return
        fragments.append(fragment)
        pending["updated_at"] = now
        if len(fragment) < STATUSTEXT_FRAGMENT_LENGTH:
            self._flight_controller_status_text_fragments.pop(key, None)
            self._record_flight_controller_health_text(
                "".join(fragments),
                int(pending["severity"]),
            )

    def _active_flight_controller_health_issues(
        self,
        now: float | None = None,
    ) -> list[dict]:
        now = time.monotonic() if now is None else now
        expired = [
            key
            for key, issue in self.flight_controller_health_issues.items()
            if not issue["latched"]
            and now - float(issue["_last_seen_monotonic"])
            > FLIGHT_CONTROLLER_HEALTH_WARNING_TTL_S
        ]
        for key in expired:
            del self.flight_controller_health_issues[key]
        return sorted(
            self.flight_controller_health_issues.values(),
            key=lambda issue: (
                0 if issue["latched"] else 1,
                int(issue["severity"]),
                str(issue["text"]),
            ),
        )

    def _sync_flight_controller_health_state(
        self,
        now: float | None = None,
    ) -> bool:
        now = time.monotonic() if now is None else now
        # This sync runs periodically even when no further STATUSTEXT arrives,
        # so an exactly-50-byte safety prefix cannot remain pending forever if
        # its continuation packet is lost.
        self._expire_flight_controller_status_text_fragments(now)
        active = self._active_flight_controller_health_issues(now)
        crash_dump_detected = any(
            issue["code"] == "CRASH_DUMP_DETECTED" for issue in active
        )
        public_issues = [
            {key: value for key, value in issue.items() if not key.startswith("_")}
            for issue in active
        ]
        self.state["flight_controller_health"] = {
            "status": "FAULT" if active else "HEALTHY",
            "release_blocked": bool(active),
            "crash_dump_detected": crash_dump_detected,
            "warning_ttl_s": FLIGHT_CONTROLLER_HEALTH_WARNING_TTL_S,
            "issues": public_issues,
        }
        if active and self._nominal_status != "FATAL":
            self.state["status"] = "FLIGHT_CONTROLLER_HEALTH_FAULT"
            if crash_dump_detected:
                self.state["message"] = (
                    "Flight controller CrashDump data is unresolved; "
                    "READY and active release are blocked"
                )
            else:
                self.state["message"] = (
                    f"Flight controller health fault: {active[0]['text']}; "
                    "READY and active release are blocked"
                )
        else:
            self.state["status"] = self._nominal_status
            self.state["message"] = self._nominal_message
        return bool(active)

    def _status_text(
        self,
        text: str,
        severity: int = mavutil.mavlink.MAV_SEVERITY_NOTICE,
    ) -> None:
        now = time.monotonic()
        if now - self.last_status_text < 1.0:
            return
        self.last_status_text = now
        try:
            self.client.status_text(text, severity)
        except Exception:
            pass

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

    def _request_home_position(self) -> None:
        connection = self.client.connection
        assert connection is not None
        connection.mav.command_long_send(
            self.client.target_system,
            self.client.target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            0,
            float(mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION),
            0,
            0,
            0,
            0,
            0,
            0,
        )
        self.last_home_request = time.monotonic()

    def initialize(self) -> None:
        self.client.connect(
            timeout=self.config.flight_controller.heartbeat_timeout_s
        )
        if self.client.initial_heartbeat is not None:
            self.telemetry.armed = self.client.is_armed(
                self.client.initial_heartbeat
            )
            self.telemetry.auto = self.client.is_auto(
                self.client.initial_heartbeat
            )
            self.last_heartbeat = time.monotonic()
        for message_id, interval_us in (
            (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 100_000),
            (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 100_000),
            (mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 100_000),
            (mavutil.mavlink.MAVLINK_MSG_ID_MISSION_CURRENT, 200_000),
            (mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, 100_000),
            (mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 500_000),
        ):
            self._request_message_interval(message_id, interval_us)
        self._request_home_position()
        self._read_servo_configuration()
        self._refresh_mission()
        if self.config.release.enabled and not self.servo_config_valid:
            self._set_status(
                "PAYLOAD_SERVO_INVALID",
                "SERVO output function/range was not verified; primary release blocked",
            )
            self._status_text(
                "Payload SERVO configuration invalid - release blocked",
                mavutil.mavlink.MAV_SEVERITY_CRITICAL,
            )
        self.vision.start()
        if self._nominal_status == "READY":
            self._set_status(
                "WAITING_FOR_NAVIGATION",
                "Connected; waiting for HOME_POSITION, GPS and USB vision",
            )
        self._status_text(
            "CUADC online comp=191 "
            f"task={self.config.vision.task} "
            f"mode={'active' if self.config.release.enabled else 'release-disabled'}"
        )
        self._write_status(force=True)

    def _read_servo_configuration(self, timeout: float = 12.0) -> None:
        self.last_servo_check = time.monotonic()
        channel = self.config.release.servo_channel
        names = {
            "function": f"SERVO{channel}_FUNCTION",
            "min_pwm": f"SERVO{channel}_MIN",
            "max_pwm": f"SERVO{channel}_MAX",
        }
        try:
            parameters = self.client.read_named_parameters(names.values(), timeout=timeout)
            values = {key: parameters[name] for key, name in names.items()}
            if not all(math.isfinite(value) for value in values.values()):
                raise ValueError("payload servo parameters must be finite")
        except Exception as exc:
            self.servo_configuration = {"valid": False, "error": str(exc)}
            self.servo_config_valid = False
            return
        self.servo_config_valid = bool(
            values["function"] == self.config.release.required_servo_function
            and values["min_pwm"] <= self.config.release.release_pwm
            <= values["max_pwm"]
        )
        self.servo_configuration = {
            **values,
            "valid": self.servo_config_valid,
            "required_function": self.config.release.required_servo_function,
            "release_pwm": self.config.release.release_pwm,
            "control_mode": (
                "RC8_AND_MAVLINK"
                if self.config.release.required_servo_function == 58
                else "MAVLINK_INDEPENDENT"
            ),
        }

    def _retry_servo_configuration(self) -> None:
        # Only retry blocking parameter transactions before arming. A timeout
        # stays fail-closed, and flight never waits on configuration recovery.
        if (
            self.servo_config_valid
            or self.telemetry.armed
            or self.telemetry.auto
            or time.monotonic() - self.last_servo_check < 10.0
        ):
            return
        self._read_servo_configuration(timeout=6.0)
        self.watchdog.ping()

    def _semantic_mission(
        self,
        live_mission: list[MissionItem],
        backup: list[MissionItem],
    ) -> tuple[list[MissionItem], list[str], list[int]]:
        """Reconstruct the approved mission without trusting a stale backup."""

        if not self.expected_mission:
            replacements = {item.seq: item for item in backup}
            return (
                [replacements.get(item.seq, item) for item in live_mission],
                [],
                sorted(replacements),
            )
        expected_by_seq = {item.seq: item for item in self.expected_mission}
        live_by_seq = {item.seq: item for item in live_mission}
        replacements: dict[int, MissionItem] = {}
        errors: list[str] = []
        substituted: list[int] = []
        for original in backup:
            expected = expected_by_seq.get(original.seq)
            if expected is None:
                errors.append(
                    f"mission backup seq {original.seq} is absent from approved mission"
                )
                continue
            item_errors, _report = compare_mission_reference(
                [original],
                [expected],
                max_position_error_m=self.config.mission.max_position_error_m,
            )
            if item_errors:
                errors.append(
                    f"mission backup seq {original.seq} is stale: "
                    + "; ".join(item_errors)
                )
                continue
            replacements[original.seq] = original
            substituted.append(original.seq)

        # An older release may have left a neutral NAV_DELAY in EEPROM without
        # its JSON backup. The hash-pinned approved item is a stronger source
        # of truth, so active-only releases may reconstruct that exact slot.
        for expected in self.expected_mission:
            live = live_by_seq.get(expected.seq)
            if live is not None and is_shadow_neutralization(live, expected):
                replacements[expected.seq] = expected
                if expected.seq not in substituted:
                    substituted.append(expected.seq)

        return (
            [replacements.get(item.seq, item) for item in live_mission],
            errors,
            sorted(substituted),
        )

    def _reset_for_new_mission(self) -> None:
        """Clear all per-flight decisions before adopting a different route."""

        self.last_end_seq_processed = -1
        self.selected = None
        self.release_commanded = False
        self.release_observed = False
        self.previous_along_track = math.inf
        self.ever_armed = False
        self.original_slots = []
        self.restore_pending = False
        self.scan_attempt = 1
        self.retry_turn_active = False
        self.retry_return_commanded = False
        self.scan_cells = []
        self.scan_rectangle = None
        self.covered_scan_cells = set()
        self.scan_coverage_ratio = 0.0
        self.stable_target_count = 0
        self.scan_fallback_reason = None
        self.state["selected_target"] = None
        self.state["release_commanded"] = False
        self.state["release_observed"] = False
        self.state["release"] = None
        self.state["scan_attempt"] = 1
        self.state["scan_coverage_ratio"] = 0.0
        self.state["scan_rectangle"] = None
        self.state["stable_target_count"] = 0
        self.state["scan_fallback_reason"] = None
        self.state.pop("nominal_release_solution", None)

    def _dynamic_semantic_mission(
        self,
        live_mission: list[MissionItem],
        backup: list[MissionItem],
    ) -> tuple[list[MissionItem], list[int]]:
        """Reconstruct only release slots neutralized by a legacy release.

        Unlike the pinned path, a sequence number alone is never enough to
        trust an old backup after a Mission Planner upload. A saved item is
        substituted only when the corresponding live item is its exact
        zero-parameter NAV_DELAY form. A missing legacy backup can also be
        reconstructed from the configured channel/PWM when the surrounding
        relative contract is otherwise present.
        """

        live_by_seq = {item.seq: item for item in live_mission}
        replacements: dict[int, MissionItem] = {}
        for original in backup:
            live = live_by_seq.get(original.seq)
            if live is not None and is_shadow_neutralization(live, original):
                replacements[original.seq] = original

        starts = [
            item.seq
            for item in live_mission
            if item.command == JUMP_TAG and int(round(item.param1)) == 9101
        ]
        ends = [
            item.seq
            for item in live_mission
            if item.command == JUMP_TAG and int(round(item.param1)) == 9102
        ]
        if len(starts) == 1 and len(ends) == 1 and starts[0] < ends[0]:
            release_seq = ends[0] + 4
            live_release = live_by_seq.get(release_seq)
            if live_release is not None and live_release.command != DO_SET_SERVO:
                reconstructed = replace(
                    live_release,
                    command=DO_SET_SERVO,
                    param1=float(self.config.release.servo_channel),
                    param2=float(self.config.release.release_pwm),
                )
                if is_shadow_neutralization(live_release, reconstructed):
                    replacements[release_seq] = reconstructed

        return (
            [replacements.get(item.seq, item) for item in live_mission],
            sorted(replacements),
        )

    def _refresh_dynamic_mission(self, force_stable: bool = False) -> None:
        live_mission: list[MissionItem] = []
        try:
            live_mission = self.client.download_mission(timeout=10.0)
        except Exception as exc:
            self.contract = None
            self.state["mission_errors"] = [str(exc)]
            self.state["mission_mode"] = "PENDING"
            self._set_status(
                "MISSION_UPDATE_PENDING",
                f"Mission readback not yet stable: {exc}",
            )
            return

        raw_fingerprint = mission_fingerprint(live_mission)
        if raw_fingerprint != self.pending_mission_fingerprint:
            self.pending_mission_fingerprint = raw_fingerprint
            self.pending_mission_readbacks = 1
        else:
            self.pending_mission_readbacks += 1
        stable_required = self.config.mission.stable_readbacks
        if not force_stable and self.pending_mission_readbacks < stable_required:
            # Immediately disarm companion actions when a new readback first
            # appears. The flight controller remains free to navigate it.
            self.mission = live_mission
            self.contract = None
            self.state["mission_errors"] = []
            self.state["mission_mode"] = "PENDING"
            self.state["mission_reference"] = {
                **self.state.get("mission_reference", {}),
                "acceptance_mode": "dynamic",
                "fingerprint": raw_fingerprint,
                "stable_readbacks": self.pending_mission_readbacks,
                "stable_readbacks_required": stable_required,
                "live_count": len(live_mission),
                "matched": None,
                "last_checked_at": time.time(),
            }
            self._set_status(
                "MISSION_UPDATE_PENDING",
                "New Mission Planner route detected; waiting for a second identical readback",
            )
            return

        # A same-process active patch has an in-memory original that is safe to
        # restore after landing. Mission uploads are forbidden while armed, so
        # this runs before accepting any later ground-side route replacement.
        if (
            self.config.release.enabled
            and self.restore_pending
            and self.original_slots
        ):
            if self.telemetry.armed:
                self.contract = None
                self._set_status(
                    "MISSION_RESTORE_FAILED",
                    "Dynamic slot restoration is pending while aircraft is armed",
                )
                return
            try:
                self.client.write_partial(self.original_slots)
                verified = self.client.download_mission(timeout=10.0)
                verified_by_seq = {item.seq: item for item in verified}
                restored = [verified_by_seq[item.seq] for item in self.original_slots]
                restore_errors, _report = compare_mission_reference(
                    restored,
                    self.original_slots,
                    max_position_error_m=self.config.mission.max_position_error_m,
                )
                if restore_errors:
                    raise RuntimeError("; ".join(restore_errors))
                live_mission = verified
                raw_fingerprint = mission_fingerprint(verified)
                self.pending_mission_fingerprint = raw_fingerprint
                self.pending_mission_readbacks = stable_required
                self.backup_path.unlink(missing_ok=True)
                self.original_slots = []
                self.restore_pending = False
            except Exception as exc:
                self.contract = None
                self.state["mission_errors"] = [str(exc)]
                self._set_status(
                    "MISSION_RESTORE_FAILED",
                    f"Dynamic mission slot restoration failed: {exc}",
                )
                return

        backup = load_patch_backup(self.backup_path)
        semantic_mission, substitutions = self._dynamic_semantic_mission(
            live_mission, backup
        )
        assessment = assess_mission(
            semantic_mission,
            payload_servo=self.config.release.servo_channel,
            min_release_pwm=float(self.config.release.release_pwm),
            origin_lat_deg=self.home_lat_deg if self.home_valid else None,
            origin_lon_deg=self.home_lon_deg if self.home_valid else None,
            max_home_distance_m=(
                self.config.mission.max_home_distance_m
                if self.home_valid
                else None
            ),
        )
        mission_mode = assessment.mode
        contract = assessment.contract
        contract_errors = list(assessment.errors)
        contract_warnings = list(assessment.warnings)
        operational_error: str | None = None

        # Both dynamic and fixed-fallback routes contain a real release item.
        # This active-only runtime may restore an exact legacy neutralization,
        # but it never creates a neutralized flight-controller mission.
        if mission_mode in {
            MISSION_MODE_DYNAMIC_READY,
            MISSION_MODE_FIXED_FALLBACK,
        } and contract is not None:
            semantic_by_seq = {item.seq: item for item in semantic_mission}
            live_by_seq = {item.seq: item for item in live_mission}
            semantic_release = semantic_by_seq[contract.fallback_release_seq]
            live_release = live_by_seq[contract.fallback_release_seq]
            try:
                if self.config.release.enabled and self.config.release.precision_required:
                    if not isinstance(contract,MissionContract):
                        raise RuntimeError('precision mode requires dynamic attack slots')
                    if not is_shadow_neutralization(live_release,semantic_release):
                        if self.telemetry.armed:
                            raise RuntimeError('precision interlock must be installed while disarmed')
                        save_patch_backup(self.backup_path,[semantic_release])
                        self.client.write_partial([neutralize_release_item(semantic_release)])
                        verified=self.client.download_mission(timeout=10.0)
                        verified_release={item.seq:item for item in verified}[contract.fallback_release_seq]
                        if not is_shadow_neutralization(verified_release,semantic_release):
                            raise RuntimeError('precision release-slot interlock readback failed')
                        live_mission=verified
                        raw_fingerprint=mission_fingerprint(verified)
                        self.pending_mission_fingerprint=raw_fingerprint
                        self.pending_mission_readbacks=stable_required
                    self.state['precision_only']=True
                    self.state['autonomous_mission_release']=False
                elif self.config.release.enabled and is_shadow_neutralization(
                    live_release, semantic_release
                ):
                    if self.telemetry.armed:
                        raise RuntimeError(
                            "release-slot activation is pending while aircraft is armed"
                        )
                    self.client.write_partial([semantic_release])
                    verified = self.client.download_mission(timeout=10.0)
                    verified_release = {
                        item.seq: item for item in verified
                    }[contract.fallback_release_seq]
                    verify_errors, _report = compare_mission_reference(
                        [verified_release],
                        [semantic_release],
                        max_position_error_m=self.config.mission.max_position_error_m,
                    )
                    if verify_errors:
                        raise RuntimeError("; ".join(verify_errors))
                    live_mission = verified
                    semantic_mission = verified
                    substitutions = []
                    raw_fingerprint = mission_fingerprint(verified)
                    self.pending_mission_fingerprint = raw_fingerprint
                    self.pending_mission_readbacks = stable_required
                    self.backup_path.unlink(missing_ok=True)
                elif self.config.release.enabled:
                    # A normal active route no longer needs a legacy backup.
                    self.backup_path.unlink(missing_ok=True)
                else:
                    raise RuntimeError(
                        "active-only runtime requires release.enabled=true; "
                        "flight-controller mission was left unchanged"
                    )
            except Exception as exc:
                contract = None
                contract_errors = [str(exc)]
                mission_mode = MISSION_MODE_BLOCKED
                operational_error = str(exc)
        else:
            # A stable new ordinary/malformed route proves that any previous
            # slot backup is stale. Removing it prevents sequence-number reuse
            # from ever modifying the new Mission Planner mission.
            self.backup_path.unlink(missing_ok=True)
            self.original_slots = []
            self.restore_pending = False

        semantic_fingerprint = mission_fingerprint(semantic_mission)
        if semantic_fingerprint != self.accepted_semantic_fingerprint:
            self._reset_for_new_mission()
        self.accepted_mission_fingerprint = raw_fingerprint
        self.accepted_semantic_fingerprint = semantic_fingerprint
        self.mission = semantic_mission
        self.contract = contract
        self.mission_assessment = MissionAssessment(
            mission_mode,
            contract,
            errors=tuple(contract_errors),
            warnings=tuple(contract_warnings),
        )
        self.scan_cells = (
            scan_coverage_cells(
                semantic_mission,
                contract,
                origin_lat_deg=self.home_lat_deg,
                origin_lon_deg=self.home_lon_deg,
            )
            if contract is not None and self.home_valid
            else []
        )
        self.scan_rectangle = (
            derive_scan_rectangle(
                semantic_mission,
                contract,
                origin_lat_deg=self.home_lat_deg,
                origin_lon_deg=self.home_lon_deg,
            )
            if contract is not None and self.home_valid
            else None
        )
        self.state["scan_rectangle"] = (
            asdict(self.scan_rectangle)
            if self.scan_rectangle is not None
            else None
        )
        self.state["mission_errors"] = contract_errors
        self.state["mission_mode"] = mission_mode
        self.state["mission_warnings"] = contract_warnings
        self.state["mission_reference"] = {
            **self.state.get("mission_reference", {}),
            "acceptance_mode": "dynamic",
            "file": None,
            "sha256": None,
            "expected_count": None,
            "matched": None,
            "fingerprint": semantic_fingerprint,
            "live_fingerprint": raw_fingerprint,
            "live_count": len(live_mission),
            "stable_readbacks": self.pending_mission_readbacks,
            "stable_readbacks_required": stable_required,
            "semantic_substitutions": substitutions,
            "live_release_command": (
                next(
                    (
                        item.command
                        for item in live_mission
                        if contract is not None
                        and item.seq
                        == getattr(contract, "fallback_release_seq", -1)
                    ),
                    None,
                )
            ),
            "contract": (
                asdict(contract) if contract is not None else None
            ),
            "last_checked_at": time.time(),
        }

        if operational_error is not None:
            self._set_status(
                "MISSION_INVALID",
                "Mission payload safety preparation failed: " + operational_error,
            )
            self._status_text(
                "ONBOARD MISSION INVALID - AUTO RELEASE BLOCKED",
                mavutil.mavlink.MAV_SEVERITY_CRITICAL,
            )
        elif mission_mode == MISSION_MODE_BLOCKED:
            self._set_status(
                "MISSION_INVALID",
                "Mission cannot complete automatic scan/drop/land task: "
                + "; ".join(contract_errors),
            )
            self._status_text(
                "Mission blocked - automatic task incomplete",
                mavutil.mavlink.MAV_SEVERITY_CRITICAL,
            )
        elif mission_mode == MISSION_MODE_FIXED_FALLBACK:
            self._set_status(
                "READY",
                "Mission accepted in fixed-fallback mode; dynamic target route is unavailable",
            )
            self._status_text(
                "Mission ready - fixed payload fallback",
                mavutil.mavlink.MAV_SEVERITY_WARNING,
            )
        else:
            if self.config.release.enabled and not self.servo_config_valid:
                self._set_status(
                    "PAYLOAD_SERVO_INVALID",
                    "Mission accepted; payload servo configuration is not verified",
                )
            else:
                self._set_status(
                    "READY",
                    "Dynamic mission accepted; onboard release mode is ACTIVE",
                )
                self._status_text("CUADC onboard ready (ACTIVE)")

    def _refresh_mission(self, force_stable: bool = False) -> None:
        self.last_mission_refresh = time.monotonic()
        if self.config.mission.acceptance_mode == "dynamic":
            self._refresh_dynamic_mission(force_stable=force_stable)
            return
        live_mission: list[MissionItem] = []
        semantic_mission: list[MissionItem] = []
        contract: MissionContract | None = None
        errors: list[str] = []
        reference_report: dict = {
            "matched": False if self.expected_mission else None,
            "actual_count": 0,
            "expected_count": len(self.expected_mission),
            "max_position_error_m": None,
        }
        substitutions: list[int] = []
        try:
            # Download first. Never restore a JSON backup into an unknown or
            # newly uploaded mission before proving that the backup belongs to
            # the currently hash-pinned reference.
            live_mission = self.client.download_mission(timeout=10.0)
            backup = load_patch_backup(self.backup_path)
            semantic_mission, backup_errors, substitutions = self._semantic_mission(
                live_mission, backup
            )
            errors.extend(backup_errors)
            contract, contract_errors = validate_mission(
                semantic_mission,
                payload_servo=self.config.release.servo_channel,
                min_release_pwm=float(self.config.release.release_pwm),
            )
            errors.extend(contract_errors)
            if self.expected_mission:
                reference_errors, reference_report = compare_mission_reference(
                    semantic_mission,
                    self.expected_mission,
                    max_position_error_m=self.config.mission.max_position_error_m,
                )
                errors.extend(reference_errors)

            if not errors and contract is not None:
                expected_by_seq = {
                    item.seq: item for item in (
                        self.expected_mission or semantic_mission
                    )
                }
                live_by_seq = {item.seq: item for item in live_mission}
                release_seq = contract.fallback_release_seq
                expected_release = expected_by_seq[release_seq]
                live_release = live_by_seq[release_seq]

                if self.config.release.enabled:
                    # Only an explicitly enabled, calibrated configuration may
                    # restore release-capable mission items, and only while
                    # disarmed. A hash-pinned reference makes this recovery
                    # independent of an old JSON file.
                    restore_items = list(backup)
                    if (
                        is_shadow_neutralization(live_release, expected_release)
                        and release_seq not in {item.seq for item in restore_items}
                    ):
                        restore_items.append(expected_release)
                    if restore_items and self.telemetry.armed:
                        errors.append(
                            "approved mission restore is pending while aircraft is armed"
                        )
                    elif restore_items:
                        restore_by_seq = {item.seq: item for item in restore_items}
                        start, end = min(restore_by_seq), max(restore_by_seq)
                        contiguous = [expected_by_seq[seq] for seq in range(start, end + 1)]
                        self.client.write_partial(contiguous)
                        verified = self.client.download_mission(timeout=10.0)
                        verify_errors, reference_report = compare_mission_reference(
                            verified,
                            self.expected_mission or semantic_mission,
                            max_position_error_m=self.config.mission.max_position_error_m,
                        )
                        if verify_errors:
                            errors.extend(
                                ["post-restore readback: " + item for item in verify_errors]
                            )
                        else:
                            live_mission = verified
                            semantic_mission = verified
                            substitutions = []
                            self.backup_path.unlink(missing_ok=True)
                            self.original_slots = []
                else:
                    errors.append(
                        "active-only runtime requires release.enabled=true; "
                        "flight-controller mission was left unchanged"
                    )
        except Exception as exc:
            errors.append(str(exc))

        if errors:
            contract = None
        self.mission = semantic_mission
        self.contract = contract
        self.state["mission_errors"] = errors
        self.state["mission_mode"] = (
            "INVALID" if errors else MISSION_MODE_PAYLOAD
        )
        self.state["mission_reference"] = {
            **self.state.get("mission_reference", {}),
            **reference_report,
            "matched": bool(self.expected_mission) and not errors,
            "live_count": len(live_mission),
            "semantic_substitutions": substitutions,
            "live_release_command": (
                next(
                    (
                        item.command
                        for item in live_mission
                        if contract is not None
                        and item.seq == contract.fallback_release_seq
                    ),
                    None,
                )
            ),
            "last_checked_at": time.time(),
        }
        if errors:
            self._set_status("MISSION_INVALID", "; ".join(errors))
            self._status_text(
                "ONBOARD MISSION INVALID - AUTO RELEASE BLOCKED",
                mavutil.mavlink.MAV_SEVERITY_CRITICAL,
            )
        else:
            if self.config.release.enabled and not self.servo_config_valid:
                self._set_status(
                    "PAYLOAD_SERVO_INVALID",
                    "Mission valid, but SERVO output function/range is not verified",
                )
            else:
                self._set_status(
                    "READY",
                    "Mission valid; onboard release mode is ACTIVE",
                )
                self._status_text("CUADC onboard ready (ACTIVE)")

    def _handle_home_position(self, message) -> None:
        latitude = float(message.latitude) / 1e7
        longitude = float(message.longitude) / 1e7
        if not (-90.0 <= latitude <= 90.0 and -180.0 <= longitude <= 180.0):
            return
        if latitude == 0.0 and longitude == 0.0:
            return
        self.home_lat_deg = latitude
        self.home_lon_deg = longitude
        self.home_alt_m = float(message.altitude) / 1000.0
        self.home_valid = True

    def _update_wind_estimate(self) -> None:
        telemetry = self.telemetry
        if (
            telemetry.airspeed_mps < 8.0
            or abs(math.degrees(telemetry.roll_rad)) > 15.0
        ):
            return
        air_north = telemetry.airspeed_mps * math.cos(telemetry.yaw_rad)
        air_east = telemetry.airspeed_mps * math.sin(telemetry.yaw_rad)
        measured_north = telemetry.velocity_north_mps - air_north
        measured_east = telemetry.velocity_east_mps - air_east
        if math.hypot(measured_north, measured_east) > 25.0:
            return
        alpha = 1.0 if self.wind_samples == 0 else 0.04
        self.wind_north_mps += alpha * (measured_north - self.wind_north_mps)
        self.wind_east_mps += alpha * (measured_east - self.wind_east_mps)
        self.wind_samples += 1

    def _detections(self) -> list[Detection]:
        snapshot = self.vision.snapshot()
        age = time.time() - float(snapshot.get("updated_at", 0.0))
        if age > self.config.vision.max_result_age_s:
            return []
        return [
            Detection(
                track_id=str(item["track_id"]),
                value=int(item["value"]),
                confidence=float(item["confidence"]),
                frames=int(item.get("frames", 1)),
                label=str(item.get("label", "")),
                north_m=float(item["north_m"]),
                east_m=float(item["east_m"]),
            )
            for item in snapshot.get("detections", [])
            if all(
                item.get(key) is not None
                for key in ("track_id", "value", "north_m", "east_m")
            )
        ]

    def _sample_scan_coverage(self) -> None:
        if self.contract is None or not self.home_valid:
            return
        if self.scan_rectangle is None:
            self.scan_rectangle = derive_scan_rectangle(
                self.mission,
                self.contract,
                origin_lat_deg=self.home_lat_deg,
                origin_lon_deg=self.home_lon_deg,
            )
            self.state["scan_rectangle"] = (
                asdict(self.scan_rectangle)
                if self.scan_rectangle is not None
                else None
            )
        if not self.scan_cells:
            self.scan_cells = scan_coverage_cells(
                self.mission,
                self.contract,
                origin_lat_deg=self.home_lat_deg,
                origin_lon_deg=self.home_lon_deg,
            )
        if not (
            self.contract.scan_start_seq
            < self.telemetry.mission_seq
            < self.contract.scan_end_seq
        ):
            return
        snapshot = self.vision.snapshot()
        updated_at = float(snapshot.get("updated_at", 0.0))
        camera_age_s = time.time() - updated_at if updated_at > 0.0 else math.inf
        self.scan_coverage_ratio = update_scan_coverage(
            self.scan_cells,
            self.covered_scan_cells,
            self.telemetry.north_m,
            self.telemetry.east_m,
            self.telemetry.relative_alt_m,
            camera_age_s,
        )
        self.state["scan_coverage_ratio"] = self.scan_coverage_ratio

    def _scan_is_complete(self) -> bool:
        return (
            self.scan_coverage_ratio >= SCAN_COVERAGE_THRESHOLD
            and self.stable_target_count >= MIN_STABLE_TARGETS
        )

    def _nominal_release_solution(
        self,
        target_north_m: float,
        target_east_m: float,
        decision_north_m: float,
        decision_east_m: float,
    ) -> ReleaseSolution:
        direction_n = target_north_m - decision_north_m
        direction_e = target_east_m - decision_east_m
        length = math.hypot(direction_n, direction_e)
        if length < 1.0:
            raise ValueError("decision guard and selected target are too close")
        direction_n /= length
        direction_e /= length
        along_wind = (
            self.wind_north_mps * direction_n
            + self.wind_east_mps * direction_e
        )
        nominal_groundspeed = max(
            8.0,
            self.config.release.nominal_airspeed_mps + along_wind,
        )
        velocity_n = direction_n * nominal_groundspeed
        velocity_e = direction_e * nominal_groundspeed
        solution = solve_release(
            target_north_m,
            target_east_m,
            self.config.release.nominal_agl_m,
            velocity_n,
            velocity_e,
            self.wind_north_mps,
            self.wind_east_mps,
            drag_rate_s=self.config.release.drag_rate_s,
        )
        return solution_with_actuator_delay(
            solution,
            velocity_n,
            velocity_e,
            self.config.release.actuator_delay_s,
        )

    def _handle_scan_end(self) -> None:
        if self.contract is None:
            self._set_status(
                "FALLBACK_ROUTE_ACTIVE",
                "No valid mission contract; original Mission Planner fallback retained",
            )
            return
        detections = self._detections()
        self.stable_target_count = len(
            [item for item in detections if item.frames >= 2]
        )
        self.state["stable_target_count"] = self.stable_target_count
        complete = self._scan_is_complete()
        if not complete:
            self.selected = None
            self.state["selected_target"] = None
            self.scan_fallback_reason = self.scan_fallback_reason or (
                f"scan incomplete after {self.scan_attempt} attempts: "
                f"coverage={self.scan_coverage_ratio:.0%}, "
                f"stable_targets={self.stable_target_count}"
            )
            self.state["scan_attempt"] = self.scan_attempt
            self.state["scan_fallback_reason"] = self.scan_fallback_reason
            self._set_status(
                "FALLBACK_ROUTE_ACTIVE",
                self.scan_fallback_reason
                + ("; precision mode withholds payload release" if self.config.release.precision_required else "; original Mission Planner payload route retained"),
            )
            self._status_text(
                "Recon incomplete - payload withheld" if self.config.release.precision_required else "Recon incomplete - fixed payload fallback active",
                mavutil.mavlink.MAV_SEVERITY_WARNING,
            )
            return
        if not self.home_valid:
            self._set_status(
                "FALLBACK_ROUTE_ACTIVE",
                "HOME_POSITION unavailable; fixed payload fallback retained",
            )
            return
        selection = select_target(self.config.vision.task, detections)
        if selection.track_id is None:
            self.selected = None
            self.state["selected_target"] = None
            self._set_status(
                "FALLBACK_ROUTE_ACTIVE",
                "No stable YOLO target; original Mission Planner fallback retained",
            )
            self._status_text(
                "No stable YOLO target - mission fallback retained",
                mavutil.mavlink.MAV_SEVERITY_WARNING,
            )
            return
        assert selection.north_m is not None and selection.east_m is not None
        if (
            math.hypot(selection.north_m, selection.east_m)
            > self.config.mission.max_home_distance_m
        ):
            self.selected = None
            self._set_status(
                "FALLBACK_ROUTE_ACTIVE",
                "YOLO target is outside the home boundary; original fallback retained",
            )
            return
        self.selected = selection
        self.state["selected_target"] = asdict(selection)
        if isinstance(self.contract, FixedFallbackContract):
            self._set_status(
                "FALLBACK_ROUTE_ACTIVE",
                "Target selected and logged; dynamic slots unavailable, fixed payload route retained",
            )
            self.state["scan_fallback_reason"] = "dynamic mission slots unavailable"
            self._status_text("Target logged - fixed payload fallback active")
            return
        if isinstance(self.contract, ReconContract):
            self._set_status(
                "RECON_SELECTION",
                "Recon-only target selected and logged; route and payload remain unchanged",
            )
            return
        if not self.config.release.enabled:
            self._set_status(
                "MISSION_INVALID",
                "Active-only runtime refused target routing because release is disabled",
            )
            return
        if self._sync_flight_controller_health_state():
            self._set_status(
                "FALLBACK_ROUTE_ACTIVE",
                "Dynamic attack/release route blocked by flight-controller health fault",
            )
            return
        by_seq = {item.seq: item for item in self.mission}
        decision = by_seq[self.contract.decision_seq]
        decision_north, decision_east = latlon_to_ne(
            decision.x,
            decision.y,
            self.home_lat_deg,
            self.home_lon_deg,
        )
        scan_points = [
            latlon_to_ne(
                item.x,
                item.y,
                self.home_lat_deg,
                self.home_lon_deg,
            )
            for item in self.mission
            if (
                self.contract.scan_start_seq < item.seq < self.contract.scan_end_seq
                and item.command == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
                and not (item.x == 0.0 and item.y == 0.0)
            )
        ]
        if len(scan_points) >= 2:
            outbound_north = scan_points[-1][0] - scan_points[0][0]
            outbound_east = scan_points[-1][1] - scan_points[0][1]
        else:
            outbound_north = outbound_east = None
        try:
            solution = self._nominal_release_solution(
                selection.north_m,
                selection.east_m,
                decision_north,
                decision_east,
            )
            patch = build_attack_patch(
                self.mission,
                self.contract,
                selection.north_m,
                selection.east_m,
                decision_north,
                decision_east,
                nominal_speed_mps=self.config.mission.planning_airspeed_mps,
                nominal_agl_m=self.config.release.nominal_agl_m,
                max_home_distance_m=self.config.mission.max_home_distance_m-self.config.mission.tracking_margin_m,
                planning_bank_deg=self.config.mission.planning_bank_deg,
                origin_lat_deg=self.home_lat_deg,
                origin_lon_deg=self.home_lon_deg,
                payload_servo=self.config.release.servo_channel,
                release_pwm=self.config.release.release_pwm,
                outbound_north=outbound_north,
                outbound_east=outbound_east,
                attack_altitude_limit_m=None,
            )
            if self.config.release.precision_required:
                patch=[neutralize_release_item(item) if item.seq==self.contract.fallback_release_seq else item for item in patch]
            originals = [by_seq[item.seq] for item in patch]
            save_patch_backup(self.backup_path, originals)
            self.client.write_partial(patch)
            # ArduPlane does not reinitialise a mission item replaced while it
            # is already active. Scan-end is exactly that boundary, so restart
            # the first patched slot after the write is acknowledged.
            self.client.set_current(self.contract.fallback_start_seq)
            replacements = {item.seq: item for item in patch}
            self.mission = [replacements.get(item.seq, item) for item in self.mission]
            self.original_slots = originals
            self.restore_pending = True
            self.state["nominal_release_solution"] = asdict(solution)
            self._set_status(
                "ATTACK_ROUTE_ACTIVE",
                "Dynamic route installed; precision-gated primary release only" if self.config.release.precision_required else "Dynamic target route installed; primary and mission fallback release armed",
            )
            self._status_text("Dynamic target route installed")
        except Exception as exc:
            # Keep the pre-write backup. A serial interruption can leave a
            # partial update on the flight controller even when write_partial
            # raises, so deleting recovery data here would be unsafe.
            self.restore_pending = self.backup_path.exists()
            self._set_status(
                "FALLBACK_ROUTE_ACTIVE",
                f"Dynamic patch failed; original Mission Planner fallback retained: {exc}",
            )
            self._status_text(
                "Route patch failed - original fallback retained",
                mavutil.mavlink.MAV_SEVERITY_WARNING,
            )

    def _actual_release_solution(
        self,
        position_age_s: float = 0.0,
    ) -> ReleaseSolution | None:
        if self.selected is None:
            return None
        assert self.selected.north_m is not None and self.selected.east_m is not None
        telemetry = self.telemetry
        solution = solve_release(
            self.selected.north_m,
            self.selected.east_m,
            max(0.2, telemetry.relative_alt_m),
            telemetry.velocity_north_mps,
            telemetry.velocity_east_mps,
            self.wind_north_mps,
            self.wind_east_mps,
            telemetry.velocity_down_mps,
            drag_rate_s=self.config.release.drag_rate_s,
        )
        return solution_with_actuator_delay(
            solution,
            telemetry.velocity_north_mps,
            telemetry.velocity_east_mps,
            self.config.release.actuator_delay_s + max(0.0, position_age_s),
        )

    def _try_primary_release(self) -> None:
        if (
            not isinstance(self.contract, MissionContract)
            or self.release_commanded
            or self.release_observed
            or self.selected is None
        ):
            return
        assert self.selected.north_m is not None and self.selected.east_m is not None
        now = time.monotonic()
        health_fault_active = self._sync_flight_controller_health_state(now)
        precision=None
        if self.config.release.precision_required and self.telemetry.relative_alt_m>0:
            precision=estimate_impact_error(self.telemetry.relative_alt_m,self.telemetry.airspeed_mps,
                                            self.config.release.actuator_delay_s,self.config.release.drag_rate_s,
                                            PrecisionBudget.from_release(self.config.release))
            self.state['precision_budget']=precision
        reasons = release_gate_reasons(
            self.config,
            self.telemetry,
            self.contract,
            now - self.last_heartbeat,
            now - self.last_position,
            now - self.last_attitude,
            now - self.last_airspeed,
            self.gps_fix_type,
            self.satellites_visible,
            self.gps_hdop,
            self.servo_config_valid,
            self.home_valid,
            self.selected.north_m,
            self.selected.east_m,
            not health_fault_active,
        )
        self.state["release_gate"] = reasons
        if reasons:
            return
        position_age_s = max(0.0, now - self.last_position)
        solution = self._actual_release_solution(position_age_s)
        if solution is None:
            return
        telemetry = self.telemetry
        along = along_track_error(
            telemetry.north_m,
            telemetry.east_m,
            solution.release_north_m,
            solution.release_east_m,
            telemetry.velocity_north_mps,
            telemetry.velocity_east_mps,
        )
        distance = math.hypot(
            solution.release_north_m - telemetry.north_m,
            solution.release_east_m - telemetry.east_m,
        )
        crossed = crossed_release_line(self.previous_along_track, along)
        self.previous_along_track = along
        cross_track = math.sqrt(max(0.0, distance * distance - along * along))
        groundspeed = math.hypot(
            telemetry.velocity_north_mps,
            telemetry.velocity_east_mps,
        )
        # The release solution already moves the command point upstream by
        # the measured actuator and telemetry delay.  Keeping a multi-metre
        # direct window would apply that delay a second time and create a
        # repeatable down-track bias.  Use the configured competition
        # tolerance for the direct gate; a packet that straddles the line is
        # still accepted below when its crossing remains within the same
        # tolerance.
        direct_window = self.config.release.target_tolerance_m
        if precision is not None:
            margin=self.config.release.target_tolerance_m-precision['r95_m']
            if margin<=0 or distance>margin:
                return
        elif distance > direct_window and not (
            crossed
            and along >= -direct_window
            and cross_track <= self.config.release.max_cross_track_m
        ):
            return
        self.client.set_servo(
            self.config.release.servo_channel,
            self.config.release.release_pwm,
        )
        self.release_commanded = True
        self.state["release_commanded"] = True
        self.state["release"] = {
            "source": "PRIMARY_DYNAMIC_BALLISTIC",
            "commanded_at": time.time(),
            "aircraft_north_m": telemetry.north_m,
            "aircraft_east_m": telemetry.east_m,
            "relative_alt_m": telemetry.relative_alt_m,
            "airspeed_mps": telemetry.airspeed_mps,
            "position_age_s": position_age_s,
            "effective_command_delay_s": (
                self.config.release.actuator_delay_s + position_age_s
            ),
            "solution": asdict(solution),
        }
        self._set_status(
            "RELEASE_COMMAND_SENT",
            f"DO_SET_SERVO channel {self.config.release.servo_channel} PWM "
            f"{self.config.release.release_pwm} sent",
        )
        self._status_text("Primary ballistic payload release commanded")

    def _observe_servo_release(self) -> None:
        channel_index = self.config.release.servo_channel - 1
        pwm = self.telemetry.servo_pwm[channel_index]
        if (
            self.release_observed
            or not self.telemetry.armed
            or pwm < self.config.release.release_pwm - 20
        ):
            return
        self.release_observed = True
        self.release_commanded = True
        self.state["release_commanded"] = True
        self.state["release_observed"] = True
        release = dict(self.state.get("release") or {})
        release.setdefault("source", "MISSION_DO_SET_SERVO_FALLBACK")
        release["servo_output_observed_at"] = time.time()
        release["observed_pwm"] = pwm
        release["mechanical_confirmation"] = False
        self.state["release"] = release
        self._set_status(
            "RELEASE_OUTPUT_OBSERVED",
            "Flight-controller PWM output observed; mechanical separation is not sensor-confirmed",
        )

    def _restore_original_slots(self) -> None:
        if not self.config.release.enabled:
            # Disabled is a blocked configuration in the active-only runtime.
            # Never write mission EEPROM from that state.
            return
        if not self.restore_pending and not self.backup_path.exists():
            return
        if not (self.original_slots or load_patch_backup(self.backup_path)):
            self.restore_pending = False
            return
        try:
            # The normal refresh path downloads first, proves the backup
            # against the hash-pinned mission and verifies a second readback.
            # Keeping restoration in that path prevents a stale seq-numbered
            # JSON file from ever being written blindly after a new upload.
            self._refresh_mission(force_stable=True)
        except Exception as exc:
            self._set_status(
                "MISSION_RESTORE_FAILED",
                f"Re-upload the original mission before another flight: {exc}",
            )
            return
        if self.backup_path.exists():
            self._set_status(
                "MISSION_RESTORE_FAILED",
                "Approved mission restore did not pass readback verification",
            )
            return
        self.original_slots = []
        self.restore_pending = False
        self._set_status(
            "FLIGHT_COMPLETE",
            "Original Mission Planner fallback slots restored and verified",
        )

    def _handle_message(self, message) -> None:
        now = time.monotonic()
        message_type = message.get_type()
        # The router can carry Mission Planner and companion traffic as well
        # as the vehicle stream.  Only the selected autopilot may update
        # armed/AUTO/navigation state or drive mission/release decisions.
        if not self._message_is_from_target_autopilot(message):
            return
        if message_type == "STATUSTEXT":
            self._handle_flight_controller_status_text(message)
            return
        retry_entry=None
        scan_end=False
        with self.telemetry_lock:
            if message_type == "HEARTBEAT":
                self.last_heartbeat = now
                self.telemetry.armed = self.client.is_armed(message)
                self.telemetry.auto = self.client.is_auto(message)
                if self.telemetry.armed:
                    self.ever_armed = True
            elif message_type == "HOME_POSITION":
                self._handle_home_position(message)
            elif message_type == "GPS_RAW_INT":
                self.gps_fix_type = int(message.fix_type)
                self.satellites_visible = int(message.satellites_visible)
                raw_eph = int(message.eph)
                self.gps_hdop = (
                    math.inf if raw_eph == 65535 else raw_eph / 100.0
                )
            elif message_type == "GLOBAL_POSITION_INT":
                origin_lat = self.home_lat_deg if self.home_valid else float(message.lat) / 1e7
                origin_lon = self.home_lon_deg if self.home_valid else float(message.lon) / 1e7
                self.telemetry.update_position(message, origin_lat, origin_lon)
                self.last_position = now
            elif message_type == "ATTITUDE":
                self.telemetry.update_attitude(message)
                self.last_attitude = now
            elif message_type == "VFR_HUD":
                self.telemetry.update_hud(message)
                self.last_airspeed = now
            elif message_type == "SERVO_OUTPUT_RAW":
                self.telemetry.update_servos(message)
            elif message_type == "MISSION_CURRENT":
                sequence = int(message.seq)
                self.telemetry.mission_seq = sequence
                if self.retry_turn_active and isinstance(self.contract,MissionContract):
                    if sequence<self.contract.scan_start_seq:
                        self.last_end_seq_processed=sequence
                        self.retry_turn_active=False
                    elif sequence>self.contract.fallback_start_seq and not self.retry_return_commanded:
                        retry_entry=retry_entry_seq(self.mission,self.contract)
                        self.last_end_seq_processed=retry_entry
                        self.retry_return_commanded=True
                elif (
                    self.contract
                    and crossed_scan_end(
                        self.last_end_seq_processed,
                        sequence,
                        self.contract.scan_end_seq,
                    )
                ):
                    self.last_end_seq_processed = sequence
                    # Mission I/O and model selection happen after releasing
                    # the telemetry lock to keep camera snapshots responsive.
                    scan_end = True
                else:
                    scan_end = False
                    if (
                        self.contract
                        and sequence < self.contract.scan_end_seq
                        and sequence < self.last_end_seq_processed
                    ):
                        # Retry starts at the first waypoint after JUMP_TAG
                        # 9101.  Rearm the 9102 crossing on that backward
                        # mission jump so the second attempt is evaluated.
                        self.last_end_seq_processed = sequence
            else:
                scan_end = False
        if retry_entry is not None:
            # Mission I/O stays outside the telemetry lock used by the camera.
            self.client.set_current(retry_entry)
        if message_type == "MISSION_CURRENT" and scan_end:
            self._handle_scan_end()
        if message_type in {"GLOBAL_POSITION_INT", "VFR_HUD", "ATTITUDE"}:
            self._update_wind_estimate()
        if message_type == "GLOBAL_POSITION_INT":
            self._sample_scan_coverage()

    def _write_status(self, force: bool = False) -> None:
        now = time.monotonic()
        period = 1.0 / self.config.runtime.status_hz
        if not force and now - self.last_status_write < period:
            return
        self._sync_flight_controller_health_state(now)
        vision = self.vision.snapshot()
        payload = {
            **self.state,
            "updated_at": time.time(),
            "home": {
                "valid": self.home_valid,
                "lat_deg": self.home_lat_deg,
                "lon_deg": self.home_lon_deg,
                "alt_m": self.home_alt_m,
            },
            "gps": {
                "fix_type": self.gps_fix_type,
                "satellites_visible": self.satellites_visible,
                "hdop": self.gps_hdop if math.isfinite(self.gps_hdop) else None,
            },
            "wind_estimate": {
                "north_mps": self.wind_north_mps,
                "east_mps": self.wind_east_mps,
                "samples": self.wind_samples,
                "method": "ground velocity minus airspeed/yaw approximation",
            },
            "payload_servo": self.servo_configuration,
            "telemetry": self.telemetry_snapshot(),
            "vision": {
                "source": vision.get("source"),
                "message": vision.get("message"),
                "model_path": vision.get("model_path"),
                "camera_connected": vision.get("camera_connected", False),
                "camera_auto": vision.get("camera_auto", False),
                "camera_configured_device": vision.get(
                    "camera_configured_device"
                ),
                "camera_device": vision.get("camera_device"),
                "camera_resolved_device": vision.get(
                    "camera_resolved_device"
                ),
                "camera_identity": vision.get("camera_identity"),
                "camera_width": vision.get("camera_width"),
                "camera_height": vision.get("camera_height"),
                "camera_fps": vision.get("camera_fps"),
                "camera_pixel_format": vision.get("camera_pixel_format"),
                "frame_count": vision.get("frame_count", 0),
                "inference_count": vision.get("inference_count", 0),
                "updated_at": vision.get("updated_at", 0.0),
                "detections": vision.get("detections", []),
            },
        }
        self._publish_mavlink_report(payload, force=force)
        atomic_write_json(self.runtime_dir / "state.json", payload)
        telemetry_log = self.runtime_dir / "telemetry.jsonl"
        rotate_file(
            telemetry_log,
            self.config.runtime.telemetry_log_max_mb * 1024 * 1024,
            self.config.runtime.telemetry_log_backups,
        )
        with telemetry_log.open(
            "a", encoding="utf-8"
        ) as stream:
            stream.write(json.dumps(payload, ensure_ascii=False) + "\n")
        self.last_status_write = now

    def _publish_mavlink_report(self, payload: dict, force: bool = False) -> None:
        runtime = self.config.runtime
        if not runtime.mavlink_report_enabled:
            return
        now = time.monotonic()
        if (
            not force
            and now - self.last_mavlink_report < 1.0 / runtime.mavlink_report_hz
        ):
            return
        time_boot_ms = int(now * 1000.0) & 0xFFFFFFFF
        selected = payload.get("selected_target") or {}
        vision_payload = payload.get("vision", {})
        detections = vision_payload.get("detections", []) or []
        best_detection = max(
            detections,
            key=lambda item: float(item.get("confidence", 0.0)),
            default={},
        )
        selected_value = selected.get("value")
        mission_mode = str(payload.get("mission_mode", "UNKNOWN"))
        mission_mode_code = {
            MISSION_MODE_PASSTHROUGH: 1,
            MISSION_MODE_PAYLOAD: 2,
            MISSION_MODE_DYNAMIC_READY: 2,
            MISSION_MODE_CONTRACT_WARNING: 3,
            MISSION_MODE_FIXED_FALLBACK: 3,
            MISSION_MODE_RECON_ONLY: 4,
        }.get(mission_mode, 0)
        mission_passed = int(mission_mode in {
            MISSION_MODE_DYNAMIC_READY,
            MISSION_MODE_FIXED_FALLBACK,
            MISSION_MODE_PAYLOAD,
        })
        contract = self.contract
        release_seq = (
            -1
            if contract is None
            else int(getattr(contract, "fallback_release_seq", -1))
        )
        integers = {
            "CU_STATUS": ONBOARD_STATUS_CODES.get(str(payload.get("status")), 99),
            "CU_TASK": int(self.config.vision.task),
            "CU_DETCNT": len(detections),
            "CU_DETVAL": int(best_detection.get("value", -1)),
            "CU_SELVAL": int(-1 if selected_value is None else selected_value),
            "CU_REL_EN": int(bool(self.config.release.enabled)),
            "CU_REL_CMD": int(bool(self.release_commanded)),
            "CU_REL_OBS": int(bool(self.release_observed)),
            "CU_GPSFIX": int(self.gps_fix_type),
            "CU_SATS": int(self.satellites_visible),
            "CU_MSEQ": int(self.telemetry.mission_seq),
            "CU_CAMOK": int(bool(vision_payload.get("camera_connected"))),
            "CU_CAMAUTO": int(bool(vision_payload.get("camera_auto"))),
            "CU_CAMW": int(vision_payload.get("camera_width") or 0),
            "CU_CAMH": int(vision_payload.get("camera_height") or 0),
            "CU_CAMFPS": int(round(float(vision_payload.get("camera_fps") or 0))),
            "CU_MPASS": mission_passed,
            "CU_MMODE": mission_mode_code,
            "CU_SCANST": -1 if contract is None else contract.scan_start_seq,
            "CU_SCANEN": -1 if contract is None else contract.scan_end_seq,
            "CU_RELSEQ": release_seq,
            "CU_LANDSEQ": -1 if contract is None else contract.land_seq,
            "CU_SCANTRY": int(self.scan_attempt),
            "CU_STABLE": int(self.stable_target_count),
        }
        floats = {
            "CU_DETCF": float(best_detection.get("confidence", -1.0)),
            "CU_DETN": float(best_detection.get("north_m", 0.0)),
            "CU_DETE": float(best_detection.get("east_m", 0.0)),
            "CU_SELN": float(selected.get("north_m", 0.0) or 0.0),
            "CU_SELE": float(selected.get("east_m", 0.0) or 0.0),
            "CU_ALT": float(self.telemetry.relative_alt_m),
            "CU_SCANCOV": float(self.scan_coverage_ratio),
            "CU_SCANLEN": float(
                0.0
                if self.scan_rectangle is None
                else self.scan_rectangle.length_m
            ),
            "CU_SCANWID": float(
                0.0
                if self.scan_rectangle is None
                else self.scan_rectangle.width_m
            ),
            "CU_AIRSPD": float(self.telemetry.airspeed_mps),
            "CU_HDOP": float(self.gps_hdop if math.isfinite(self.gps_hdop) else -1.0),
            "CU_INF": float(payload.get("vision", {}).get("inference_count", 0)),
        }
        try:
            self.client.companion_heartbeat(integers["CU_STATUS"])
            for name, value in integers.items():
                self.client.named_value_int(name, value, time_boot_ms)
            for name, value in floats.items():
                self.client.named_value_float(name, value, time_boot_ms)
            self.state["mavlink_report"] = {
                "enabled": True,
                "last_sent_at": time.time(),
                "integer_fields": integers,
                "float_fields": floats,
            }
            self.last_mavlink_report = now
        except Exception as exc:
            self.state["mavlink_report"] = {
                "enabled": True,
                "error": f"{type(exc).__name__}: {exc}",
            }

    def shutdown(self, *_args) -> None:
        self.stop_requested = True
        # Signal the camera worker immediately instead of waiting for the main
        # loop to finish a bounded MAVLink mission transaction and enter its
        # finally block.  This lets an idle V4L2 reader release promptly while
        # systemd waits for the transaction to finish safely.
        self.vision.stop()

    def run(self) -> int:
        self.initialize()
        self.watchdog.ready(
            f"Onboard task {self.config.vision.task} running in "
            f"{'ACTIVE' if self.config.release.enabled else 'RELEASE-DISABLED'} mode"
        )
        try:
            while not self.stop_requested:
                message = self.client.recv(
                    [
                        "HEARTBEAT",
                        "HOME_POSITION",
                        "GPS_RAW_INT",
                        "GLOBAL_POSITION_INT",
                        "ATTITUDE",
                        "VFR_HUD",
                        "SERVO_OUTPUT_RAW",
                        "MISSION_CURRENT",
                        "STATUSTEXT",
                    ],
                    timeout=0.01 if self.config.release.precision_required else 0.1,
                )
                if message is not None:
                    self._handle_message(message)
                if self.vision.error and self._nominal_status not in {
                    "VISION_FAILED",
                    "MISSION_INVALID",
                }:
                    self._set_status("VISION_FAILED", self.vision.error)
                    self._status_text(
                        "USB camera or YOLO failed - mission fallback only",
                        mavutil.mavlink.MAV_SEVERITY_CRITICAL,
                    )
                elif (
                    not self.vision.error
                    and self._nominal_status == "VISION_FAILED"
                ):
                    self._set_status(
                        "READY" if self.home_valid else "WAITING_FOR_NAVIGATION",
                        "USB camera recovered; vision processing resumed",
                    )
                    self._status_text("USB camera recovered")
                self._try_primary_release()
                self._observe_servo_release()
                now = time.monotonic()
                if not self.home_valid and now - self.last_home_request >= 2.0:
                    self._request_home_position()
                self._retry_servo_configuration()
                if (
                    not self.telemetry.armed
                    and not self.ever_armed
                    and now - self.last_mission_refresh
                    >= self.config.flight_controller.mission_refresh_s
                ):
                    self._refresh_mission()
                if self.ever_armed and not self.telemetry.armed:
                    self._restore_original_slots()
                    self.ever_armed = False
                self._write_status()
                self.watchdog.ping()
            return 0
        finally:
            self.watchdog.stopping()
            self.vision.stop()
            if self.vision.is_alive():
                self.vision.join(timeout=3.0)
            self._write_status(force=True)


def main(config: OnboardConfig) -> int:
    runtime = OnboardRuntime(config)
    signal.signal(signal.SIGINT, runtime.shutdown)
    signal.signal(signal.SIGTERM, runtime.shutdown)
    try:
        return runtime.run()
    except KeyboardInterrupt:
        return 130
    except (MavlinkTimeout, RuntimeError, ValueError) as exc:
        runtime._set_status("FATAL", str(exc))
        runtime._write_status(force=True)
        print(f"CUADC onboard fatal: {exc}", flush=True)
        return 2
