from __future__ import annotations

from dataclasses import dataclass, fields
import hashlib
import math
import os
from pathlib import Path
from typing import Any, TypeVar

import yaml


def _discover_project_root() -> Path:
    def is_project_root(candidate: Path) -> bool:
        return all(
            (
                (candidate / "pyproject.toml").is_file(),
                (candidate / "config/onboard.yaml").is_file(),
                (candidate / "assets/targets/catalog.json").is_file(),
            )
        )

    configured = os.environ.get("CUADC_ONBOARD_ROOT") or os.environ.get(
        "CUADC_PROJECT_ROOT"
    )
    if configured:
        candidate = Path(configured).expanduser().resolve()
        if is_project_root(candidate):
            return candidate

    checked: set[Path] = set()
    for start in (Path.cwd().resolve(), Path(__file__).resolve().parent):
        for candidate in (start, *start.parents):
            if candidate in checked:
                continue
            checked.add(candidate)
            if is_project_root(candidate):
                return candidate
    raise RuntimeError(
        "CUADC onboard project root not found; run inside the onboard project "
        "or set CUADC_ONBOARD_ROOT"
    )


ROOT = _discover_project_root()
RUNTIME_DIR = ROOT / ".runtime/onboard"


def default_config_path() -> Path:
    configured = os.environ.get("CUADC_ONBOARD_CONFIG")
    if configured:
        return Path(configured).expanduser()
    local = ROOT / "config/onboard.local.yaml"
    return local if local.is_file() else ROOT / "config/onboard.yaml"


DEFAULT_CONFIG = default_config_path()

SCAN_START_TAG = 9101
SCAN_END_TAG = 9102
PAYLOAD_SERVO = 8
PAYLOAD_RELEASE_PWM = 1800
COMPANION_UDP_PORT = 14551
# The real-aircraft runtime reads the servo parameters it needs explicitly.
PARAM_NAMES: tuple[str, ...] = ()
# Public Canberra test coordinates are only safe fallbacks; HOME_POSITION is
# used as the live origin as soon as the flight controller provides it.
ORIGIN_LAT_DEG = -35.363261
ORIGIN_LON_DEG = 149.165230
DOWN_CAMERA_HORIZONTAL_FOV_RAD = math.radians(90.0)
CAMERA_TELEMETRY_ALIGNMENT_S = 0.0


@dataclass(frozen=True)
class FlightControllerConfig:
    # The companion never opens the flight-controller serial device directly.
    # A single router process owns serial and exposes this loopback endpoint.
    endpoint: str = "udpin:127.0.0.1:14551"
    baud: int = 115200
    heartbeat_timeout_s: float = 30.0
    telemetry_timeout_s: float = 1.5
    mission_refresh_s: float = 5.0


@dataclass(frozen=True)
class TelemetryRouterConfig:
    serial_device: str = "/dev/cuadc-flight-controller"
    baud: int = 115200
    serial_reconnect_s: float = 2.0
    companion_host: str = "127.0.0.1"
    companion_port: int = 14551
    router_bind_host: str = "127.0.0.1"
    router_local_port: int = 14560
    # ArduPlane 4.6.2 can hard-fault if command 401 reaches it before GPS
    # initialisation.  The companion does not need this command because the
    # flight controller runs its own pre-arm checks periodically.
    block_run_prearm_checks: bool = True
    status_interval_s: float = 1.0
    log_tlog: bool = True
    tlog_max_mb: int = 256
    tlog_retained_files: int = 5


@dataclass(frozen=True)
class CameraConfig:
    device: str = "auto"
    width: int = 1280
    height: int = 720
    fps: int = 30
    horizontal_fov_deg: float = 90.0
    fx_px: float = 0.0
    fy_px: float = 0.0
    cx_px: float = 0.0
    cy_px: float = 0.0
    distortion: tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0)
    mount_roll_deg: float = 0.0
    mount_pitch_deg: float = 0.0
    mount_yaw_deg: float = 0.0
    telemetry_alignment_s: float = 0.0
    agl_offset_m: float = 0.0
    save_preview: bool = True
    calibration_confirmed: bool = False
    reconnect_s: float = 2.0


@dataclass(frozen=True)
class VisionConfig:
    task: int = 1
    confidence: float = 0.20
    inference_hz: float = 2.0
    inference_size_task1: int = 640
    inference_size_task2: int = 960
    angles: tuple[int, ...] = (0, 90, 180, 270)
    device: str = "cpu"
    task1_weights: str = "models/vision/task1_v4m_best.pt"
    task2_weights: str = "models/vision/task2_sim_digits_v1.pt"
    max_result_age_s: float = 2.0
    validation_confirmed: bool = False

    def inference_size(self) -> int:
        return (
            self.inference_size_task1
            if self.task == 1
            else self.inference_size_task2
        )

    def weights_path(self, root: Path = ROOT) -> Path:
        configured = (
            self.task1_weights if self.task == 1 else self.task2_weights
        )
        candidate = Path(configured).expanduser()
        if not candidate.is_absolute():
            candidate = root / candidate
        return candidate.resolve()


@dataclass(frozen=True)
class ReleaseConfig:
    # Missing/false remains a fail-closed value. The active-only runtime and
    # mission installer reject it instead of rewriting the mission to a no-op.
    enabled: bool = False
    servo_channel: int = 8
    release_pwm: int = 1800
    # Default to an independent output. The device profile may explicitly use
    # 58 on output 8 for RC8 manual loading checks plus MAVLink release.
    # ArduPlane holds DO_SET_SERVO until RC input moves beyond its dead zone;
    # this is not an interlock against manual or receiver-failsafe movement.
    required_servo_function: int = 0
    actuator_brand: str = "PTK"
    actuator_supply_v: float = 5.0
    payload_water_ml: float = 350.0
    payload_total_mass_kg: float = 0.370
    actuator_delay_s: float = 0.12
    drag_rate_s: float = 0.095
    nominal_agl_m: float = 20.0
    nominal_airspeed_mps: float = 20.0
    min_agl_m: float = 12.0
    max_agl_m: float = 35.0
    min_airspeed_mps: float = 12.0
    max_airspeed_mps: float = 32.0
    max_roll_deg: float = 20.0
    max_yaw_rate_dps: float = 12.0
    max_cross_track_m: float = 6.0
    command_window_s: float = 0.25
    ballistics_calibrated: bool = False
    precision_required: bool = False
    precision_calibrated: bool = False
    target_tolerance_m: float = .35
    gps_relative_sigma_m: float = 3.0
    camera_position_sigma_m: float = .3
    actuator_delay_sigma_s: float = .03
    altitude_sigma_m: float = .5
    velocity_sigma_mps: float = .3
    wind_sigma_mps: float = .5
    drag_sigma_s: float = .02


@dataclass(frozen=True)
class MissionConfig:
    max_home_distance_m: float = 300.0
    tracking_margin_m: float = 0.0
    planning_bank_deg: float = 30.0
    planning_airspeed_mps: float = 15.0
    require_auto: bool = True
    require_3d_gps: bool = True
    min_satellites: int = 6
    max_hdop: float = 2.5
    # ``dynamic`` accepts the live Mission Planner route and only enables the
    # payload executive when the 9101/9102 contract is present. ``pinned`` is
    # retained for audited flights that deliberately lock one exact route.
    acceptance_mode: str = "dynamic"
    stable_readbacks: int = 2
    # Pin the exact Mission Planner mission approved for this aircraft. The
    # runtime still downloads the live flight-controller mission, but refuses
    # READY if its route differs from this file beyond normal ArduPilot
    # coordinate/parameter normalization.
    expected_file: str = ""
    expected_sha256: str = ""
    max_position_error_m: float = 2.0

    def expected_path(self, root: Path = ROOT) -> Path | None:
        if not self.expected_file:
            return None
        candidate = Path(self.expected_file).expanduser()
        if not candidate.is_absolute():
            candidate = root / candidate
        return candidate.resolve()


@dataclass(frozen=True)
class RuntimeConfig:
    directory: str = ".runtime/onboard"
    status_hz: float = 2.0
    mavlink_report_enabled: bool = True
    mavlink_report_hz: float = 2.0
    telemetry_log_max_mb: int = 100
    telemetry_log_backups: int = 3

    def path(self, root: Path = ROOT) -> Path:
        candidate = Path(self.directory).expanduser()
        if not candidate.is_absolute():
            candidate = root / candidate
        return candidate.resolve()


@dataclass(frozen=True)
class OnboardConfig:
    flight_controller: FlightControllerConfig = FlightControllerConfig()
    telemetry_router: TelemetryRouterConfig = TelemetryRouterConfig()
    camera: CameraConfig = CameraConfig()
    vision: VisionConfig = VisionConfig()
    release: ReleaseConfig = ReleaseConfig()
    mission: MissionConfig = MissionConfig()
    runtime: RuntimeConfig = RuntimeConfig()

    def validate(self) -> None:
        fc = self.flight_controller
        router = self.telemetry_router
        camera = self.camera
        vision = self.vision
        release = self.release
        mission = self.mission
        if not fc.endpoint:
            raise ValueError("flight_controller.endpoint cannot be empty")
        if not 1200 <= fc.baud <= 4_000_000:
            raise ValueError("flight_controller.baud is outside a serial-safe range")
        if min(
            fc.heartbeat_timeout_s,
            fc.telemetry_timeout_s,
            fc.mission_refresh_s,
        ) <= 0.0:
            raise ValueError("flight-controller timeouts/refresh interval must be positive")
        if not router.serial_device:
            raise ValueError("telemetry_router.serial_device cannot be empty")
        if not 1200 <= router.baud <= 4_000_000:
            raise ValueError("telemetry_router.baud is outside a serial-safe range")
        if router.serial_reconnect_s <= 0.0:
            raise ValueError("telemetry-router reconnect interval must be positive")
        if router.status_interval_s <= 0.0:
            raise ValueError("telemetry_router.status_interval_s must be positive")
        for name, port in (
            ("companion_port", router.companion_port),
            ("router_local_port", router.router_local_port),
        ):
            if not 1 <= port <= 65535:
                raise ValueError(f"telemetry_router.{name} must be within 1..65535")
        expected_endpoint = (
            f"udpin:{router.companion_host}:{router.companion_port}"
        )
        if fc.endpoint != expected_endpoint:
            raise ValueError(
                "flight_controller.endpoint must match the router companion endpoint "
                f"({expected_endpoint})"
            )
        if router.router_local_port == router.companion_port:
            raise ValueError("router and companion UDP ports must be different")
        if router.tlog_max_mb <= 0 or router.tlog_retained_files < 1:
            raise ValueError("telemetry-router TLOG limits are invalid")
        if camera.width < 320 or camera.height < 240 or camera.fps <= 0:
            raise ValueError("camera width/height/fps are invalid")
        if not camera.device.strip():
            raise ValueError("camera.device cannot be empty")
        if not 20.0 <= camera.horizontal_fov_deg <= 170.0:
            raise ValueError("camera.horizontal_fov_deg must be calibrated within 20..170")
        intrinsics_set = camera.fx_px > 0.0 or camera.fy_px > 0.0
        if intrinsics_set:
            if camera.fx_px <= 0.0 or camera.fy_px <= 0.0:
                raise ValueError("camera fx_px/fy_px must be provided together")
            if not 0.0 <= camera.cx_px < camera.width:
                raise ValueError("camera.cx_px must lie inside the configured image")
            if not 0.0 <= camera.cy_px < camera.height:
                raise ValueError("camera.cy_px must lie inside the configured image")
        if len(camera.distortion) not in (4, 5, 8):
            raise ValueError("camera.distortion must contain 4, 5 or 8 coefficients")
        if not math.isfinite(camera.telemetry_alignment_s) or abs(camera.telemetry_alignment_s)>2:
            raise ValueError("camera.telemetry_alignment_s must be a signed measured offset within ±2 s")
        if not math.isfinite(camera.agl_offset_m) or abs(camera.agl_offset_m)>5:
            raise ValueError("camera.agl_offset_m must be within ±5 m")
        if camera.reconnect_s <= 0.0:
            raise ValueError("camera.reconnect_s must be positive")
        if vision.task not in (1, 2):
            raise ValueError("vision.task must be 1 or 2")
        if not 0.01 <= vision.confidence <= 0.95:
            raise ValueError("vision.confidence must be within 0.01..0.95")
        if vision.inference_hz <= 0.0:
            raise ValueError("vision.inference_hz must be positive")
        if min(vision.inference_size_task1, vision.inference_size_task2) < 320:
            raise ValueError("vision inference sizes must be at least 320")
        if not vision.angles or any(angle not in (0, 90, 180, 270) for angle in vision.angles):
            raise ValueError("vision.angles may contain only 0, 90, 180 and 270")
        if len(set(vision.angles)) != len(vision.angles):
            raise ValueError("vision.angles cannot contain duplicates")
        if not 1 <= release.servo_channel <= 16:
            raise ValueError("release.servo_channel must be within 1..16")
        if not 800 <= release.release_pwm <= 2200:
            raise ValueError("release.release_pwm must be within 800..2200")
        if release.required_servo_function not in (0, 58):
            raise ValueError(
                "release.required_servo_function must be 0 (independent) "
                "or 58 (explicit RC8/MAVLink compatibility)"
            )
        if release.required_servo_function == 58 and release.servo_channel != 8:
            raise ValueError("RC8/MAVLink compatibility requires release.servo_channel=8")
        if not release.actuator_brand.strip():
            raise ValueError("release.actuator_brand cannot be empty")
        if not 4.5 <= release.actuator_supply_v <= 6.0:
            raise ValueError("release.actuator_supply_v must be within 4.5..6.0 V")
        if release.payload_water_ml <= 0.0:
            raise ValueError("release.payload_water_ml must be positive")
        if release.payload_total_mass_kg < release.payload_water_ml / 1000.0:
            raise ValueError("release.payload_total_mass_kg is below the water mass")
        if release.actuator_delay_s < 0.0 or release.drag_rate_s < 0.0:
            raise ValueError("release delay and drag rate cannot be negative")
        if not 0.0 < release.min_agl_m < release.max_agl_m:
            raise ValueError("release AGL limits are invalid")
        if not 0.0 < release.min_airspeed_mps < release.max_airspeed_mps:
            raise ValueError("release airspeed limits are invalid")
        if not release.min_agl_m <= release.nominal_agl_m <= release.max_agl_m:
            raise ValueError("release.nominal_agl_m is outside the release gate")
        if not release.min_airspeed_mps <= release.nominal_airspeed_mps <= release.max_airspeed_mps:
            raise ValueError("release.nominal_airspeed_mps is outside the release gate")
        if not 1.0 <= release.max_roll_deg <= 45.0:
            raise ValueError("release.max_roll_deg must be within 1..45")
        if not 0.5 <= release.max_yaw_rate_dps <= 30.0:
            raise ValueError("release.max_yaw_rate_dps must be within 0.5..30")
        if release.max_cross_track_m <= 0.0 or release.command_window_s <= 0.0:
            raise ValueError("release cross-track/window settings must be positive")
        if release.target_tolerance_m <= 0 or any(not math.isfinite(value) or value<0 for value in
                (release.gps_relative_sigma_m,release.camera_position_sigma_m,release.actuator_delay_sigma_s,
                 release.altitude_sigma_m,release.velocity_sigma_mps,release.wind_sigma_mps,release.drag_sigma_s)):
            raise ValueError("release precision budget is invalid")
        if release.precision_required and mission.acceptance_mode != 'dynamic':
            raise ValueError("precision release requires dynamic mission acceptance")
        if mission.max_home_distance_m <= 0.0:
            raise ValueError("mission.max_home_distance_m must be positive")
        if not 5.0 <= mission.planning_bank_deg <= 35.0:
            raise ValueError("mission.planning_bank_deg must be within 5..35")
        if not 0 <= mission.tracking_margin_m < mission.max_home_distance_m/2:
            raise ValueError("mission.tracking_margin_m must fit inside the home boundary")
        if not release.min_airspeed_mps <= mission.planning_airspeed_mps <= release.max_airspeed_mps:
            raise ValueError(
                "mission.planning_airspeed_mps is outside the release airspeed gate"
            )
        if not 0 <= mission.min_satellites <= 30:
            raise ValueError("mission.min_satellites must be within 0..30")
        if mission.max_hdop <= 0.0:
            raise ValueError("mission.max_hdop must be positive")
        if mission.acceptance_mode not in {"dynamic", "pinned"}:
            raise ValueError("mission.acceptance_mode must be dynamic or pinned")
        if not 1 <= mission.stable_readbacks <= 3:
            raise ValueError("mission.stable_readbacks must be within 1..3")
        if mission.max_position_error_m <= 0.0:
            raise ValueError("mission.max_position_error_m must be positive")
        expected_path = mission.expected_path()
        if expected_path is None:
            if mission.expected_sha256:
                raise ValueError(
                    "mission.expected_sha256 requires mission.expected_file"
                )
            if mission.acceptance_mode == "pinned":
                raise ValueError(
                    "pinned mission acceptance requires mission.expected_file"
                )
        else:
            if not expected_path.is_file():
                raise ValueError(f"expected mission file is missing: {expected_path}")
            expected_hash = mission.expected_sha256.lower()
            if len(expected_hash) != 64 or any(
                character not in "0123456789abcdef" for character in expected_hash
            ):
                raise ValueError("mission.expected_sha256 must be a SHA-256 hex digest")
            actual_hash = hashlib.sha256(expected_path.read_bytes()).hexdigest()
            if actual_hash != expected_hash:
                raise ValueError(
                    "expected mission SHA-256 mismatch: "
                    f"configured={expected_hash}, actual={actual_hash}"
                )
        if self.runtime.status_hz <= 0.0:
            raise ValueError("runtime.status_hz must be positive")
        if self.runtime.mavlink_report_hz <= 0.0:
            raise ValueError("runtime.mavlink_report_hz must be positive")
        if self.runtime.mavlink_report_hz > self.runtime.status_hz:
            raise ValueError("runtime.mavlink_report_hz cannot exceed status_hz")
        if (
            self.runtime.telemetry_log_max_mb <= 0
            or self.runtime.telemetry_log_backups < 1
        ):
            raise ValueError("runtime telemetry log limits are invalid")
        if release.enabled:
            missing_confirmations = []
            if camera.device.strip().lower() == "auto":
                missing_confirmations.append(
                    "camera.device fixed to the calibrated camera"
                )
            if not camera.calibration_confirmed:
                missing_confirmations.append("camera.calibration_confirmed")
            elif not intrinsics_set:
                missing_confirmations.append("camera fx_px/fy_px/cx_px/cy_px")
            if not vision.validation_confirmed:
                missing_confirmations.append("vision.validation_confirmed")
            if not release.ballistics_calibrated:
                missing_confirmations.append("release.ballistics_calibrated")
            if release.precision_required and not release.precision_calibrated:
                missing_confirmations.append("release.precision_calibrated")
            if missing_confirmations:
                raise ValueError(
                    "active release requires completed calibration: "
                    + ", ".join(missing_confirmations)
                )


T = TypeVar("T")


def _load_section(cls: type[T], payload: dict[str, Any], name: str) -> T:
    raw = payload.get(name, {})
    if raw is None:
        raw = {}
    if not isinstance(raw, dict):
        raise ValueError(f"{name} must be a YAML mapping")
    allowed = {item.name for item in fields(cls)}
    unknown = set(raw) - allowed
    if unknown:
        raise ValueError(f"unknown {name} settings: {', '.join(sorted(unknown))}")
    if cls is VisionConfig and "angles" in raw:
        raw = {**raw, "angles": tuple(int(value) for value in raw["angles"])}
    if cls is CameraConfig and "distortion" in raw:
        raw = {
            **raw,
            "distortion": tuple(float(value) for value in raw["distortion"]),
        }
    return cls(**raw)


def load_config(path: Path) -> OnboardConfig:
    try:
        payload = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    except FileNotFoundError as exc:
        raise ValueError(f"onboard config not found: {path}") from exc
    except yaml.YAMLError as exc:
        raise ValueError(f"invalid onboard YAML: {exc}") from exc
    if not isinstance(payload, dict):
        raise ValueError("onboard config root must be a YAML mapping")
    allowed = {
        "flight_controller",
        "telemetry_router",
        "camera",
        "vision",
        "release",
        "mission",
        "runtime",
    }
    unknown = set(payload) - allowed
    if unknown:
        raise ValueError(f"unknown onboard sections: {', '.join(sorted(unknown))}")
    config = OnboardConfig(
        flight_controller=_load_section(
            FlightControllerConfig, payload, "flight_controller"
        ),
        telemetry_router=_load_section(
            TelemetryRouterConfig, payload, "telemetry_router"
        ),
        camera=_load_section(CameraConfig, payload, "camera"),
        vision=_load_section(VisionConfig, payload, "vision"),
        release=_load_section(ReleaseConfig, payload, "release"),
        mission=_load_section(MissionConfig, payload, "mission"),
        runtime=_load_section(RuntimeConfig, payload, "runtime"),
    )
    config.validate()
    return config


def camera_angles_rad(config: CameraConfig) -> tuple[float, float, float]:
    return (
        math.radians(config.mount_roll_deg),
        math.radians(config.mount_pitch_deg),
        math.radians(config.mount_yaw_deg),
    )
