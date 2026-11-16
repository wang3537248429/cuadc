from __future__ import annotations

import math
import os
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Mapping


def _discover_project_root() -> Path:
    def is_project_root(candidate: Path) -> bool:
        return all(
            (
                (candidate / "pyproject.toml").is_file(),
                (candidate / "assets/targets/catalog.json").is_file(),
                (candidate / "config/ardupilot/plane.parm").is_file(),
                (candidate / "worlds/cuadc_world.sdf.in").is_file(),
            )
        )

    configured = os.environ.get("CUADC_PROJECT_ROOT")
    if configured:
        candidate = Path(configured).expanduser().resolve()
        if is_project_root(candidate):
            return candidate

    starts = (Path.cwd().resolve(), Path(__file__).resolve().parent)
    checked: set[Path] = set()
    for start in starts:
        for candidate in (start, *start.parents):
            if candidate in checked:
                continue
            checked.add(candidate)
            if is_project_root(candidate):
                return candidate
    raise RuntimeError(
        "CUADC project root not found; run inside the project or set "
        "CUADC_PROJECT_ROOT"
    )


ROOT = _discover_project_root()
GAZEBO_DIR = ROOT
SIM_CONFIG_DIR = ROOT / "config"
RUNTIME_DIR = Path(os.environ.get("CUADC_RUNTIME_DIR", ROOT / ".runtime"))
RUNS_DIR = Path(os.environ.get("CUADC_RUNS_DIR", ROOT / "runs"))

SCAN_START_TAG = 9101
SCAN_END_TAG = 9102
PAYLOAD_SERVO = 8
PAYLOAD_RELEASE_PWM = 1800
# Mission Planner route files supplied for the physical competition use this
# low pulse as the release action. It is intentionally separate from the
# legacy dynamic-route profile above, whose high latch remains 1800 us.
RAW_REPLAY_RELEASE_PWM = 1000
# Provisional real-payload profile supplied by the operator.  PTK 8812 MG-D
# data at 5 V is used only as a speed-class reference because the exact PTK
# model has not been identified.  Ground video must replace the total delay.
PAYLOAD_ACTUATOR_BRAND = "PTK"
PAYLOAD_ACTUATOR_SUPPLY_V = 5.0
PAYLOAD_ACTUATOR_DELAY_S = 0.12
PAYLOAD_WATER_ML = 350.0
PAYLOAD_TOTAL_MASS_KG = 0.370
# Airframe and propulsion values supplied for the physical aircraft. The
# Gazebo model's auxiliary links account for 0.701 kg of the 2.50 kg empty
# airframe, leaving 1.799 kg on the fuselage link itself.
AIRCRAFT_EMPTY_MASS_KG = 2.500
AIRCRAFT_AUXILIARY_LINK_MASS_KG = 0.701
AIRCRAFT_FUSELAGE_MASS_KG = (
    AIRCRAFT_EMPTY_MASS_KG - AIRCRAFT_AUXILIARY_LINK_MASS_KG
)
AIRCRAFT_STREAMER_MASS_KG = 6 * 0.003
AIRCRAFT_LAUNCH_MASS_KG = (
    AIRCRAFT_EMPTY_MASS_KG + PAYLOAD_TOTAL_MASS_KG + AIRCRAFT_STREAMER_MASS_KG
)
MOTOR_COUNT = 2
MAX_THRUST_KGF = 3.5
STANDARD_GRAVITY_MPS2 = 9.80665
MAX_THRUST_N = MAX_THRUST_KGF * STANDARD_GRAVITY_MPS2
MAX_THRUST_PER_MOTOR_N = MAX_THRUST_N / MOTOR_COUNT
MAX_MOTOR_SHAFT_SPEED_RAD_S = 1000.0
MOTOR_THRUST_COEFFICIENT = (
    MAX_THRUST_PER_MOTOR_N / MAX_MOTOR_SHAFT_SPEED_RAD_S**2
)
# Belly-landed fixed wings have no wheel brakes.  A bounded contact brake
# keeps the post-touchdown ground roll inside the declared landing zone while
# remaining completely inactive in flight.
GROUND_BRAKE_FORCE_N = 30.0
# The public 0.312 kg drop reference gives k~=0.113 1/s at 20 m/s.  Preserve
# its effective CdA and scale by mass for the 0.370 kg full-bottle profile.
PAYLOAD_DRAG_RATE_S = 0.095
# Gazebo Sim 7's scalar SDF loader evaluates a bare constant c as c*c
# (MakeConstantScalingFactor stores c/3 and three constant-c polynomials).
# Supplying sqrt(k) therefore produces the intended acceleration rate k.
GAZEBO_WIND_FORCE_SCALE = math.sqrt(PAYLOAD_DRAG_RATE_S)
PAYLOAD_NOMINAL_AGL_M = 14.0
# Reconnaissance remains deliberately conservative; the attack/return leg
# uses the higher cruise value below so the aircraft does not spend the
# competition window loitering at the scan speed.
PAYLOAD_NOMINAL_AIRSPEED_MPS = 12.5
PAYLOAD_TARGET_TOLERANCE_M = 0.35
PAYLOAD_MAX_CROSS_TRACK_M = 6.0
PAYLOAD_COMMAND_WINDOW_S = 0.25
PAYLOAD_MAX_ROLL_RAD = math.radians(20.0)
PAYLOAD_MAX_YAW_RATE_RPS = math.radians(12.0)
# The public competition baseline is 300 m.  Official route profiles may set a
# larger value from their checked-in field configuration; the value is read at
# process start so validation and the final rule report use the same boundary.
try:
    MISSION_MAX_HOME_DISTANCE_M = float(
        os.environ.get("CUADC_MISSION_MAX_HOME_DISTANCE_M", "300")
    )
except ValueError as exc:
    raise ValueError("CUADC_MISSION_MAX_HOME_DISTANCE_M must be numeric") from exc
if not math.isfinite(MISSION_MAX_HOME_DISTANCE_M) or MISSION_MAX_HOME_DISTANCE_M <= 0:
    raise ValueError("CUADC_MISSION_MAX_HOME_DISTANCE_M must be positive")
MISSION_TRACKING_MARGIN_M = 5.0
MISSION_PLANNING_BANK_DEG = 20.0
MISSION_PLANNING_AIRSPEED_MPS = PAYLOAD_NOMINAL_AIRSPEED_MPS
MP_UDP_PORT = 14550
COMPANION_UDP_PORT = 14551
WMS_PORT = 8765
SITL_INSTANCE = 5
SITL_TCP_PORT = 5760 + SITL_INSTANCE * 10
FDM_PORT = 9002 + SITL_INSTANCE * 10
# ArduPilot 文档和 SITL 示例广泛使用的公开 Canberra 测试坐标。仿真只依赖
# 该点附近的局部东北坐标，不包含真实比赛或试飞场地位置。
ORIGIN_LAT_DEG = -35.363261
ORIGIN_LON_DEG = 149.165230
ORIGIN_ALT_M = 46.0
DOWN_CAMERA_HORIZONTAL_FOV_RAD = math.radians(80.0)
SIM_CAMERA_WIDTH = 1920
SIM_CAMERA_HEIGHT = 1080
SIM_CAMERA_FPS = 30
SIM_VISION_INFERENCE_HZ = 6.0
# Signed camera/telemetry time calibration. Keep neutral until measured for
# the actual capture/inference pipeline; an old detector delay is not reusable.
CAMERA_TELEMETRY_ALIGNMENT_S = 0.0
# The simulator explicitly sets HOME to ground; camera is 0.30 m below body.
SIM_CAMERA_AGL_OFFSET_M = -0.30
RECON_AIRSPEED_MPS = 10.5
RECON_TURN_AIRSPEED_MPS = 12.5

PARAM_NAMES = (
    "SCR_USER1",
    "SCR_USER2",
    "SCR_USER3",
    "SCR_USER4",
    "SCR_USER5",
    "SCR_USER6",
    "SIM_WIND_SPD",
    "SIM_WIND_DIR",
)

SIM_FLIGHT_PROFILE = {
    "AIRSPEED_CRUISE": 12.5,
    "AIRSPEED_MIN": 10.0,
    "AIRSPEED_MAX": 18.0,
    "ROLL_LIMIT_DEG": 26.0,
    "WP_LOITER_RAD": 70.0,
    "WP_RADIUS": 5.0,
    "WP_MAX_RADIUS": 0.0,
    "NAVL1_PERIOD": 9.0,
    "NAVL1_DAMPING": 1.0,
    "NAVL1_XTRACK_I": 0.05,
    "RLL2SRV_TCONST": 0.90,
    "RLL_RATE_FF": 0.32,
    "RLL_RATE_P": 0.14,
    "RLL_RATE_I": 0.09,
    "RLL_RATE_IMAX": 0.40,
    "RLL_RATE_D": 0.025,
    "RLL_RATE_SMAX": 120.0,
    "PTCH2SRV_TCONST": 0.4,
    "PTCH2SRV_RLL": 0.3,
    "PTCH_RATE_FF": 0.55,
    "PTCH_RATE_P": 0.18,
    "PTCH_RATE_I": 0.15,
    "PTCH_RATE_D": 0.008,
    "TECS_TIME_CONST": 4.0,
    "TECS_PTCH_DAMP": 0.8,
    "TECS_SPDWEIGHT": 2.0,
    "TECS_THR_DAMP": 0.8,
    "STALL_PREVENTION": 1.0,
    "TECS_LAND_ARSPD": 11.0,
    "TECS_LAND_SINK": 0.6,
    "TECS_LAND_SPDWGT": 0.0,
    "TECS_LAND_TCONST": 1.8,
    "LAND_FLARE_ALT": 1.0,
    "LAND_FLARE_SEC": 0.8,
    "TECS_SINK_MAX": 5.0,
    "TECS_CLMB_MAX": 3.0,
    "THR_MAX": 100.0,
    "TKOFF_THR_MAX": 45.0,
    "TECS_LAND_THR": 0.0,
    "LAND_PF_ARSPD": 11.0,
    "LAND_PITCH_DEG": -3.0,
    "LAND_THEN_NEUTRL": 1.0,
}


@dataclass(frozen=True)
class SimParameters:
    task: int = 1
    group: str = "A"
    seed: int = 1
    launch_speed_mps: float = 13.5
    launch_pitch_deg: float = 10.0
    # Keep the parameter-model default at the historical 0.20 value so
    # float32 round-trip comparisons remain stable.  The official simulation
    # profile sets SCR_USER6=0.10 in plane.parm; the live YOLO worker therefore
    # still uses the lower threshold needed for edge-clipped panels.
    confidence: float = 0.20
    wind_speed_mps: float = 0.0
    wind_from_deg: float = 0.0

    def validate(self) -> None:
        if self.task not in (1, 2):
            raise ValueError("SCR_USER1 must be 1 (Task 1) or 2 (Task 2)")
        if self.group not in ("A", "B"):
            raise ValueError("SCR_USER2 must select group A or B")
        if self.seed < 0:
            raise ValueError("SCR_USER3 seed must be non-negative")
        if not 8.0 <= self.launch_speed_mps <= 35.0:
            raise ValueError("SCR_USER4 launch speed must be within 8..35 m/s")
        if not -5.0 <= self.launch_pitch_deg <= 30.0:
            raise ValueError("SCR_USER5 launch pitch must be within -5..30 deg")
        if not 0.01 <= self.confidence <= 0.95:
            raise ValueError("SCR_USER6 confidence must be within 0.01..0.95")
        if not 0.0 <= self.wind_speed_mps <= 25.0:
            raise ValueError("SIM_WIND_SPD must be within 0..25 m/s")

    @classmethod
    def from_ardupilot(cls, values: Mapping[str, float]) -> "SimParameters":
        result = cls(
            task=int(round(values.get("SCR_USER1", 1))),
            group="B" if int(round(values.get("SCR_USER2", 0))) == 1 else "A",
            seed=max(0, int(round(values.get("SCR_USER3", 1)))),
            launch_speed_mps=float(values.get("SCR_USER4", 13.5)),
            launch_pitch_deg=float(values.get("SCR_USER5", 10.0)),
            confidence=float(values.get("SCR_USER6", 0.20)),
            wind_speed_mps=float(values.get("SIM_WIND_SPD", 0.0)),
            wind_from_deg=float(values.get("SIM_WIND_DIR", 0.0)) % 360.0,
        )
        result.validate()
        return result

    def to_dict(self) -> dict:
        return asdict(self)
