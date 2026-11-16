from __future__ import annotations

import json
import math
import os
from dataclasses import asdict, dataclass, field
from pathlib import Path

from .config import ORIGIN_LAT_DEG, ORIGIN_LON_DEG, RUNTIME_DIR
from .mission import latlon_to_ne


@dataclass
class Telemetry:
    timestamp_s: float = 0.0
    sample_time_s: float = 0.0
    attitude_sample_time_s: float = 0.0
    attitude_timestamp_s: float = 0.0
    lat_deg: float = 0.0
    lon_deg: float = 0.0
    relative_alt_m: float = 0.0
    north_m: float = 0.0
    east_m: float = 0.0
    velocity_north_mps: float = 0.0
    velocity_east_mps: float = 0.0
    velocity_down_mps: float = 0.0
    roll_rad: float = 0.0
    pitch_rad: float = 0.0
    yaw_rad: float = 0.0
    roll_rate_rps: float = 0.0
    pitch_rate_rps: float = 0.0
    yaw_rate_rps: float = 0.0
    target_roll_deg: float = 0.0
    target_pitch_deg: float = 0.0
    airspeed_mps: float = 0.0
    groundspeed_mps: float = 0.0
    throttle_percent: int = 0
    servo_pwm: list[int] = field(default_factory=lambda: [0] * 16)
    armed: bool = False
    auto: bool = False
    mission_seq: int = -1

    def update_position(
        self,
        message,
        origin_lat_deg: float = ORIGIN_LAT_DEG,
        origin_lon_deg: float = ORIGIN_LON_DEG,
    ) -> None:
        self.timestamp_s = float(getattr(message, "_timestamp", 0.0))
        self.sample_time_s = float(getattr(message, "time_boot_ms", 0.0))/1000.0
        self.lat_deg = float(message.lat) / 1e7
        self.lon_deg = float(message.lon) / 1e7
        self.relative_alt_m = float(message.relative_alt) / 1000.0
        self.north_m, self.east_m = latlon_to_ne(
            self.lat_deg,
            self.lon_deg,
            origin_lat_deg,
            origin_lon_deg,
        )
        self.velocity_north_mps = float(message.vx) / 100.0
        self.velocity_east_mps = float(message.vy) / 100.0
        self.velocity_down_mps = float(message.vz) / 100.0
        if getattr(message, "hdg", 65535) != 65535 and self.attitude_sample_time_s <= 0:
            self.yaw_rad = math.radians(float(message.hdg) / 100.0)

    def update_attitude(self, message) -> None:
        self.attitude_sample_time_s = float(getattr(message, "time_boot_ms", 0.0))/1000.0
        self.attitude_timestamp_s = float(getattr(message, "_timestamp", 0.0))
        self.roll_rad = float(message.roll)
        self.pitch_rad = float(message.pitch)
        self.yaw_rad = float(message.yaw)
        self.roll_rate_rps = float(message.rollspeed)
        self.pitch_rate_rps = float(message.pitchspeed)
        self.yaw_rate_rps = float(message.yawspeed)

    def update_nav_controller(self, message) -> None:
        self.target_roll_deg = float(message.nav_roll)
        self.target_pitch_deg = float(message.nav_pitch)

    def update_hud(self, message) -> None:
        self.airspeed_mps = float(message.airspeed)
        self.groundspeed_mps = float(message.groundspeed)
        self.throttle_percent = int(message.throttle)

    def update_servos(self, message) -> None:
        self.servo_pwm = [
            int(getattr(message, f"servo{index}_raw", 0))
            for index in range(1, 17)
        ]


def write_telemetry(telemetry: Telemetry, path: Path = RUNTIME_DIR / "telemetry.json") -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(".json.tmp")
    temporary.write_text(
        json.dumps(asdict(telemetry), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)


def append_telemetry_history(
    telemetry: Telemetry, path: Path = RUNTIME_DIR / "telemetry.jsonl"
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(asdict(telemetry), ensure_ascii=False) + "\n")
