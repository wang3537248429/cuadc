from __future__ import annotations

import json
import os
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from .config import RUNS_DIR


@dataclass
class RunResult:
    schema: str = "cuadc-run-result.v3"
    run_id: str = ""
    created_at: str = ""
    seed: int = 1
    task: int = 1
    group: str = "A"
    source: str = "PENDING"
    status: str = "STARTING"
    replay_completed: bool = False
    competition_rules_passed: bool | None = None
    competition_rule_failures: list[str] = field(default_factory=list)
    mission_capability: str = "BLOCKED"
    mission_warnings: list[str] = field(default_factory=list)
    detections: list[dict[str, Any]] = field(default_factory=list)
    selected_target: dict[str, Any] | None = None
    recognition_complete: bool = False
    guessed: bool = False
    release: dict[str, Any] | None = None
    impact: dict[str, Any] | None = None
    first_touchdown: dict[str, Any] | None = None
    final_stop: dict[str, Any] | None = None
    route_metrics: dict[str, Any] = field(
        default_factory=lambda: {
            "distance_m": 0.0,
            "max_home_distance_m": 0.0,
            "mission_time_s": 0.0,
            "max_roll_deg": 0.0,
            "max_landing_roll_deg": 0.0,
            "min_airspeed_mps": None,
            "scan_cross_track_rms_m": 0.0,
            "scan_cross_track_max_m": 0.0,
            "scan_mean_airspeed_mps": 0.0,
            "scan_mean_agl_m": 0.0,
            "scan_samples": 0,
            "turn_radius_mean_m": 0.0,
            "turn_radius_min_m": None,
            "turn_radius_max_m": 0.0,
            "turn_radius_samples": 0,
        }
    )
    message: str = ""

    @classmethod
    def create(cls, seed: int, task: int, group: str) -> "RunResult":
        now = datetime.now(timezone.utc)
        return cls(
            run_id=now.strftime("%Y%m%dT%H%M%S.%fZ"),
            created_at=now.isoformat(),
            seed=seed,
            task=task,
            group=group,
        )

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


class ResultStore:
    def __init__(self, result: RunResult, runs_dir: Path = RUNS_DIR):
        self.result = result
        self.runs_dir = runs_dir
        self.run_dir = runs_dir / result.run_id
        self.path = self.run_dir / "result.json"

    def save(self) -> Path:
        self.run_dir.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(".json.tmp")
        temporary.write_text(
            json.dumps(self.result.to_dict(), ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        os.replace(temporary, self.path)
        latest = self.runs_dir / "latest.json"
        latest_tmp = latest.with_suffix(".json.tmp")
        latest_tmp.write_text(self.path.read_text(encoding="utf-8"), encoding="utf-8")
        os.replace(latest_tmp, latest)
        return self.path
