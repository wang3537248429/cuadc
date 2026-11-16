from __future__ import annotations

import json
import math
import os
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

from .config import RUNTIME_DIR
from .scenario import GROUP_CENTERS_NE, ZONE_HALF_SIZE_M
from .field_layout import FieldLayout, load_field_layout


TAKEOFF_LANDING_HALF_SIZE_M = 25.0


@dataclass
class RouteState:
    """Public route/telemetry state shared by the GUI and local WMS."""

    schema: str = "route_state.v1"
    updated_at: float = 0.0
    group: str = "A"
    phase: str = "STARTING"
    aircraft: dict[str, Any] = field(default_factory=dict)
    mission: list[dict[str, Any]] = field(default_factory=list)
    trajectory: list[dict[str, float]] = field(default_factory=list)
    detections: list[dict[str, Any]] = field(default_factory=list)
    selected_target: dict[str, Any] | None = None
    release: dict[str, Any] | None = None
    landing: dict[str, Any] = field(default_factory=dict)
    mission_capability: str = "BLOCKED"
    mission_warnings: list[str] = field(default_factory=list)
    scan_attempt: int = 1
    scan_coverage_ratio: float = 0.0
    stable_target_count: int = 0
    fallback_reason: str | None = None
    origin_lat_deg: float = -35.363261
    origin_lon_deg: float = 149.165230
    field_layout: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        payload["public_areas"] = public_areas(self.field_layout or None, self.group)
        return payload


def public_areas(
    layout_data: dict[str, Any] | None = None,
    group: str = "A",
) -> dict[str, Any]:
    layout = FieldLayout(**layout_data) if layout_data else load_field_layout()
    other = "B" if group == "A" else "A"
    heading = math.radians(layout.target_heading_deg)
    centers = layout.group_centers_ne or {
        group: (layout.target_center_north_m, layout.target_center_east_m),
        other: (
            layout.target_center_north_m + math.sin(heading) * 110.0,
            layout.target_center_east_m - math.cos(heading) * 110.0,
        ),
    }
    return {
        "takeoff_landing": {
            "center_north_m": layout.runway_center_north_m,
            "center_east_m": layout.runway_center_east_m,
            "heading_deg": layout.runway_heading_deg,
            "half_length_m": layout.runway_length_m / 2.0,
            "half_width_m": layout.runway_width_m / 2.0,
        },
        "target_zones": [
            {
                "name": name,
                "center_north_m": centers[name][0],
                "center_east_m": centers[name][1],
                "half_size_m": ZONE_HALF_SIZE_M,
                "heading_deg": layout.target_heading_deg,
            }
            for name in ("A", "B")
        ],
        "max_home_distance_m": 300.0,
        "field_size_m": layout.field_size_m,
    }


def in_takeoff_landing_zone(
    north_m: float,
    east_m: float,
    layout: FieldLayout | None = None,
) -> bool:
    if layout is not None:
        delta_n = north_m - layout.runway_center_north_m
        delta_e = east_m - layout.runway_center_east_m
        heading = math.radians(layout.runway_heading_deg)
        along = delta_n * math.cos(heading) + delta_e * math.sin(heading)
        cross = -delta_n * math.sin(heading) + delta_e * math.cos(heading)
        return (
            abs(along) <= layout.runway_length_m / 2.0
            and abs(cross) <= layout.runway_width_m / 2.0
        )
    return (
        abs(north_m) <= TAKEOFF_LANDING_HALF_SIZE_M
        and abs(east_m) <= TAKEOFF_LANDING_HALF_SIZE_M
    )


def save_route_state(
    state: RouteState,
    path: Path = RUNTIME_DIR / "route_state.json",
) -> None:
    state.updated_at = time.time()
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(".json.tmp")
    temporary.write_text(
        json.dumps(state.to_dict(), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)


def load_route_state(path: Path = RUNTIME_DIR / "route_state.json") -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError):
        return RouteState().to_dict()
    if payload.get("schema") != "route_state.v1":
        return RouteState().to_dict()
    return payload
