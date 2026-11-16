from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import pytest

from cuadc_sim.mission import latlon_to_ne, ne_to_latlon
from cuadc_sim.config import SIM_FLIGHT_PROFILE
from cuadc_sim.route_state import RouteState, in_takeoff_landing_zone, public_areas
from cuadc_sim.wms import LAYER_NAME, capabilities_xml, render_map


def test_ardupilot_46_profile_uses_active_speed_and_l1_parameters():
    root = Path(__file__).resolve().parents[1]
    text = (root / "config/ardupilot/plane.parm").read_text(encoding="utf-8")
    parameters = {
        line.split()[0]: line.split()[1]
        for line in text.splitlines()
        if line.strip() and not line.lstrip().startswith("#")
    }
    assert parameters["AIRSPEED_CRUISE"] == "12.5"
    assert parameters["AIRSPEED_MIN"] == "10"
    assert parameters["AIRSPEED_MAX"] == "18"
    assert parameters["ROLL_LIMIT_DEG"] == "26"
    assert parameters["WP_LOITER_RAD"] == "70"
    assert parameters["NAVL1_PERIOD"] == "9"
    assert parameters["NAVL1_DAMPING"] == "1.0"
    assert parameters["STALL_PREVENTION"] == "1"
    assert "TRIM_ARSPD_CM" not in parameters
    assert "ARSPD_FBW_MIN" not in parameters
    assert "ARSPD_FBW_MAX" not in parameters
    for name, value in SIM_FLIGHT_PROFILE.items():
        assert float(parameters[name]) == pytest.approx(value)


@pytest.mark.parametrize("north,east", [(0, 0), (25, -25), (200, 30), (-200, -30), (300, 0)])
def test_wms_and_gazebo_coordinate_roundtrip_is_sub_meter(north, east):
    lat, lon = ne_to_latlon(north, east)
    restored_north, restored_east = latlon_to_ne(lat, lon)
    assert restored_north == pytest.approx(north, abs=0.001)
    assert restored_east == pytest.approx(east, abs=0.001)


def test_wms_capabilities_and_png_are_mission_planner_compatible():
    capabilities = capabilities_xml("http://127.0.0.1:8765/wms?").decode()
    assert 'version="1.1.1"' in capabilities
    assert "EPSG:4326" in capabilities
    assert "image/png" in capabilities
    assert f"<Name>{LAYER_NAME}</Name>" in capabilities
    south, west = ne_to_latlon(-320, -180)
    north, east = ne_to_latlon(320, 180)
    png = render_map(
        (west, south, east, north),
        512,
        512,
        RouteState(
            group="A",
            mission=[
                {"seq": 1, "command": 22, "north_m": 0.0, "east_m": 0.0, "alt_m": 20.0},
                {"seq": 2, "command": 16, "north_m": 100.0, "east_m": 20.0, "alt_m": 20.0},
            ],
        ).to_dict(),
    )
    image = cv2.imdecode(np.frombuffer(png, dtype=np.uint8), cv2.IMREAD_COLOR)
    assert image.shape == (512, 512, 3)


def test_route_state_contains_only_public_geometry_and_estimates():
    state = RouteState(
        detections=[
            {
                "track_id": "T01",
                "value": 8,
                "confidence": 0.8,
                "frames": 3,
                "label": "8",
                "north_m": 201.0,
                "east_m": 2.0,
            }
        ]
    ).to_dict()
    assert state["schema"] == "route_state.v1"
    assert "wells" not in json.dumps(state)
    assert {zone["name"] for zone in public_areas()["target_zones"]} == {"A", "B"}
    assert in_takeoff_landing_zone(25, -25)
    assert not in_takeoff_landing_zone(25.01, 0)


def test_default_decision_and_display_modules_never_read_scenario_truth():
    root = Path(__file__).resolve().parents[1]
    for relative in (
        "src/cuadc_sim/companion.py",
        "src/cuadc_sim/vision.py",
        "src/cuadc_sim/vision_service.py",
        "src/cuadc_sim/gui.py",
        "src/cuadc_sim/wms.py",
    ):
        source = (root / relative).read_text(encoding="utf-8")
        assert "scenario.json" not in source
        assert "TRUTH_FALLBACK" not in source


def test_mission_planner_config_script_backs_up_configures_and_restores(tmp_path: Path):
    root = Path(__file__).resolve().parents[1]
    config = tmp_path / "config.xml"
    config.write_text(
        '<?xml version="1.0"?><Config><MapType>Old Map</MapType></Config>',
        encoding="utf-8",
    )
    script = root / "scripts/configure_mission_planner_wms.py"
    subprocess.run(
        [sys.executable, str(script), "--config", str(config)],
        check=True,
        capture_output=True,
        text=True,
    )
    configured = ET.parse(config).getroot()
    assert configured.findtext("WMSserver") == "http://127.0.0.1:8765/wms?"
    assert configured.findtext("WMSLayer") == "cuadc_field"
    assert configured.findtext("MapType") == "WMS Custom"
    subprocess.run(
        [sys.executable, str(script), "--config", str(config), "--restore"],
        check=True,
        capture_output=True,
        text=True,
    )
    restored = ET.parse(config).getroot()
    assert restored.findtext("MapType") == "Old Map"
