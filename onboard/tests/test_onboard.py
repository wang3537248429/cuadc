from __future__ import annotations

from dataclasses import replace
import hashlib
import math
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from pymavlink import mavutil

from cuadc_onboard.cli import expected_value_from_path
from cuadc_onboard import camera as camera_module
from cuadc_onboard.camera import CameraVisionWorker, open_camera
from cuadc_onboard.camera_device import discover_camera_devices
from cuadc_onboard.config import OnboardConfig, RuntimeConfig, load_config
from cuadc_onboard.router import MavlinkRouter, TlogWriter
from cuadc_onboard.service import rotate_file
from cuadc_onboard.runtime import (
    FLIGHT_CONTROLLER_HEALTH_WARNING_TTL_S,
    ONBOARD_STATUS_CODES,
    STATUSTEXT_FRAGMENT_TTL_S,
    OnboardRuntime,
    release_gate_reasons,
    solution_with_actuator_delay,
)
from cuadc_onboard.ballistics import solve_release
from cuadc_onboard.mavlink_io import MavlinkClient
from cuadc_onboard.config import ORIGIN_LAT_DEG, ORIGIN_LON_DEG
from cuadc_onboard.mission import (
    MISSION_MODE_RECON_ONLY,
    ReconContract,
    MissionItem,
    assess_mission,
    build_attack_patch,
    compare_mission_reference,
    is_shadow_neutralization,
    latlon_to_ne,
    load_qgc_waypoints,
    neutralize_release_item,
    resequence,
    save_patch_backup,
    validate_mission,
)
from cuadc_onboard.telemetry import Telemetry
from cuadc_onboard.selection import Selection


ROOT = Path(__file__).resolve().parents[1]


def active_test_config():
    """Synthetic calibrated profile for unit tests; never an installed device config."""
    base = load_config(ROOT / "config/onboard.yaml")
    return replace(
        base,
        camera=replace(base.camera, calibration_confirmed=True),
        vision=replace(base.vision, validation_confirmed=True),
        release=replace(base.release, enabled=True, ballistics_calibrated=True, precision_required=False),
    )


def load_qgc(path: Path) -> list[MissionItem]:
    result: list[MissionItem] = []
    for line in path.read_text(encoding="utf-8").splitlines()[1:]:
        fields = line.split("\t")
        result.append(
            MissionItem(
                seq=int(fields[0]),
                current=int(fields[1]),
                frame=int(fields[2]),
                command=int(fields[3]),
                param1=float(fields[4]),
                param2=float(fields[5]),
                param3=float(fields[6]),
                param4=float(fields[7]),
                x=float(fields[8]),
                y=float(fields[9]),
                z=float(fields[10]),
                autocontinue=int(fields[11]),
            )
        )
    return result


def onboard_mission() -> list[MissionItem]:
    mission = load_qgc(ROOT / "tests/fixtures/group_a.waypoints")
    mission[9] = replace(mission[9], param1=8.0, param2=1800.0)
    return mission


def pinned_repository_config():
    base = active_test_config()
    expected_path = ROOT / "config/route_takeoff_search.waypoints"
    return replace(
        base,
        release=replace(base.release, enabled=False),
        mission=replace(
            base.mission,
            acceptance_mode="pinned",
            expected_file=str(expected_path),
            expected_sha256=hashlib.sha256(expected_path.read_bytes()).hexdigest(),
        ),
    )


def test_repository_onboard_config_is_private_by_default_and_valid():
    assert not OnboardConfig().release.enabled
    config = load_config(ROOT / "config/onboard.yaml")
    assert config.flight_controller.endpoint == "udpin:127.0.0.1:14551"
    assert config.telemetry_router.serial_device == "/dev/cuadc-flight-controller"
    assert config.camera.device == (
        "auto"
    )
    assert config.camera.horizontal_fov_deg == pytest.approx(100.0)
    assert config.vision.inference_hz == pytest.approx(3.0)
    assert config.vision.inference_size_task1 == 320
    assert config.vision.angles == (0,)
    assert config.vision.max_result_age_s == pytest.approx(5.0)
    assert config.release.servo_channel == 8
    assert config.release.release_pwm == 1800
    assert config.release.required_servo_function == 58
    assert config.release.actuator_brand == "PTK"
    assert config.release.actuator_supply_v == pytest.approx(5.0)
    assert config.release.payload_water_ml == pytest.approx(350.0)
    assert config.release.payload_total_mass_kg == pytest.approx(0.370)
    assert not config.camera.calibration_confirmed
    assert not config.vision.validation_confirmed
    assert config.release.actuator_delay_s == pytest.approx(0.12)
    assert config.release.drag_rate_s == pytest.approx(0.095)
    assert not config.release.ballistics_calibrated
    assert not config.release.enabled
    assert config.vision.weights_path().name == "task1_v4m_best.pt"
    assert config.mission.acceptance_mode == "dynamic"
    assert config.mission.planning_airspeed_mps == pytest.approx(12.8)
    assert config.mission.planning_bank_deg == pytest.approx(20.0)
    assert config.mission.stable_readbacks == 2
    assert config.mission.expected_path() is None
    assert config.mission.expected_sha256 == ""


def test_dynamic_runtime_accepts_recon_only_route_without_mission_or_servo_writes(
    tmp_path: Path,
):
    base = active_test_config()
    config = replace(
        base,
        release=replace(base.release, enabled=True),
        mission=replace(base.mission, stable_readbacks=1),
        runtime=RuntimeConfig(directory=str(tmp_path)),
    )
    live = [
        MissionItem(seq=0, frame=0, command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT),
        MissionItem(seq=1, frame=3, command=mavutil.mavlink.MAV_CMD_NAV_TAKEOFF),
        MissionItem(
            seq=2,
            frame=3,
            command=mavutil.mavlink.MAV_CMD_JUMP_TAG,
            param1=9101.0,
        ),
        MissionItem(
            seq=3,
            frame=3,
            command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            x=0.10077250,
            y=0.10004220,
            z=20.0,
        ),
        MissionItem(
            seq=4,
            frame=3,
            command=mavutil.mavlink.MAV_CMD_JUMP_TAG,
            param1=9102.0,
        ),
        MissionItem(
            seq=5,
            frame=3,
            command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            x=0.10087250,
            y=0.10014220,
            z=20.0,
        ),
        MissionItem(
            seq=6,
            frame=3,
            command=mavutil.mavlink.MAV_CMD_NAV_LAND,
            x=0.09927250,
            y=0.10024220,
        ),
    ]

    class Client:
        @staticmethod
        def download_mission(timeout: float):
            assert timeout == 10.0
            return list(live)

        @staticmethod
        def write_partial(_items):
            raise AssertionError("recon-only routes must never be patched")

        @staticmethod
        def set_servo(_channel, _pwm):
            raise AssertionError("recon-only routes must never actuate a servo")

        @staticmethod
        def status_text(_text, _severity):
            return None

    runtime = OnboardRuntime(config)
    runtime.client = Client()
    runtime._refresh_mission()

    assert runtime.state["status"] == "MISSION_INVALID"
    assert runtime.state["mission_mode"] == "BLOCKED"
    assert runtime.contract is None
    assert runtime.state["mission_reference"]["live_release_command"] is None
    assert any("DO_SET_SERVO" in error for error in runtime.state["mission_errors"])


def test_camera_worker_stop_releases_live_capture(tmp_path: Path):
    class FakeCapture:
        released = False

        def release(self):
            self.released = True

    worker = CameraVisionWorker(OnboardConfig(), lambda: {}, tmp_path)
    capture = FakeCapture()
    worker._set_capture(capture)

    worker.stop()

    assert worker.stop_event.is_set()
    assert capture.released is True


def test_auto_camera_discovery_uses_capture_index_zero(tmp_path: Path):
    dev_dir = tmp_path / "dev"
    by_id_dir = dev_dir / "v4l" / "by-id"
    by_id_dir.mkdir(parents=True)
    video0 = dev_dir / "video0"
    video1 = dev_dir / "video1"
    video0.touch()
    video1.touch()
    capture_link = by_id_dir / "usb-example_CAMERA-video-index0"
    metadata_link = by_id_dir / "usb-example_CAMERA-video-index1"
    capture_link.symlink_to(Path("../../video0"))
    metadata_link.symlink_to(Path("../../video1"))

    devices = discover_camera_devices(
        by_id_dir=by_id_dir,
        dev_dir=dev_dir,
        sys_class_dir=tmp_path / "missing-sysfs",
    )

    assert devices == (str(capture_link),)


def test_auto_camera_sysfs_fallback_rejects_metadata_index(tmp_path: Path):
    dev_dir = tmp_path / "dev"
    sys_class_dir = tmp_path / "sys" / "class" / "video4linux"
    driver_dir = tmp_path / "sys" / "drivers" / "uvcvideo"
    dev_dir.mkdir()
    driver_dir.mkdir(parents=True)
    for name, index in (("video0", "0"), ("video1", "1")):
        (dev_dir / name).touch()
        sys_entry = sys_class_dir / name
        (sys_entry / "device").mkdir(parents=True)
        (sys_entry / "index").write_text(index, encoding="utf-8")
        (sys_entry / "device" / "driver").symlink_to(driver_dir)

    devices = discover_camera_devices(
        by_id_dir=tmp_path / "missing-by-id",
        dev_dir=dev_dir,
        sys_class_dir=sys_class_dir,
    )

    assert devices == (str(dev_dir / "video0"),)


def test_open_camera_auto_skips_unreadable_device(monkeypatch):
    class FakeCapture:
        def __init__(self, source, _backend):
            self.source = str(source)
            self.opened = self.source.endswith("video1")
            self.properties = {}
            self.released = False

        def set(self, name, value):
            self.properties[name] = value
            return True

        def get(self, name):
            if name == camera_module.cv2.CAP_PROP_FPS:
                return 30.0
            if name == camera_module.cv2.CAP_PROP_FOURCC:
                return float(camera_module.cv2.VideoWriter_fourcc(*"MJPG"))
            return self.properties.get(name, 0.0)

        def isOpened(self):
            return self.opened

        def read(self):
            if not self.opened:
                return False, None
            return True, np.zeros((720, 1280, 3), dtype=np.uint8)

        def release(self):
            self.released = True

    monkeypatch.setattr(
        camera_module,
        "camera_candidates",
        lambda _configured: ("/dev/video0", "/dev/video1"),
    )
    monkeypatch.setattr(camera_module.cv2, "VideoCapture", FakeCapture)

    capture = open_camera(OnboardConfig())
    ok, frame = capture.read()

    assert ok and frame.shape == (720, 1280, 3)
    assert capture.device == "/dev/video1"
    assert capture.width == 1280
    assert capture.height == 720
    assert capture.fps == 30.0
    assert capture.pixel_format == "MJPG"
    capture.release()


def test_flight_controller_normalizations_still_match_pinned_mission():
    expected = load_qgc_waypoints(
        ROOT / "config/route_takeoff_search.waypoints"
    )
    actual = list(expected)
    actual[0] = replace(actual[0], current=0, x=0.0, y=0.0, z=0.0)
    actual[6] = replace(actual[6], frame=0)
    actual[11] = replace(actual[11], frame=0)
    actual[13] = replace(actual[13], param3=-58.0)
    actual[15] = replace(actual[15], frame=0)
    actual[19] = replace(actual[19], param4=1.0)
    errors, report = compare_mission_reference(actual, expected)
    assert not errors
    assert report["matched"]
    assert report["actual_count"] == report["expected_count"] == 20


def test_active_full_flow_route_satisfies_contract_and_home_boundary():
    route = load_qgc_waypoints(
        ROOT / "config/route_active_mode.waypoints"
    )
    home = route[0]
    assessment = assess_mission(
        route,
        payload_servo=8,
        min_release_pwm=1800.0,
        origin_lat_deg=home.x,
        origin_lon_deg=home.y,
        max_home_distance_m=300.0,
    )
    assert assessment.mode == "DYNAMIC_READY"
    assert assessment.contract is not None
    assert assessment.contract.fallback_release_seq == 23
    fallback_release = route[assessment.contract.fallback_release_seq]
    assert fallback_release.command == 183
    assert fallback_release.param1 == 8
    assert fallback_release.param2 == 1800.0
    assert not assessment.errors


def test_differently_formatted_file_with_same_route_is_semantically_accepted(
    tmp_path: Path,
):
    source = ROOT / "config/route_takeoff_search.waypoints"
    original_payload = source.read_bytes()
    lines = source.read_text(encoding="utf-8").splitlines()
    variant = tmp_path / "same-route-different-format.waypoints"
    variant.write_text(
        "QGC WPL 110\n\n"
        + "\n".join("    ".join(line.split()) for line in lines[1:])
        + "\n\n",
        encoding="utf-8",
    )

    assert hashlib.sha256(original_payload).digest() != hashlib.sha256(
        variant.read_bytes()
    ).digest()
    expected = load_qgc_waypoints(source)
    actual = load_qgc_waypoints(variant)
    errors, report = compare_mission_reference(actual, expected)
    assert not errors
    assert report["matched"] is True


def test_pinned_mission_rejects_a_different_waypoint():
    expected = load_qgc_waypoints(
        ROOT / "config/route_takeoff_search.waypoints"
    )
    actual = list(expected)
    actual[8] = replace(actual[8], x=actual[8].x + 0.0001)
    errors, report = compare_mission_reference(
        actual, expected, max_position_error_m=2.0
    )
    assert errors
    assert "seq 8 position differs" in "; ".join(errors)
    assert not report["matched"]


def test_shadow_release_is_only_the_exact_zero_nav_delay_form():
    expected = load_qgc_waypoints(
        ROOT / "config/route_takeoff_search.waypoints"
    )[15]
    neutral = neutralize_release_item(expected)
    assert is_shadow_neutralization(neutral, expected)
    assert not is_shadow_neutralization(replace(neutral, param1=0.1), expected)


def test_runtime_rejects_stale_patch_backup_for_another_mission(tmp_path: Path):
    config = pinned_repository_config()
    runtime = OnboardRuntime(
        replace(config, runtime=RuntimeConfig(directory=str(tmp_path)))
    )
    expected = runtime.expected_mission
    neutral_live = list(expected)
    neutral_live[15] = neutralize_release_item(expected[15])
    semantic, errors, substitutions = runtime._semantic_mission(
        neutral_live, [expected[15]]
    )
    assert not errors
    assert substitutions == [15]
    assert semantic[15].command == mavutil.mavlink.MAV_CMD_DO_SET_SERVO
    stale = replace(expected[15], param2=1900.0)
    _semantic, stale_errors, _substitutions = runtime._semantic_mission(
        neutral_live, [stale]
    )
    assert stale_errors


def test_active_only_runtime_rejects_disabled_pinned_mode_without_writes(tmp_path: Path):
    config = pinned_repository_config()
    runtime = OnboardRuntime(
        replace(config, runtime=RuntimeConfig(directory=str(tmp_path)))
    )
    expected = runtime.expected_mission
    live = list(expected)
    live[0] = replace(live[0], current=0, x=0.0, y=0.0, z=0.0)
    live[6] = replace(live[6], frame=0)
    live[11] = replace(live[11], frame=0)
    live[13] = replace(live[13], param3=-58.0)
    live[15] = neutralize_release_item(expected[15])
    live[19] = replace(live[19], param4=1.0)
    save_path = runtime.backup_path
    save_patch_backup(save_path, [expected[15]])

    class Client:
        @staticmethod
        def download_mission(timeout: float):
            assert timeout == 10.0
            return list(live)

        @staticmethod
        def write_partial(_items):
            raise AssertionError("an already neutralized mission must remain read-only")

        @staticmethod
        def status_text(_text, _severity):
            return None

    runtime.client = Client()
    runtime._refresh_mission()
    assert runtime.contract is None
    assert runtime.state["status"] == "MISSION_INVALID"
    assert runtime.state["mission_reference"]["matched"] is False
    assert runtime.state["mission_reference"]["live_count"] == 20
    assert any(
        "active-only runtime requires release.enabled=true" in error
        for error in runtime.state["mission_errors"]
    )

    live[8] = replace(live[8], x=live[8].x + 0.001)
    runtime._refresh_mission()
    assert runtime.contract is None
    assert runtime.state["status"] == "MISSION_INVALID"
    assert any("seq 8 position differs" in error for error in runtime.state["mission_errors"])


def test_dynamic_runtime_passes_through_an_ordinary_mission_without_writes(
    tmp_path: Path,
):
    base = active_test_config()
    config = replace(
        base,
        mission=replace(base.mission, stable_readbacks=1),
        runtime=RuntimeConfig(directory=str(tmp_path)),
    )
    runtime = OnboardRuntime(config)
    ordinary = [
        MissionItem(seq=0, frame=0, command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT),
        MissionItem(
            seq=1,
            frame=3,
            command=mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            x=29.0,
            y=115.0,
            z=20.0,
        ),
        MissionItem(
            seq=2,
            frame=3,
            command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            x=29.001,
            y=115.001,
            z=30.0,
        ),
        MissionItem(
            seq=3,
            frame=3,
            command=mavutil.mavlink.MAV_CMD_NAV_LAND,
            x=29.0,
            y=115.0,
        ),
    ]

    class Client:
        @staticmethod
        def download_mission(timeout: float):
            assert timeout == 10.0
            return list(ordinary)

        @staticmethod
        def write_partial(_items):
            raise AssertionError("ordinary missions must remain read-only")

        @staticmethod
        def status_text(_text, _severity):
            return None

    runtime.client = Client()
    runtime._refresh_mission()

    assert runtime.state["status"] == "MISSION_INVALID"
    assert runtime.state["mission_mode"] == "BLOCKED"
    assert runtime.contract is None
    assert runtime.mission == ordinary
    assert any("JUMP_TAG 9101" in error for error in runtime.state["mission_errors"])


def test_dynamic_runtime_rejects_disabled_payload_contract_without_writes(
    tmp_path: Path,
):
    base = active_test_config()
    config = replace(
        base,
        release=replace(base.release, enabled=False),
        mission=replace(base.mission, stable_readbacks=1),
        runtime=RuntimeConfig(directory=str(tmp_path)),
    )
    runtime = OnboardRuntime(config)
    original = load_qgc_waypoints(
        ROOT / "config/route_takeoff_search.waypoints"
    )
    inserted = replace(
        original[5],
        seq=0,
        x=original[5].x + 0.0002,
    )
    live = resequence([*original[:6], inserted, *original[6:]])
    writes: list[list[MissionItem]] = []

    class Client:
        @staticmethod
        def download_mission(timeout: float):
            assert timeout == 10.0
            return list(live)

        @staticmethod
        def write_partial(items):
            patch = list(items)
            writes.append(patch)
            for item in patch:
                live[item.seq] = item

        @staticmethod
        def status_text(_text, _severity):
            return None

    runtime.client = Client()
    runtime._refresh_mission()

    assert runtime.state["status"] == "MISSION_INVALID"
    assert runtime.state["mission_mode"] == "BLOCKED"
    assert len(runtime.mission) == 21
    assert runtime.contract is None
    assert writes == []
    assert runtime.mission[16].command == mavutil.mavlink.MAV_CMD_DO_SET_SERVO
    assert runtime.mission[18:] == resequence([*original[:6], inserted, *original[6:]])[18:]
    assert any(
        "active-only runtime requires release.enabled=true" in error
        for error in runtime.state["mission_errors"]
    )


def test_dynamic_runtime_allows_malformed_tagged_route_but_disables_payload(
    tmp_path: Path,
):
    base = active_test_config()
    config = replace(
        base,
        mission=replace(base.mission, stable_readbacks=1),
        runtime=RuntimeConfig(directory=str(tmp_path)),
    )
    runtime = OnboardRuntime(config)
    live = onboard_mission()
    live = [
        item
        for item in live
        if not (
            item.command == mavutil.mavlink.MAV_CMD_JUMP_TAG
            and int(round(item.param1)) == 9102
        )
    ]
    live = resequence(live)

    class Client:
        @staticmethod
        def download_mission(timeout: float):
            return list(live)

        @staticmethod
        def write_partial(_items):
            raise AssertionError("malformed payload contracts must remain read-only")

        @staticmethod
        def status_text(_text, _severity):
            return None

    runtime.client = Client()
    runtime._refresh_mission()

    assert runtime.state["status"] == "MISSION_INVALID"
    assert runtime.state["mission_mode"] == "BLOCKED"
    assert runtime.contract is None
    assert any("9102" in error for error in runtime.state["mission_errors"])


def test_dynamic_runtime_waits_for_two_identical_readbacks(tmp_path: Path):
    base = active_test_config()
    runtime = OnboardRuntime(
        replace(base, runtime=RuntimeConfig(directory=str(tmp_path)))
    )
    ordinary = [
        MissionItem(seq=0, frame=0, command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT),
        MissionItem(seq=1, frame=3, command=mavutil.mavlink.MAV_CMD_NAV_LAND),
    ]

    class Client:
        @staticmethod
        def download_mission(timeout: float):
            return list(ordinary)

        @staticmethod
        def write_partial(_items):
            raise AssertionError("ordinary missions must remain read-only")

        @staticmethod
        def status_text(_text, _severity):
            return None

    runtime.client = Client()
    runtime._refresh_mission()
    assert runtime.state["status"] == "MISSION_UPDATE_PENDING"
    assert runtime.contract is None

    runtime._refresh_mission()
    assert runtime.state["status"] == "MISSION_INVALID"


def test_mission_messages_are_component_addressed():
    client = MavlinkClient()
    client.target_system = 1
    client.target_component = 1
    client.connection = SimpleNamespace(
        mav=SimpleNamespace(srcSystem=1, srcComponent=191)
    )

    class Message:
        target_system = 1
        target_component = 191
        mission_type = mavutil.mavlink.MAV_MISSION_TYPE_MISSION

        @staticmethod
        def get_srcSystem():
            return 1

        @staticmethod
        def get_srcComponent():
            return 1

    assert client.mission_message_is_for_this_client(Message())
    other_gcs = Message()
    other_gcs.target_component = 190
    assert not client.mission_message_is_for_this_client(other_gcs)


class FlightControllerStatusText:
    def __init__(
        self,
        text: str,
        severity: int = mavutil.mavlink.MAV_SEVERITY_CRITICAL,
        system: int = 1,
        component: int = mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
        message_id: int = 0,
        chunk_seq: int = 0,
    ):
        self.text = text
        self.severity = severity
        self.system = system
        self.component = component
        self.id = message_id
        self.chunk_seq = chunk_seq

    @staticmethod
    def get_type() -> str:
        return "STATUSTEXT"

    def get_srcSystem(self) -> int:
        return self.system

    def get_srcComponent(self) -> int:
        return self.component


def test_runtime_latches_only_target_autopilot_crashdump_and_expires_prearm(
    tmp_path: Path,
):
    config = replace(OnboardConfig(), runtime=RuntimeConfig(directory=str(tmp_path)))
    runtime = OnboardRuntime(config)
    runtime.client = SimpleNamespace(target_system=1, target_component=1)
    runtime._set_status("READY", "mission and vision are ready")

    runtime._handle_message(
        FlightControllerStatusText(
            "CrashDump data detected",
            system=255,
            component=mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER,
        )
    )
    assert runtime.state["status"] == "READY"
    assert not runtime.state["flight_controller_health"]["release_blocked"]

    runtime._handle_message(
        FlightControllerStatusText("PreArm: GPS 1: Bad fix")
    )
    assert runtime.state["status"] == "FLIGHT_CONTROLLER_HEALTH_FAULT"
    assert runtime.state["flight_controller_health"]["release_blocked"]
    issue = next(iter(runtime.flight_controller_health_issues.values()))
    runtime._sync_flight_controller_health_state(
        issue["_last_seen_monotonic"]
        + FLIGHT_CONTROLLER_HEALTH_WARNING_TTL_S
        + 0.01
    )
    assert runtime.state["status"] == "READY"
    assert not runtime.state["flight_controller_health"]["issues"]

    runtime._handle_message(
        FlightControllerStatusText(b"CrashDump data detected\x00")
    )
    crash = next(iter(runtime.flight_controller_health_issues.values()))
    runtime._sync_flight_controller_health_state(
        crash["_last_seen_monotonic"]
        + FLIGHT_CONTROLLER_HEALTH_WARNING_TTL_S
        + 600.0
    )
    runtime._set_status("READY", "a later mission refresh reported ready")
    health = runtime.state["flight_controller_health"]
    assert runtime.state["status"] == "FLIGHT_CONTROLLER_HEALTH_FAULT"
    assert health["crash_dump_detected"] is True
    assert health["release_blocked"] is True
    assert health["issues"][0]["latched"] is True
    assert "_last_seen_monotonic" not in health["issues"][0]


def test_runtime_shutdown_immediately_signals_camera_worker(tmp_path: Path):
    config = replace(OnboardConfig(), runtime=RuntimeConfig(directory=str(tmp_path)))
    runtime = OnboardRuntime(config)

    assert runtime.stop_requested is False
    assert runtime.vision.stop_event.is_set() is False
    runtime.shutdown()
    assert runtime.stop_requested is True
    assert runtime.vision.stop_event.is_set() is True


def test_runtime_reassembles_chunked_flight_controller_status_text(tmp_path: Path):
    config = replace(OnboardConfig(), runtime=RuntimeConfig(directory=str(tmp_path)))
    runtime = OnboardRuntime(config)
    runtime.client = SimpleNamespace(target_system=1, target_component=1)

    full_text = "PreArm: AHRS: EKF3 Yaw inconsistent 71 deg. Wait or reboot"
    first = full_text[:50]
    second = full_text[50:]
    assert len(first) == 50
    runtime._handle_message(
        FlightControllerStatusText(first, message_id=23, chunk_seq=0)
    )
    assert not runtime.state["flight_controller_health"]["issues"]

    runtime._handle_message(
        FlightControllerStatusText(second, message_id=23, chunk_seq=1)
    )
    issues = runtime.state["flight_controller_health"]["issues"]
    assert len(issues) == 1
    assert issues[0]["code"] == "PREARM_FAILURE"
    assert issues[0]["text"] == full_text
    assert not any(issue["text"] == "r reboot" for issue in issues)


def test_runtime_ignores_orphaned_status_text_continuation(tmp_path: Path):
    config = replace(OnboardConfig(), runtime=RuntimeConfig(directory=str(tmp_path)))
    runtime = OnboardRuntime(config)
    runtime.client = SimpleNamespace(target_system=1, target_component=1)

    runtime._handle_message(
        FlightControllerStatusText("r reboot", message_id=99, chunk_seq=1)
    )
    assert not runtime.state["flight_controller_health"]["issues"]


def test_runtime_classifies_safety_prefix_when_status_continuation_is_lost(
    tmp_path: Path,
):
    config = replace(OnboardConfig(), runtime=RuntimeConfig(directory=str(tmp_path)))
    runtime = OnboardRuntime(config)
    runtime.client = SimpleNamespace(target_system=1, target_component=1)

    first = "PreArm: GPS configuration invalid; continuation miss"[:50]
    assert len(first) == 50
    runtime._handle_message(
        FlightControllerStatusText(first, message_id=41, chunk_seq=0)
    )
    pending = next(iter(runtime._flight_controller_status_text_fragments.values()))
    runtime._sync_flight_controller_health_state(
        pending["updated_at"] + STATUSTEXT_FRAGMENT_TTL_S + 0.01
    )

    issues = runtime.state["flight_controller_health"]["issues"]
    assert issues[0]["code"] == "PREARM_FAILURE"
    assert issues[0]["text"] == first


def test_runtime_classifies_safety_prefix_on_out_of_order_continuation(
    tmp_path: Path,
):
    config = replace(OnboardConfig(), runtime=RuntimeConfig(directory=str(tmp_path)))
    runtime = OnboardRuntime(config)
    runtime.client = SimpleNamespace(target_system=1, target_component=1)

    first = "PreArm: GPS configuration invalid; continuation miss"[:50]
    runtime._handle_message(
        FlightControllerStatusText(first, message_id=42, chunk_seq=0)
    )
    runtime._handle_message(
        FlightControllerStatusText("late chunk", message_id=42, chunk_seq=2)
    )

    issues = runtime.state["flight_controller_health"]["issues"]
    assert issues[0]["code"] == "PREARM_FAILURE"
    assert issues[0]["text"] == first


def test_connect_ignores_ground_station_heartbeat(monkeypatch):
    class Heartbeat:
        def __init__(self, system: int, component: int, autopilot: int):
            self.system = system
            self.component = component
            self.autopilot = autopilot
            self.base_mode = 0
            self.custom_mode = 0

        def get_srcSystem(self):
            return self.system

        def get_srcComponent(self):
            return self.component

    ground_station = Heartbeat(
        255,
        mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
    )
    flight_controller = Heartbeat(
        1,
        mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
        mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
    )

    class Connection:
        def __init__(self):
            self.messages = [ground_station, flight_controller]
            self.target_system = 0
            self.target_component = 0
            self.mav = SimpleNamespace(srcSystem=191, srcComponent=191)

        def recv_match(self, **_kwargs):
            return self.messages.pop(0) if self.messages else None

        def close(self):
            return None

    connection = Connection()
    monkeypatch.setattr(
        mavutil, "mavlink_connection", lambda *_args, **_kwargs: connection
    )
    client = MavlinkClient("memory:test")
    client.connect(timeout=1.0)
    assert client.initial_heartbeat is flight_controller
    assert client.target_system == 1
    assert client.target_component == 1
    assert connection.mav.srcSystem == 1
    assert connection.mav.srcComponent == 191
    assert connection.source_system == 1
    assert connection.source_component == 191


def test_runtime_accepts_flight_state_only_from_target_autopilot(tmp_path: Path):
    config = replace(OnboardConfig(), runtime=RuntimeConfig(directory=str(tmp_path)))
    runtime = OnboardRuntime(config)

    class Client:
        target_system = 1
        target_component = mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
        is_armed = staticmethod(MavlinkClient.is_armed)
        is_auto = staticmethod(MavlinkClient.is_auto)

    class Heartbeat:
        def __init__(self, system: int, component: int):
            self.system = system
            self.component = component
            self.base_mode = (
                mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                | mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            )
            self.custom_mode = 10

        @staticmethod
        def get_type() -> str:
            return "HEARTBEAT"

        def get_srcSystem(self) -> int:
            return self.system

        def get_srcComponent(self) -> int:
            return self.component

    runtime.client = Client()
    runtime._handle_message(
        Heartbeat(1, mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER)
    )
    assert runtime.telemetry.armed is False
    assert runtime.telemetry.auto is False

    runtime._handle_message(
        Heartbeat(1, mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1)
    )
    assert runtime.telemetry.armed is True
    assert runtime.telemetry.auto is True


def test_router_endpoint_must_match_companion_listener():
    base = OnboardConfig()
    invalid = replace(
        base,
        flight_controller=replace(
            base.flight_controller,
            endpoint="udpin:127.0.0.1:14552",
        ),
    )
    with pytest.raises(ValueError, match="must match the router companion endpoint"):
        invalid.validate()


def test_runtime_log_rotation_is_bounded(tmp_path: Path):
    path = tmp_path / "telemetry.jsonl"
    path.write_bytes(b"123456")
    rotate_file(path, max_bytes=4, backups=2)
    assert not path.exists()
    assert (tmp_path / "telemetry.jsonl.1").read_bytes() == b"123456"
    path.write_bytes(b"abcdef")
    rotate_file(path, max_bytes=4, backups=2)
    assert (tmp_path / "telemetry.jsonl.1").read_bytes() == b"abcdef"
    assert (tmp_path / "telemetry.jsonl.2").read_bytes() == b"123456"


def test_tlog_writer_records_complete_mavlink_frames(tmp_path: Path):
    writer = TlogWriter(tmp_path, max_mb=1, retained_files=2)
    encoder = mavutil.mavlink.MAVLink(None)
    message = encoder.heartbeat_encode(
        mavutil.mavlink.MAV_TYPE_FIXED_WING,
        mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
        0,
        0,
        mavutil.mavlink.MAV_STATE_ACTIVE,
    )
    frame = message.pack(encoder)
    assert writer.append(frame) == 1
    path = writer.path
    writer.close()
    assert path is not None
    payload = path.read_bytes()
    assert len(payload) == 8 + len(frame)
    assert payload[8:] == frame


def test_unknown_onboard_setting_is_rejected(tmp_path: Path):
    path = tmp_path / "bad.yaml"
    path.write_text("release:\n  enabled: false\n  magic: 1\n", encoding="utf-8")
    with pytest.raises(ValueError, match="unknown release settings"):
        load_config(path)


def test_independent_servo_function_is_allowed_and_unsupported_function_is_rejected():
    base = OnboardConfig()
    replace(
        base,
        release=replace(base.release, required_servo_function=0),
    ).validate()

    replace(
        base,
        release=replace(base.release, required_servo_function=58),
    ).validate()

    for function in (1, 2, 57, 59, 70):
        unsupported = replace(
            base,
            release=replace(base.release, required_servo_function=function),
        )
        with pytest.raises(ValueError, match="must be 0"):
            unsupported.validate()


@pytest.mark.parametrize("function", [0, 58])
def test_active_release_requires_explicit_calibration_confirmations(function):
    base = OnboardConfig()
    base = replace(base, release=replace(base.release, required_servo_function=function))
    active = replace(base, release=replace(base.release, enabled=True))
    with pytest.raises(ValueError, match="completed calibration"):
        active.validate()
    calibrated = replace(
        base,
        camera=replace(
            base.camera,
            device="/dev/v4l/by-id/usb-calibrated-camera-video-index0",
            calibration_confirmed=True,
            fx_px=640.0,
            fy_px=640.0,
            cx_px=640.0,
            cy_px=360.0,
        ),
        vision=replace(base.vision, validation_confirmed=True),
        release=replace(
            base.release,
            enabled=True,
            ballistics_calibrated=True,
        ),
    )
    calibrated.validate()


def test_active_release_rejects_auto_camera_even_with_calibration():
    base = OnboardConfig()
    unsafe = replace(
        base,
        camera=replace(
            base.camera,
            calibration_confirmed=True,
            fx_px=640.0,
            fy_px=640.0,
            cx_px=640.0,
            cy_px=360.0,
        ),
        vision=replace(base.vision, validation_confirmed=True),
        release=replace(
            base.release,
            enabled=True,
            ballistics_calibrated=True,
        ),
    )

    with pytest.raises(ValueError, match="fixed to the calibrated camera"):
        unsafe.validate()


def test_filename_label_parser_supports_both_real_validation_tasks():
    values = {"bomber": 12, "air_defense": 6}
    assert expected_value_from_path(Path("flight_bomber_01.jpg"), 1, values) == 12
    assert expected_value_from_path(Path("frame_number-07.jpg"), 2, values) == 7
    assert expected_value_from_path(Path("task2_76.jpg"), 2, values) == 76
    assert expected_value_from_path(Path("unlabelled.jpg"), 2, values) is None


def test_real_mission_contract_accepts_channel_8_pwm_1800():
    contract, errors = validate_mission(
        onboard_mission(), payload_servo=8, min_release_pwm=1800
    )
    assert contract is not None
    assert not errors
    _, wrong_channel = validate_mission(
        onboard_mission(), payload_servo=9, min_release_pwm=1800
    )
    assert wrong_channel


def test_real_patch_keeps_fallback_on_continuous_target_bound_line():
    mission = onboard_mission()
    contract, errors = validate_mission(
        mission, payload_servo=8, min_release_pwm=1800
    )
    assert contract is not None and not errors
    patch = build_attack_patch(
        mission,
        contract,
        target_north_m=200.0,
        target_east_m=0.0,
        current_north_m=220.0,
        current_east_m=0.0,
        nominal_speed_mps=20.0,
        nominal_agl_m=20.0,
        max_home_distance_m=300.0,
        planning_bank_deg=35.0,
        origin_lat_deg=ORIGIN_LAT_DEG,
        origin_lon_deg=ORIGIN_LON_DEG,
        payload_servo=8,
        release_pwm=1800,
    )
    fallback_north, fallback_east = latlon_to_ne(
        patch[1].x,
        patch[1].y,
        ORIGIN_LAT_DEG,
        ORIGIN_LON_DEG,
    )
    assert math.hypot(fallback_north - 200.0, fallback_east) == pytest.approx(
        0.0, abs=0.01
    )
    assert patch[1].param2 == 3.0
    assert patch[2].param1 == 8
    assert patch[2].param2 == 1800


def test_20mps_turn_is_rejected_when_scan_guard_is_too_far_from_home():
    mission = onboard_mission()
    contract, errors = validate_mission(
        mission, payload_servo=8, min_release_pwm=1800
    )
    assert contract is not None and not errors
    with pytest.raises(ValueError, match="half-turn fits"):
        build_attack_patch(
            mission,
            contract,
            target_north_m=200.0,
            target_east_m=0.0,
            current_north_m=240.0,
            current_east_m=0.0,
            nominal_speed_mps=20.0,
            nominal_agl_m=20.0,
            max_home_distance_m=300.0,
            planning_bank_deg=35.0,
        )


def test_actuator_delay_moves_command_earlier_along_velocity():
    base = solve_release(200.0, 0.0, 20.0, 20.0, 0.0, drag_rate_s=0.0)
    delayed = solution_with_actuator_delay(base, 20.0, 0.0, 0.15)
    assert delayed.release_north_m == pytest.approx(base.release_north_m - 3.0)
    assert delayed.north_lead_m == pytest.approx(base.north_lead_m + 3.0)


def test_shadow_mode_neutralizes_autopilot_fallback_without_a_position():
    item = onboard_mission()[9]
    neutral = neutralize_release_item(item)
    assert neutral.command == mavutil.mavlink.MAV_CMD_NAV_DELAY
    assert neutral.param1 == neutral.param2 == 0.0
    assert neutral.x == neutral.y == neutral.z == 0.0


@pytest.mark.parametrize("function", [0, 58])
def test_release_gate_requires_explicit_enable_and_fresh_safe_flight_state(function):
    mission = onboard_mission()
    contract, errors = validate_mission(
        mission, payload_servo=8, min_release_pwm=1800
    )
    assert contract is not None and not errors
    telemetry = Telemetry(
        relative_alt_m=20.0,
        velocity_north_mps=20.0,
        airspeed_mps=20.0,
        roll_rad=math.radians(5.0),
        armed=True,
        auto=True,
        mission_seq=contract.fallback_start_seq + 1,
    )
    shadow = OnboardConfig()
    shadow = replace(shadow, release=replace(shadow.release, required_servo_function=function))
    reasons = release_gate_reasons(
        shadow, telemetry, contract, 0.1, 0.1, 0.1, 0.1, 3, 10, 1.0, True, True, 200.0, 0.0
    )
    assert "release disabled by configuration" in reasons
    active = replace(
        shadow,
        camera=replace(shadow.camera, calibration_confirmed=True),
        vision=replace(shadow.vision, validation_confirmed=True),
        release=replace(
            shadow.release,
            enabled=True,
            servo_channel=8,
            release_pwm=1800,
            ballistics_calibrated=True,
        ),
    )
    assert release_gate_reasons(
        active, telemetry, contract, 0.1, 0.1, 0.1, 0.1, 3, 10, 1.0, True, True, 200.0, 0.0
    ) == []
    loitering = replace(telemetry, mission_seq=contract.fallback_start_seq)
    assert "aircraft is outside the patched attack segment" in release_gate_reasons(
        active, loitering, contract, 0.1, 0.1, 0.1, 0.1, 3, 10, 1.0, True, True, 200.0, 0.0
    )
    stale = release_gate_reasons(
        active, telemetry, contract, 2.0, 0.1, 0.1, 0.1, 3, 10, 1.0, True, True, 200.0, 0.0
    )
    assert "heartbeat is stale" in stale
    away = replace(telemetry, velocity_north_mps=-20.0)
    assert "ground track is not closing on the selected target" in release_gate_reasons(
        active, away, contract, 0.1, 0.1, 0.1, 0.1, 3, 10, 1.0, True, True, 200.0, 0.0
    )
    turning = replace(telemetry, yaw_rate_rps=math.radians(13.0))
    assert "yaw rate is outside the release gate" in release_gate_reasons(
        active, turning, contract, 0.1, 0.1, 0.1, 0.1, 3, 10, 1.0, True, True, 200.0, 0.0
    )


@pytest.mark.parametrize("function", [0, 58])
def test_primary_release_commands_channel_8_once_inside_ballistic_window(tmp_path: Path, function):
    base = OnboardConfig()
    config = replace(
        base,
        camera=replace(base.camera, calibration_confirmed=True),
        vision=replace(base.vision, validation_confirmed=True),
        release=replace(
            base.release,
            enabled=True,
            servo_channel=8,
            release_pwm=1800,
            required_servo_function=function,
            ballistics_calibrated=True,
        ),
        runtime=RuntimeConfig(directory=str(tmp_path)),
    )
    runtime = OnboardRuntime(config)
    mission = onboard_mission()
    contract, errors = validate_mission(
        mission, payload_servo=8, min_release_pwm=1800
    )
    assert contract is not None and not errors
    runtime.contract = contract
    runtime.mission = mission
    runtime.home_valid = True
    runtime.gps_fix_type = 3
    runtime.satellites_visible = 10
    runtime.gps_hdop = 1.0
    runtime.servo_config_valid = True
    runtime.selected = Selection(
        "T01", 12, 200.0, 0.0, "test", "YOLO", guessed=False
    )
    runtime.telemetry = Telemetry(
        relative_alt_m=20.0,
        velocity_north_mps=20.0,
        velocity_east_mps=0.0,
        airspeed_mps=20.0,
        roll_rad=0.0,
        armed=True,
        auto=True,
        mission_seq=contract.fallback_start_seq + 1,
    )
    solution = runtime._actual_release_solution()
    assert solution is not None
    runtime.telemetry.north_m = solution.release_north_m
    runtime.telemetry.east_m = solution.release_east_m
    now = __import__("time").monotonic()
    runtime.last_heartbeat = now
    runtime.last_position = now
    runtime.last_attitude = now
    runtime.last_airspeed = now

    commands: list[tuple[int, int]] = []

    class Client:
        target_system = 1
        target_component = mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1

        @staticmethod
        def set_servo(channel: int, pwm: int) -> None:
            commands.append((channel, pwm))

    runtime.client = Client()
    runtime._handle_message(
        FlightControllerStatusText("PreArm: AHRS: EKF3 Yaw inconsistent")
    )
    runtime._try_primary_release()
    assert commands == []
    assert "flight controller reports an unresolved health fault" in runtime.state[
        "release_gate"
    ]
    issue = next(iter(runtime.flight_controller_health_issues.values()))
    runtime._sync_flight_controller_health_state(
        issue["_last_seen_monotonic"]
        + FLIGHT_CONTROLLER_HEALTH_WARNING_TTL_S
        + 0.01
    )
    runtime._try_primary_release()
    runtime._try_primary_release()
    assert commands == [(8, 1800)]
    assert runtime.state["release"]["source"] == "PRIMARY_DYNAMIC_BALLISTIC"


def test_raspberry_pi_state_is_encoded_as_standard_mavlink_named_values(tmp_path: Path):
    base = OnboardConfig()
    config = replace(base, runtime=RuntimeConfig(directory=str(tmp_path)))
    runtime = OnboardRuntime(config)
    runtime.gps_fix_type = 3
    runtime.satellites_visible = 12
    runtime.telemetry.relative_alt_m = 20.5
    runtime.telemetry.airspeed_mps = 19.8
    runtime.telemetry.mission_seq = 7
    integer_reports: dict[str, int] = {}
    float_reports: dict[str, float] = {}
    heartbeats: list[int] = []

    class Client:
        @staticmethod
        def companion_heartbeat(status_code: int) -> None:
            heartbeats.append(status_code)

        @staticmethod
        def named_value_int(name: str, value: int, _time_ms: int) -> None:
            integer_reports[name] = value

        @staticmethod
        def named_value_float(name: str, value: float, _time_ms: int) -> None:
            float_reports[name] = value

    runtime.client = Client()
    runtime._publish_mavlink_report(
        {
            "status": "READY",
            "mission_mode": "PAYLOAD",
            "selected_target": {"value": 0, "north_m": 120.0, "east_m": -5.0},
            "vision": {
                "inference_count": 42,
                "camera_connected": True,
                "camera_auto": True,
                "camera_width": 1280,
                "camera_height": 720,
                "camera_fps": 30.0,
                "detections": [
                    {
                        "value": 76,
                        "confidence": 0.88,
                        "north_m": 118.0,
                        "east_m": -4.0,
                    }
                ],
            },
        },
        force=True,
    )
    assert heartbeats == [ONBOARD_STATUS_CODES["READY"]]
    assert integer_reports["CU_TASK"] == 1
    assert integer_reports["CU_DETVAL"] == 76
    assert integer_reports["CU_SELVAL"] == 0
    assert integer_reports["CU_GPSFIX"] == 3
    assert integer_reports["CU_CAMOK"] == 1
    assert integer_reports["CU_CAMAUTO"] == 1
    assert integer_reports["CU_CAMW"] == 1280
    assert integer_reports["CU_CAMH"] == 720
    assert integer_reports["CU_CAMFPS"] == 30
    assert integer_reports["CU_MPASS"] == 1
    assert integer_reports["CU_MMODE"] == 2
    assert float_reports["CU_DETCF"] == pytest.approx(0.88)
    assert float_reports["CU_SELN"] == pytest.approx(120.0)
    assert float_reports["CU_ALT"] == pytest.approx(20.5)


def test_mavlink_report_messages_are_wire_compatible():
    class Sink:
        def __init__(self):
            self.payload = bytearray()

        def write(self, data: bytes) -> int:
            self.payload.extend(data)
            return len(data)

    sink = Sink()
    encoder = mavutil.mavlink.MAVLink(sink, srcSystem=1, srcComponent=191)
    client = MavlinkClient()
    client.connection = SimpleNamespace(mav=encoder)
    client.companion_heartbeat(ONBOARD_STATUS_CODES["READY"])
    client.named_value_int("CU_TASK", 2, 1234)
    client.named_value_float("CU_DETCF", 0.88, 1234)

    parser = mavutil.mavlink.MAVLink(None)
    messages = parser.parse_buffer(bytes(sink.payload)) or []
    assert [message.get_type() for message in messages] == [
        "HEARTBEAT",
        "NAMED_VALUE_INT",
        "NAMED_VALUE_FLOAT",
    ]
    assert all(message.get_srcComponent() == 191 for message in messages)
    assert messages[0].custom_mode == ONBOARD_STATUS_CODES["READY"]
    assert messages[1].name == "CU_TASK"
    assert messages[1].value == 2
    assert messages[2].name == "CU_DETCF"
    assert messages[2].value == pytest.approx(0.88, abs=1e-6)
