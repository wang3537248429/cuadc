import math
import json
import time
from types import SimpleNamespace

import pytest
from pymavlink import mavutil

from cuadc_sim import companion
from cuadc_sim.companion import CompanionRuntime
from cuadc_sim.config import SimParameters
from cuadc_sim.mission import (
    FixedFallbackContract,
    MissionContract,
    MissionItem,
    ne_to_latlon,
    RawReplayContract,
)
from cuadc_sim.results import ResultStore
from cuadc_sim.selection import Selection


@pytest.fixture(autouse=True)
def isolated_field_layout(monkeypatch):
    from cuadc_sim.field_layout import FieldLayout

    monkeypatch.setattr(companion, "load_field_layout", FieldLayout)


def test_disarmed_parameter_change_starts_an_isolated_run(tmp_path, monkeypatch):
    runtime_dir = tmp_path / "runtime"
    runs_dir = tmp_path / "runs"
    runtime_dir.mkdir()
    monkeypatch.setattr(companion, "RUNTIME_DIR", runtime_dir)
    monkeypatch.setattr(
        companion,
        "write_runtime",
        lambda parameters: ({"seed": parameters.seed}, runtime_dir / "world.sdf"),
    )
    monkeypatch.setattr(
        companion,
        "ResultStore",
        lambda result: ResultStore(result, runs_dir),
    )

    for name in (
        "payload_released.flag",
        "impact.json",
        "touchdown.json",
        "vision_state.json",
        "vision_frame.jpg",
    ):
        (runtime_dir / name).write_text("stale", encoding="utf-8")

    runtime = CompanionRuntime("udp:127.0.0.1:1", dry_run=True)
    runtime.mission = [object()]
    runtime.contract = object()
    runtime.last_end_seq_processed = 9
    runtime.launched = True
    runtime.release_commanded = True
    runtime.released = True
    runtime.completed = True
    runtime.selected = Selection("T01", 12, 200, 0, "old", "YOLO")
    runtime.previous_along_track = -4.0
    runtime.last_mission_attempt = 123.0

    updated = SimParameters(task=2, group="B", seed=47)
    runtime._reset_run(updated)

    assert runtime.parameters == updated
    assert runtime.mission == []
    assert runtime.contract is None
    assert runtime.last_end_seq_processed == -1
    assert not runtime.launched
    assert not runtime.release_commanded
    assert not runtime.released
    assert not runtime.completed
    assert runtime.selected is None
    assert math.isinf(runtime.previous_along_track)
    assert runtime.last_mission_attempt == 0.0
    assert runtime.result.task == 2
    assert runtime.result.group == "B"
    assert runtime.result.seed == 47
    assert runtime.result.status == "WAITING_FOR_MISSION"
    assert all(not (runtime_dir / name).exists() for name in (
        "payload_released.flag",
        "impact.json",
        "touchdown.json",
        "vision_state.json",
        "vision_frame.jpg",
    ))


def test_mission_validation_waits_until_mission_planner_upload_is_quiet():
    runtime = CompanionRuntime("udp:127.0.0.1:1", dry_run=True)
    runtime.last_mission_attempt = 100.0

    request = SimpleNamespace(get_type=lambda: "MISSION_REQUEST_INT")
    runtime._note_mission_wire_activity(request, now=120.0)

    assert runtime.mission_refresh_requested
    assert not runtime._mission_refresh_due(now=121.9)
    assert runtime._mission_refresh_due(now=122.0)


@pytest.mark.parametrize('distance,expected',[(.34,True),(.36,False),(1.5,False)])
def test_dynamic_release_requires_the_narrow_command_window(monkeypatch,distance,expected):
    runtime=CompanionRuntime('udp:127.0.0.1:1',dry_run=True)
    runtime.contract=MissionContract(2,5,6,6,8,9,14)
    runtime.selected=Selection('T01',9,100,0,'test','YOLO')
    runtime.result.selected_target={'north_m':100,'east_m':0}
    runtime.telemetry.armed=True
    runtime.telemetry.mission_seq=7
    runtime.telemetry.relative_alt_m=16
    runtime.telemetry.velocity_north_mps=12.5
    runtime.telemetry.timestamp_s=0
    solution=SimpleNamespace(release_north_m=distance,release_east_m=0)
    monkeypatch.setattr(companion,'solve_release',lambda *a,**kw:solution)
    monkeypatch.setattr(companion,'solution_with_actuator_delay',lambda value,*a:value)
    commands=[]
    runtime._command_payload_release=lambda *a,**kw:commands.append(a)
    runtime._try_release()
    assert bool(commands)==expected


def test_empty_sitl_mission_does_not_create_or_restart_a_raw_replay(tmp_path, monkeypatch):
    monkeypatch.setattr(companion, "RUNTIME_DIR", tmp_path)
    runtime = CompanionRuntime("udp:127.0.0.1:1", dry_run=True)
    runtime.store = SimpleNamespace(save=lambda: None)
    runtime.client.download_mission = lambda timeout=15.0: []
    runtime._captured_uploaded_mission = lambda: None

    runtime._refresh_mission()

    assert runtime.result.status == "MISSION_INVALID"
    assert not (tmp_path / "mission_replay.json").exists()
    assert not (tmp_path / "restart.request").exists()


def test_completed_disarmed_run_resets_when_ground_station_starts_a_new_upload(
    tmp_path, monkeypatch
):
    runtime_dir = tmp_path / "runtime"
    runs_dir = tmp_path / "runs"
    runtime_dir.mkdir()
    monkeypatch.setattr(companion, "RUNTIME_DIR", runtime_dir)
    monkeypatch.setattr(
        companion,
        "write_runtime",
        lambda parameters: ({"seed": parameters.seed}, runtime_dir / "world.sdf"),
    )
    monkeypatch.setattr(
        companion,
        "ResultStore",
        lambda result: ResultStore(result, runs_dir),
    )

    runtime = CompanionRuntime("udp:127.0.0.1:1", dry_run=True)
    runtime.launched = True
    runtime.completed = True
    runtime.result.status = "COMPLETED"
    runtime.telemetry.armed = False
    upload_count = SimpleNamespace(
        get_type=lambda: "MISSION_COUNT",
        get_srcSystem=lambda: 253,
        count=30,
    )

    runtime._note_mission_wire_activity(upload_count, now=200.0)

    assert not runtime.launched
    assert not runtime.completed
    assert runtime.result.status == "WAITING_FOR_MISSION"
    assert runtime.uploaded_mission_count == 30
    assert runtime.uploaded_mission_items == {}


def test_internal_mission_count_does_not_reset_a_completed_flight():
    runtime = CompanionRuntime("udp:127.0.0.1:1", dry_run=True)
    runtime.launched = True
    runtime.completed = True
    runtime.result.status = "COMPLETED"
    internal_count = SimpleNamespace(
        get_type=lambda: "MISSION_COUNT",
        get_srcSystem=lambda: runtime.client.target_system,
        count=30,
    )

    runtime._note_mission_wire_activity(internal_count, now=200.0)

    assert runtime.launched
    assert runtime.completed
    assert runtime.result.status == "COMPLETED"


def test_vehicle_request_zero_marks_new_task_even_when_gcs_count_is_not_echoed():
    runtime=CompanionRuntime('udp:127.0.0.1:1',dry_run=True)
    request=SimpleNamespace(get_type=lambda:'MISSION_REQUEST_INT',
                            get_srcSystem=lambda:runtime.client.target_system,seq=0)
    runtime._note_mission_wire_activity(request)
    assert runtime.new_task_pending
    runtime.new_task_pending=False;runtime.telemetry.armed=True
    runtime._note_mission_wire_activity(request)
    assert not runtime.new_task_pending


def test_scan_decision_drains_last_inference_within_existing_decision_leg():
    from cuadc_sim.selection import Detection
    runtime=CompanionRuntime('udp:127.0.0.1:1',dry_run=True)
    runtime.contract=MissionContract(2,5,6,6,8,9,14)
    runtime.telemetry.mission_seq=5
    runtime.scan_end_pending_at=100
    values=[Detection('a',9,.9,frames=2),Detection('b',5,.9,frames=2,north_m=40)]
    runtime._detections=lambda:(values,'YOLO')
    calls=[];runtime._handle_scan_end=lambda:calls.append(True)
    runtime._poll_scan_decision(now=100.5)
    assert not calls
    values.append(Detection('c',4,.9,frames=2,north_m=80))
    runtime._poll_scan_decision(now=100.8)
    assert calls==[True] and runtime.scan_end_pending_at is None
    runtime.scan_end_pending_at=200;values.pop()
    runtime._poll_scan_decision(now=200+companion.SCAN_FINAL_FRAME_GRACE_S)
    assert len(calls)==2


def test_uploaded_route_hand_launch_uses_its_computed_course(monkeypatch):
    runtime = CompanionRuntime("udp:127.0.0.1:1", dry_run=True)
    runtime.contract = RawReplayContract(
        home_lat_deg=29.3,
        home_lon_deg=104.6,
        home_alt_m=46.0,
        first_flight_seq=1,
        land_seq=29,
        release_sequences=(16,),
        automatic_takeoff=False,
    )
    monkeypatch.setattr(
        companion,
        "load_field_layout",
        lambda: SimpleNamespace(initial_heading_deg=212.4),
    )

    assert runtime._launch_heading_deg() == 147.6


def test_builtin_hand_launch_keeps_group_heading():
    runtime = CompanionRuntime("udp:127.0.0.1:1", dry_run=True)
    runtime.parameters = SimParameters(group="B")

    assert runtime._launch_heading_deg() == 180.0


def test_parameter_comparison_ignores_float32_roundoff():
    expected = SimParameters().to_dict()
    observed = {**expected, "confidence": 0.20000000298023224}

    assert CompanionRuntime._parameters_match(observed, expected)
    assert not CompanionRuntime._parameters_match(
        {**observed, "launch_speed_mps": 14.0}, expected
    )


def test_idle_mission_safety_poll_is_slow_enough_not_to_churn_uploads():
    runtime = CompanionRuntime("udp:127.0.0.1:1", dry_run=True)
    runtime.last_mission_attempt = 100.0
    runtime.last_mission_wire_activity = 100.0

    assert not runtime._mission_refresh_due(now=159.9)
    assert runtime._mission_refresh_due(now=160.0)


def test_incomplete_scan_keeps_uploaded_route_without_an_automatic_retry():
    runtime = CompanionRuntime("udp:127.0.0.1:1", dry_run=True)
    runtime.store = SimpleNamespace(save=lambda: None)
    runtime.contract = FixedFallbackContract(2, 5, 3, 8, 12)
    runtime.scan_coverage_ratio = 0.55
    runtime._detections = lambda: ([], "CAMERA")
    requested: list[int] = []
    runtime.client.set_current = requested.append

    runtime._handle_scan_end()

    assert requested == []
    assert runtime.scan_attempt == 1
    assert runtime.result.status == "FALLBACK_ROUTE_ACTIVE"
    assert "fixed" in runtime.result.message.lower() or "payload" in runtime.result.message.lower()


def test_dynamic_drop_changes_only_reserved_slots_and_preserves_return_route(monkeypatch):
    from pathlib import Path
    from pymavlink import mavwp
    from cuadc_sim.mission import validate_mission
    from cuadc_sim.selection import Detection
    loader = mavwp.MAVWPLoader()
    loader.load(str(Path(__file__).resolve().parents[1]/'config/missions/recon_low_a.waypoints'))
    original = [MissionItem.from_message(loader.wp(index)) for index in range(loader.count())]
    runtime = CompanionRuntime('udp:127.0.0.1:1', dry_run=True)
    runtime.store = SimpleNamespace(save=lambda: None)
    runtime.mission = list(original)
    runtime.contract, errors = validate_mission(original)
    assert not errors
    runtime.scan_coverage_ratio = 1
    runtime._detections = lambda: ([Detection('', value, .9, frames=4, north_m=180+index*18,
                                             east_m=-17 if index == 2 else 17)
                                  for index, value in enumerate((4, 5, 9))], 'YOLO')
    monkeypatch.setattr(companion, 'save_patch_backup', lambda *_args: None)
    writes, jumps = [], []
    runtime.client.write_partial = lambda items: writes.extend(items)
    runtime.client.set_current = jumps.append
    runtime._handle_scan_end()
    assert runtime.result.status == 'ATTACK_ROUTE_ACTIVE'
    slots = set(range(runtime.contract.fallback_start_seq, runtime.contract.fallback_exit_seq+1))
    assert {item.seq for item in writes} == slots
    assert all(after == before for before, after in zip(original, runtime.mission) if before.seq not in slots)
    assert jumps == [runtime.contract.fallback_start_seq]


def test_release_slot_commands_single_release_when_ballistic_window_was_missed():
    runtime = CompanionRuntime("udp:127.0.0.1:1", dry_run=True)
    runtime.store = SimpleNamespace(save=lambda: None)
    runtime.contract = MissionContract(2, 5, 6, 6, 8, 9, 14)
    runtime.selected = Selection("T01", 12, 200, 0, "target", "YOLO")
    runtime.result.selected_target = {"north_m": 200.0, "east_m": 0.0}
    runtime.telemetry.armed = True
    runtime.telemetry.north_m = 211.0
    runtime.telemetry.east_m = 0.0
    runtime.telemetry.velocity_north_mps = -16.0

    runtime._try_mission_release_fallback(6)
    assert not runtime.release_commanded

    runtime._try_mission_release_fallback(7)
    assert runtime.release_commanded
    assert not runtime.released
    assert runtime.result.release["source"] == "MISSION_RELEASE_SLOT_FALLBACK"

    first_command = runtime.result.release
    runtime._try_mission_release_fallback(9)
    assert runtime.result.release is first_command


def test_raw_route_release_matches_only_uploaded_sequence_once():
    runtime = CompanionRuntime("udp:127.0.0.1:1", dry_run=True)
    runtime.store = SimpleNamespace(save=lambda: None)
    runtime.contract = RawReplayContract(
        29.0, 104.0, 300.0, 1, 9, release_sequences=(7,), automatic_takeoff=True
    )
    runtime.mission = [
        MissionItem(7, 0, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, param1=8, param2=1000)
    ]
    runtime.telemetry.armed = True
    runtime.launched = True
    runtime._try_mission_release_fallback(6)
    assert not runtime.release_commanded
    runtime._try_mission_release_fallback(7)
    assert runtime.release_commanded
    assert runtime.result.release["source"] == "MISSION_PLANNER_RAW_COMMAND"
    assert runtime.result.release["mission_command"]["pwm"] == 1000
    first = runtime.result.release
    runtime._try_mission_release_fallback(7)
    assert runtime.result.release is first


def test_stale_release_latch_from_previous_world_is_discarded(tmp_path, monkeypatch):
    runtime_dir = tmp_path / "runtime"
    runtime_dir.mkdir()
    monkeypatch.setattr(companion, "RUNTIME_DIR", runtime_dir)
    (runtime_dir / "payload_released.flag").write_text("old", encoding="utf-8")
    (runtime_dir / "impact.json").write_text("{}", encoding="utf-8")

    runtime = CompanionRuntime("udp:127.0.0.1:1", dry_run=True)
    runtime.store = SimpleNamespace(save=lambda: None)
    runtime.launched = False
    runtime._poll_physical_release()
    runtime._poll_impact()

    assert not runtime.released
    assert runtime.result.impact is None
    assert not (runtime_dir / "payload_released.flag").exists()
    assert not (runtime_dir / "impact.json").exists()


def test_physical_touchdown_and_full_stop_are_recorded_inside_landing_zone(
    tmp_path, monkeypatch
):
    monkeypatch.setattr(companion, "RUNTIME_DIR", tmp_path)
    monkeypatch.setattr(companion, "save_route_state", lambda _state: None)
    runtime = CompanionRuntime("udp:127.0.0.1:1", dry_run=True)
    runtime.store = SimpleNamespace(save=lambda: None)
    runtime.launched = True
    runtime.airborne = True
    runtime.launch_time = time.monotonic() - 10
    runtime.contract = SimpleNamespace(land_seq=14)
    runtime.telemetry.lat_deg = 28.0
    runtime.telemetry.lon_deg = 115.0
    runtime.telemetry.north_m = 12.0
    runtime.telemetry.east_m = -4.0
    (tmp_path / "touchdown.json").write_text(
        json.dumps(
            {
                "sim_time_s": 12.0,
                "position": {"x": 12.0, "y": 4.0, "z": 0.13},
                "groundspeed_mps": 8.0,
            }
        ),
        encoding="utf-8",
    )
    runtime.telemetry.groundspeed_mps = 8.0
    runtime.telemetry.mission_seq = 14
    runtime.telemetry.armed = True
    runtime._poll_touchdown()
    runtime._update_flight_metrics()
    assert runtime.result.first_touchdown["in_takeoff_landing_zone"]

    runtime.telemetry.groundspeed_mps = 0.4
    runtime.telemetry.armed = False
    runtime._update_flight_metrics()
    assert runtime.result.final_stop is None

    runtime.telemetry.groundspeed_mps = 0.1
    runtime._update_flight_metrics()
    runtime.stopped_since = time.monotonic() - companion.FULL_STOP_HOLD_S
    runtime._update_flight_metrics()
    assert runtime.result.final_stop["in_takeoff_landing_zone"]


def test_scan_metrics_report_real_cross_track_speed_and_height(monkeypatch):
    monkeypatch.setattr(companion, "save_route_state", lambda _state: None)
    runtime = CompanionRuntime("udp:127.0.0.1:1", dry_run=True)
    runtime.store = SimpleNamespace(save=lambda: None)
    start_lat, start_lon = ne_to_latlon(145.0, 0.0)
    end_lat, end_lon = ne_to_latlon(235.0, 0.0)
    runtime.mission = [
        MissionItem(3, 3, companion.mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, x=start_lat, y=start_lon),
        MissionItem(4, 3, companion.mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, x=end_lat, y=end_lon),
    ]
    runtime.contract = SimpleNamespace(
        scan_start_seq=2,
        scan_end_seq=5,
        fallback_start_seq=7,
        land_seq=20,
    )
    runtime.launched = True
    runtime.launch_time = time.monotonic() - 10.0
    runtime.telemetry.lat_deg = 28.0
    runtime.telemetry.lon_deg = 115.0
    runtime.telemetry.north_m = 190.0
    runtime.telemetry.east_m = 2.0
    runtime.telemetry.mission_seq = 4
    runtime.telemetry.airspeed_mps = 13.5
    runtime.telemetry.relative_alt_m = 20.0

    runtime._update_flight_metrics()

    metrics = runtime.result.route_metrics
    assert metrics["scan_samples"] == 1
    assert metrics["scan_cross_track_rms_m"] == 2.0
    assert metrics["scan_cross_track_max_m"] == 2.0
    assert metrics["scan_mean_airspeed_mps"] == 13.5
    assert metrics["scan_mean_agl_m"] == 20.0

    runtime.telemetry.mission_seq = 7
    runtime.telemetry.roll_rad = math.radians(20.0)
    runtime.telemetry.groundspeed_mps = 13.5
    runtime.telemetry.yaw_rate_rps = 0.35
    runtime._update_flight_metrics()
    assert metrics["turn_radius_samples"] == 1
    assert metrics["turn_radius_mean_m"] == 13.5 / 0.35


def test_rectangle_scan_metrics_exclude_turn_helpers(monkeypatch):
    monkeypatch.setattr(companion, "save_route_state", lambda _state: None)
    runtime = CompanionRuntime("udp:127.0.0.1:1", dry_run=True)
    runtime.store = SimpleNamespace(save=lambda: None)

    def waypoint(seq: int, north_m: float, east_m: float) -> MissionItem:
        lat, lon = ne_to_latlon(north_m, east_m)
        return MissionItem(
            seq,
            3,
            companion.mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            x=lat,
            y=lon,
            z=20.0,
        )

    runtime.mission = [
        waypoint(3, 145.0, 0.0),
        waypoint(4, 235.0, 0.0),
        waypoint(5, 260.0, 0.0),
        waypoint(6, 270.0, 15.0),
        waypoint(7, 260.0, 30.0),
        waypoint(8, 235.0, 30.0),
        waypoint(9, 145.0, 30.0),
    ]
    runtime.contract = SimpleNamespace(
        scan_start_seq=2,
        scan_end_seq=10,
        fallback_start_seq=12,
        land_seq=20,
    )
    runtime.launched = True
    runtime.launch_time = time.monotonic() - 10.0
    runtime.telemetry.lat_deg = 28.0
    runtime.telemetry.lon_deg = 115.0
    runtime.telemetry.north_m = 190.0
    runtime.telemetry.east_m = 2.0
    runtime.telemetry.mission_seq = 4
    runtime.telemetry.airspeed_mps = 13.5
    runtime.telemetry.relative_alt_m = 20.0
    runtime._update_flight_metrics()
    assert runtime.result.route_metrics["scan_samples"] == 1
    assert runtime.result.route_metrics["scan_cross_track_rms_m"] == pytest.approx(2.0)

    runtime.telemetry.mission_seq = 6
    runtime.telemetry.east_m = 15.0
    runtime._update_flight_metrics()
    assert runtime.result.route_metrics["scan_samples"] == 1

    runtime.telemetry.mission_seq = 9
    runtime.telemetry.east_m = 32.0
    runtime._update_flight_metrics()
    assert runtime.result.route_metrics["scan_samples"] == 2
    assert runtime.result.route_metrics["scan_cross_track_rms_m"] == pytest.approx(2.0)


def test_dynamic_mission_replaces_stale_raw_replay_world(tmp_path, monkeypatch):
    from pathlib import Path

    monkeypatch.setattr(companion, "RUNTIME_DIR", tmp_path)
    monkeypatch.setattr(companion, "save_route_state", lambda _state: None)
    monkeypatch.setattr(companion, "write_runtime", lambda *_args, **_kwargs: None)
    runtime = CompanionRuntime("udp:127.0.0.1:1", dry_run=True)
    runtime.store = SimpleNamespace(save=lambda: None)
    items = []
    path = Path(__file__).resolve().parents[1] / "config/missions/group_a.waypoints"
    for row in path.read_text().splitlines()[1:]:
        f = row.split()
        items.append(MissionItem(
            seq=int(f[0]), current=int(f[1]), frame=int(f[2]), command=int(f[3]),
            param1=float(f[4]), param2=float(f[5]), param3=float(f[6]), param4=float(f[7]),
            x=float(f[8]), y=float(f[9]), z=float(f[10]), autocontinue=int(f[11]),
        ))
    runtime.client.download_mission = lambda timeout: items
    runtime._captured_uploaded_mission = lambda: None
    replay = tmp_path / "mission_replay.json"
    replay.write_text("{}")
    runtime._refresh_mission()
    assert runtime.result.status in {"READY", "READY_FIXED_FALLBACK"}
    assert companion.deserialize_mission(json.loads(replay.read_text())['items']) == items
    assert not (tmp_path / "restart.request").exists()
    assert runtime.origin_lat_deg == companion.ORIGIN_LAT_DEG


@pytest.mark.parametrize('randomize,armed',[(True,False),(False,False),(True,True)])
def test_new_task_seed_changes_only_for_disarmed_random_tasks(tmp_path,monkeypatch,randomize,armed):
    monkeypatch.setattr(companion,'RUNTIME_DIR',tmp_path)
    (tmp_path/'task_policy.json').write_text(json.dumps({'randomize_each_upload':randomize}))
    writes=[];commands=[]
    monkeypatch.setattr(companion,'write_runtime',lambda parameters:writes.append(parameters))
    monkeypatch.setattr(companion.secrets,'randbelow',lambda upper:7)
    runtime=CompanionRuntime('udp:127.0.0.1:1',dry_run=True)
    runtime.client.set_parameters=lambda values:commands.append(values)
    runtime.telemetry.armed=armed
    initial=runtime.parameters.seed
    runtime._prepare_task_seed()
    first=runtime.parameters.seed
    runtime._prepare_task_seed()
    if randomize and not armed:
        assert initial!=first!=runtime.parameters.seed
        assert len(commands)==len(writes)==2
        assert (tmp_path/'restart.request').exists()
        from cuadc_sim.scenario import generate_scenario
        assert generate_scenario(writes[0])!=generate_scenario(writes[1])
    else:
        assert runtime.parameters.seed==initial
        assert not commands and not writes
        assert not (tmp_path/'restart.request').exists()
