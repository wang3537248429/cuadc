from __future__ import annotations

import math
import json
from pathlib import Path
import pytest

from pymavlink import mavutil

from cuadc_sim.config import RAW_REPLAY_RELEASE_PWM, SimParameters
from cuadc_sim.field_layout import derive_field_layout
from cuadc_sim.mission import (
    MISSION_MODE_RAW_REPLAY,
    MissionItem,
    RawReplayContract,
    assess_raw_replay,
)
from cuadc_sim.replay_route import (
    deserialize_mission,
    load_replay_route,
    mission_route_compatible,
    write_replay_route,
)
from cuadc_sim.scenario import write_runtime


ROUTES = (
    Path(__file__).parent / "fixtures/sample_route_1.waypoints",
    Path(__file__).parent / "fixtures/sample_route_2.waypoints",
)


def load_wpl(path: Path) -> list[MissionItem]:
    items = []
    for line in path.read_text(encoding="utf-8").splitlines()[1:]:
        if not line.strip():
            continue
        fields = line.split("\t")
        items.append(
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
    return items


def test_supplied_routes_round_trip_without_rewriting(tmp_path):
    for path in ROUTES:
        original = load_wpl(path)
        assessment = assess_raw_replay(original, payload_pwm=RAW_REPLAY_RELEASE_PWM)
        assert assessment.mode == MISSION_MODE_RAW_REPLAY
        assert assessment.contract is not None
        route_path = tmp_path / f"{path.stem}.json"
        write_replay_route(original, derive_field_layout(original), route_path)
        payload = load_replay_route(route_path)
        assert payload is not None
        assert deserialize_mission(payload["items"]) == original
        assert payload["layout"]["origin_lat_deg"] == original[0].x
        assert payload["layout"]["origin_lon_deg"] == original[0].y


def test_tagged_low_active_upload_can_use_dynamic_water_without_rewriting_other_slots(tmp_path, monkeypatch):
    from types import SimpleNamespace
    from dataclasses import replace
    from cuadc_sim import companion
    from cuadc_sim.companion import CompanionRuntime
    from cuadc_sim.mission import MissionContract
    items = load_wpl(ROUTES[0])
    # Use the documented low-active PWM and a legal small radius in the
    # replaceable water segment; the dynamic planner supplies its own radius.
    items = [replace(item,param3=15) if item.command==18 else item for item in items]
    monkeypatch.setattr(companion,'RUNTIME_DIR',tmp_path)
    fixed_layout=derive_field_layout(items)
    monkeypatch.setattr(companion,'load_field_layout',lambda:fixed_layout)
    monkeypatch.setattr(companion,'write_runtime',lambda *_args,**_kwargs:None)
    runtime=CompanionRuntime('udp:127.0.0.1:1',dry_run=True)
    runtime.store=SimpleNamespace(save=lambda:None)
    runtime.client.download_mission=lambda **_kwargs:list(items)
    runtime._refresh_mission()
    assert isinstance(runtime.contract,MissionContract)
    assert runtime.result.status=='READY'
    assert runtime.release_pwm==1000
    assert runtime.origin_lat_deg==items[0].x
    assert runtime.origin_lon_deg==items[0].y
    assert runtime.mission==items
    assert (tmp_path/'mission_replay.json').is_file()


def test_public_route_fixtures_keep_geometry_at_a_synthetic_origin():
    for path in ROUTES:
        items = load_wpl(path)
        home = items[0]
        assert abs(home.x - 0.12345) < 1e-7
        assert abs(home.y - 0.12345) < 1e-7
        for item in items:
            if item.x != 0.0 or item.y != 0.0:
                assert abs(item.x - home.x) < 0.02
                assert abs(item.y - home.y) < 0.02


def test_editing_scan_changes_planned_trajectory_but_keeps_the_arena(tmp_path,monkeypatch):
    import json
    from types import SimpleNamespace
    from dataclasses import replace
    from cuadc_sim import companion
    from cuadc_sim.companion import CompanionRuntime
    from cuadc_sim.mission import ne_to_latlon,latlon_to_ne
    from cuadc_sim.field_layout import load_field_layout
    source=Path(__file__).resolve().parents[1]/'config/missions/group_a.waypoints'
    items=load_wpl(source)
    monkeypatch.setattr(companion,'RUNTIME_DIR',tmp_path)
    write_runtime(SimParameters(),runtime_dir=tmp_path)
    monkeypatch.setattr(companion,'load_field_layout',lambda:load_field_layout(tmp_path/'field_layout.json'))
    runtime=CompanionRuntime('udp:127.0.0.1:1',dry_run=True)
    runtime.store=SimpleNamespace(save=lambda:None)
    runtime.client.download_mission=lambda **kwargs:list(items)
    runtime._refresh_mission()
    old_scene=json.loads((tmp_path/'scenario.json').read_text())
    old_path=list(runtime.route_state.mission)
    start,end=runtime.contract.scan_start_seq,runtime.contract.scan_end_seq
    updated=[]
    for item in items:
        if start<item.seq<end and item.command==16:
            n,e=latlon_to_ne(item.x,item.y,items[0].x,items[0].y)
            lat,lon=ne_to_latlon(n,e+12,items[0].x,items[0].y)
            item=replace(item,x=lat,y=lon)
        updated.append(item)
    items=updated
    runtime._refresh_mission()
    new_scene=json.loads((tmp_path/'scenario.json').read_text())
    old_group=next(g for g in old_scene['groups'] if g['name']=='A')
    new_group=next(g for g in new_scene['groups'] if g['name']=='A')
    assert new_scene==old_scene
    assert new_group['center_east_m']==old_group['center_east_m']
    assert runtime.route_state.mission!=old_path
    assert not (tmp_path/'restart.request').exists()
    assert deserialize_mission(load_replay_route(tmp_path/'mission_replay.json')['items'])==items


def test_route_two_rule_deviations_are_warnings_only():
    assessment = assess_raw_replay(load_wpl(ROUTES[1]), payload_pwm=RAW_REPLAY_RELEASE_PWM)
    assert assessment.launch_allowed
    assert isinstance(assessment.contract, RawReplayContract)
    assert not assessment.errors
    assert any("300 m" in warning for warning in assessment.warnings)
    assert any("NAV_TAKEOFF" in warning for warning in assessment.warnings)


def test_empty_or_displaced_planned_home_aligns_to_first_executable_waypoint():
    items = load_wpl(ROUTES[0])
    empty_home = MissionItem(**{**items[0].__dict__, "x": 0.0, "y": 0.0})
    inferred = assess_raw_replay([empty_home, *items[1:]], payload_pwm=RAW_REPLAY_RELEASE_PWM)
    assert inferred.launch_allowed
    assert inferred.contract is not None
    assert inferred.contract.home_lat_deg == items[1].x
    assert inferred.contract.home_lon_deg == items[1].y
    assert any("aligned to the first executable waypoint" in warning for warning in inferred.warnings)

    displaced_home = MissionItem(**{**items[0].__dict__, "x": -35.363261, "y": 149.165230})
    displaced = assess_raw_replay([displaced_home, *items[1:]], payload_pwm=RAW_REPLAY_RELEASE_PWM)
    assert displaced.contract is not None
    assert displaced.contract.home_lat_deg == items[1].x


def test_uploaded_route_cannot_move_the_fixed_field(tmp_path):
    items = load_wpl(ROUTES[1])
    layout = derive_field_layout(items)
    replay_path = tmp_path / "mission_replay.json"
    write_replay_route(items, layout, replay_path)

    _, world_path = write_runtime(
        parameters=SimParameters(),
        runtime_dir=tmp_path,
        replay_data=load_replay_route(replay_path),
    )
    world = world_path.read_text(encoding="utf-8")
    from cuadc_sim.arena import load_arena
    fixed=load_arena().layout
    assert f"<latitude_deg>{fixed.origin_lat_deg:.10f}</latitude_deg>" in world
    assert f"<longitude_deg>{fixed.origin_lon_deg:.10f}</longitude_deg>" in world
    import xml.etree.ElementTree as ET
    pose=list(map(float,ET.fromstring(world).findtext('.//model[@name="cuadc_plane"]/pose','').split()))
    if not pose:
        pose=list(map(float,ET.fromstring(world).findtext('.//include[uri="model://cuadc_twin"]/pose').split()))
    assert pose[-1]==pytest.approx(-math.radians(fixed.initial_heading_deg))
    assert "concrete_synthetic.jpg" in world
    assert "runway_centerline" in world
    assert "slab_joint_" in world
    assert json.loads((tmp_path/'field_layout.json').read_text())['field_size_m']==fixed.field_size_m


def test_raw_release_is_bound_to_matching_sequence_and_pwm():
    items = load_wpl(ROUTES[0])
    assessment = assess_raw_replay(items, payload_pwm=RAW_REPLAY_RELEASE_PWM)
    assert assessment.contract is not None
    assert assessment.contract.release_sequences == (16,)
    assert all(
        item.command == mavutil.mavlink.MAV_CMD_DO_SET_SERVO
        and int(round(item.param1)) == 8
        and math.isclose(item.param2, 1000.0)
        for item in items
        if item.seq in assessment.contract.release_sequences
    )


def test_downloaded_waypoint_radius_quantization_does_not_replace_upload():
    uploaded = load_wpl(ROUTES[0])
    downloaded = [
        MissionItem(**{**item.__dict__, "param2": round(item.param2)})
        if item.command == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
        else item
        for item in uploaded
    ]

    assert mission_route_compatible(uploaded, downloaded)
    changed = list(downloaded)
    changed[7] = MissionItem(**{**changed[7].__dict__, "param2": 11.0})
    assert not mission_route_compatible(uploaded, changed)
