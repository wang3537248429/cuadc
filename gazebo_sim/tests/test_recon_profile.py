from pathlib import Path
import math

import pytest
from pymavlink import mavutil, mavwp

from cuadc_sim.mission import (MissionItem, NAV_WAYPOINT, validate_mission, latlon_to_ne,
                               build_descent_side_patch)
from cuadc_sim.recon_profile import ReconProfile, apply_recon_profile


def load_template(group='a'):
    loader=mavwp.MAVWPLoader()
    loader.load(str(Path(__file__).resolve().parents[1]/f'config/missions/group_{group}.waypoints'))
    return [MissionItem.from_message(loader.wp(index)) for index in range(loader.count())]


def test_low_scan_preserves_release_contract_and_slows_before_descent():
    original=load_template()
    updated=apply_recon_profile(original,ReconProfile())
    contract,errors=validate_mission(updated)
    assert not errors
    assert len(updated)>len(original)
    assert updated[2].command==mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED
    assert updated[2].param1==0
    assert updated[2].param2==ReconProfile().cruise_airspeed_mps
    assert updated[3].command==mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS
    assert updated[3].z==14
    assert any(item.param2==ReconProfile().airspeed_mps and item.command==mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED for item in updated)
    assert original[3].z==20
    scan=[item for item in updated if contract.scan_start_seq<item.seq<contract.scan_end_seq and item.command==NAV_WAYPOINT]
    assert all(item.z==14 for item in scan)
    assert updated[contract.fallback_exit_seq+1].command==mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED
    assert updated[contract.fallback_exit_seq+1].param2==ReconProfile().cruise_airspeed_mps
    assert updated[-1].x==original[0].x
    north,east=latlon_to_ne(updated[-1].x,updated[-1].y,original[0].x,original[0].y)
    assert abs(north)<.01 and east==pytest.approx(20,abs=.01)


def test_low_speed_turns_respect_bank_and_field_limits():
    profile=ReconProfile()
    mission=apply_recon_profile(load_template(),profile)
    origin=mission[0]
    for item in mission:
        if item.command==mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS:
            radius=abs(item.param3)
            bank=math.degrees(math.atan(profile.cruise_airspeed_mps**2/(9.80665*radius)))
            north,east=latlon_to_ne(item.x,item.y,origin.x,origin.y)
            assert bank < profile.bank_limit_deg
            assert math.hypot(north,east)+radius < 300


@pytest.mark.parametrize('side',[-1,1])
@pytest.mark.parametrize('group',['a','b'])
def test_east_west_landing_keeps_its_aim_point_when_mirroring(side,group):
    mission=apply_recon_profile(load_template(group),ReconProfile())
    contract,errors=validate_mission(mission)
    assert not errors
    landing=build_descent_side_patch(mission,contract,side)[-1]
    north,east=latlon_to_ne(landing.x,landing.y)
    assert abs(north)<.01
    assert east==pytest.approx(side*20,abs=.01)
    original_arcs=[item for item in mission[contract.fallback_exit_seq+1:]
                   if item.command==mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS]
    mirrored_arcs=[item for item in build_descent_side_patch(mission,contract,side)
                   if item.command==mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS]
    assert [item.param3 for item in mirrored_arcs]==[item.param3*side for item in original_arcs]


@pytest.mark.parametrize('profile',[ReconProfile(airspeed_mps=8),ReconProfile(altitude_m=1),ReconProfile(altitude_m=float('nan'))])
def test_unflyable_profiles_are_rejected(profile):
    with pytest.raises(ValueError):profile.validate()
