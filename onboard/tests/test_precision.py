from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from pymavlink import mavutil

from cuadc_onboard.config import load_config, RuntimeConfig
from cuadc_onboard.mission import load_qgc_waypoints, assess_mission, is_shadow_neutralization
from cuadc_onboard.precision import PrecisionBudget, estimate_impact_error
from cuadc_onboard.runtime import OnboardRuntime
from cuadc_onboard.mission import retry_entry_seq


ROOT=Path(__file__).resolve().parents[1]


def test_ordinary_gnss_error_cannot_be_claimed_as_sub_metre_accuracy():
    report=estimate_impact_error(20,15,.12,.095,PrecisionBudget())
    assert report['r95_m']>7
    # Even a perfect position fix does not remove bottle timing uncertainty.
    low_gps=estimate_impact_error(20,15,.12,.095,PrecisionBudget(gps_relative_sigma_m=0))
    assert 1<low_gps['r95_m']<report['r95_m']


def test_precision_mode_removes_autonomous_fallback_and_survives_readback(tmp_path):
    base=load_config(ROOT/'config/onboard.yaml')
    config=replace(base,release=replace(base.release,enabled=True,precision_required=True),
                   mission=replace(base.mission,stable_readbacks=1),runtime=RuntimeConfig(directory=str(tmp_path)))
    live=load_qgc_waypoints(ROOT/'tests/fixtures/group_a.waypoints')
    live[9]=replace(live[9],param1=8,param2=1800)
    contract=assess_mission(live,payload_servo=8,min_release_pwm=1800).contract
    original=live[contract.fallback_release_seq]
    writes=[]
    class Client:
        def download_mission(self,timeout=10):return list(live)
        def write_partial(self,items):
            writes.extend(items)
            for item in items:live[item.seq]=item
        def status_text(self,*args):pass
    runtime=OnboardRuntime(config);runtime.client=Client();runtime.servo_config_valid=True
    runtime._refresh_dynamic_mission(force_stable=True)
    assert is_shadow_neutralization(live[contract.fallback_release_seq],original)
    assert runtime.state['autonomous_mission_release'] is False
    count=len(writes)
    runtime._refresh_dynamic_mission(force_stable=True)
    assert len(writes)==count
    assert is_shadow_neutralization(live[contract.fallback_release_seq],original)


def test_incomplete_scan_does_not_insert_an_unplanned_holding_turn(tmp_path):
    base=load_config(ROOT/'config/onboard.yaml')
    runtime=OnboardRuntime(replace(base,runtime=RuntimeConfig(directory=str(tmp_path))))
    items=load_qgc_waypoints(ROOT/'tests/fixtures/group_a.waypoints')
    start=next(index for index,item in enumerate(items) if item.command==600 and round(item.param1)==9101)
    hold=replace(items[start-1],command=mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS,param1=1,param3=-70)
    items.insert(start,hold)
    runtime.mission=[replace(item,seq=index) for index,item in enumerate(items)]
    runtime.contract=assess_mission(runtime.mission).contract
    assert runtime.contract is not None
    entry=retry_entry_seq(runtime.mission,runtime.contract)
    calls=[]
    def set_current(sequence):
        assert not runtime.telemetry_lock.locked()
        calls.append(sequence)
    runtime.client=SimpleNamespace(set_current=set_current)
    runtime._message_is_from_target_autopilot=lambda message:True
    runtime._detections=lambda:[]
    runtime._status_text=lambda *args:None
    runtime._handle_scan_end()
    assert calls==[]
    turn=runtime.contract.fallback_start_seq
    for sequence in (turn,turn+1,turn+1):
        runtime._handle_message(SimpleNamespace(get_type=lambda:'MISSION_CURRENT',seq=sequence))
    assert calls==[]
    assert not runtime.retry_turn_active
    assert runtime.state['status']=='FALLBACK_ROUTE_ACTIVE'
    runtime._handle_message(SimpleNamespace(get_type=lambda:'MISSION_CURRENT',seq=entry))
    assert not runtime.retry_turn_active
    assert runtime.scan_attempt==1
