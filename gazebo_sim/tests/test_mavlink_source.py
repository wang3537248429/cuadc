from types import SimpleNamespace

from cuadc_sim.mavlink_io import wait_autopilot_heartbeat


def test_routed_companion_and_gcs_cannot_select_command_destination():
    def heartbeat(system,component,autopilot):
        return SimpleNamespace(get_srcSystem=lambda:system,get_srcComponent=lambda:component,autopilot=autopilot)
    messages=iter([heartbeat(255,0,8),heartbeat(1,191,8),heartbeat(1,1,3)])
    connection=SimpleNamespace(recv_match=lambda **kwargs:next(messages),target_system=0,target_component=0)
    selected=wait_autopilot_heartbeat(connection,timeout=1)
    assert selected.get_srcComponent()==1
    assert (connection.target_system,connection.target_component)==(1,1)
