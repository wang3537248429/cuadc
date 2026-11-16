from dataclasses import replace
from types import SimpleNamespace

import pytest

from cuadc_onboard import mavlink_io
from cuadc_onboard.config import OnboardConfig, RuntimeConfig
from cuadc_onboard.mavlink_io import MavlinkClient, MavlinkTimeout
from cuadc_onboard.runtime import OnboardRuntime


def rc8_runtime(tmp_path):
    base = OnboardConfig()
    return OnboardRuntime(replace(
        base,
        release=replace(base.release, required_servo_function=58),
        runtime=RuntimeConfig(directory=str(tmp_path)),
    ))


@pytest.mark.parametrize("channel", [1, 7, 9, 16])
def test_rc8_profile_cannot_silently_control_a_different_output(channel):
    base = OnboardConfig()
    config = replace(base, release=replace(
        base.release, required_servo_function=58, servo_channel=channel,
    ))
    with pytest.raises(ValueError, match="servo_channel=8"):
        config.validate()


@pytest.mark.parametrize("function,minimum,maximum,valid", [
    (58, 1100, 1900, True),
    (58, 1100, 1800, True),
    (0, 1100, 1900, False),
    (57, 1100, 1900, False),
    (58.1, 1100, 1900, False),
    (58, 1100, 1700, False),
    (58, 1850, 1900, False),
    (58, float("nan"), 1900, False),
    (58, 1100, float("inf"), False),
])
def test_rc8_still_requires_exact_hardware_function_and_pwm_range(
    tmp_path, function, minimum, maximum, valid,
):
    runtime = rc8_runtime(tmp_path)
    values = {"SERVO8_FUNCTION": function, "SERVO8_MIN": minimum, "SERVO8_MAX": maximum}
    runtime.client = SimpleNamespace(read_named_parameters=lambda *_a, **_k: values)
    runtime._read_servo_configuration()
    assert runtime.servo_config_valid is valid
    assert runtime.servo_configuration["valid"] is valid
    if valid:
        assert runtime.servo_configuration["control_mode"] == "RC8_AND_MAVLINK"


def test_independent_profile_does_not_silently_accept_rc_passthrough(tmp_path):
    runtime = OnboardRuntime(replace(
        OnboardConfig(), runtime=RuntimeConfig(directory=str(tmp_path)),
    ))
    values = {"SERVO8_FUNCTION": 58.0, "SERVO8_MIN": 1100.0, "SERVO8_MAX": 1900.0}
    runtime.client = SimpleNamespace(read_named_parameters=lambda *_a, **_k: values)
    runtime._read_servo_configuration()
    assert not runtime.servo_config_valid


def test_timeout_blocks_then_ground_retry_recovers_without_parameter_writes(tmp_path):
    runtime = rc8_runtime(tmp_path)
    calls = []

    def read(names, timeout):
        calls.append((set(names), timeout))
        if len(calls) == 1:
            raise MavlinkTimeout("delayed reply")
        return {"SERVO8_FUNCTION": 58.0, "SERVO8_MIN": 1100.0, "SERVO8_MAX": 1900.0}

    runtime.client = SimpleNamespace(read_named_parameters=read)
    runtime._read_servo_configuration()
    assert not runtime.servo_config_valid
    assert "delayed reply" in runtime.servo_configuration["error"]
    runtime.last_servo_check -= 11.0
    runtime._retry_servo_configuration()
    assert runtime.servo_config_valid
    runtime._retry_servo_configuration()
    assert len(calls) == 2
    assert calls[-1][1] == 6.0


@pytest.mark.parametrize("armed,auto", [(True, False), (False, True), (True, True)])
def test_configuration_retry_never_blocks_armed_or_auto_loop(tmp_path, armed, auto):
    runtime = rc8_runtime(tmp_path)
    runtime.telemetry.armed = armed
    runtime.telemetry.auto = auto
    runtime.client = SimpleNamespace()  # Any attempt to read would fail this test.
    runtime._retry_servo_configuration()
    assert not runtime.servo_config_valid


def test_ground_manual_loading_does_not_latch_release(tmp_path):
    runtime = rc8_runtime(tmp_path)
    for pwm in [1500, 1800, 1900, 1500]:
        runtime.telemetry.servo_pwm[7] = pwm
        runtime._observe_servo_release()
        assert not runtime.release_observed
        assert not runtime.release_commanded
    runtime.telemetry.armed = True
    runtime._observe_servo_release()
    assert not runtime.release_observed


def test_actual_output_while_armed_still_prevents_a_second_release(tmp_path):
    runtime = rc8_runtime(tmp_path)
    runtime.telemetry.armed = True
    runtime.telemetry.servo_pwm[7] = 1800
    runtime._observe_servo_release()
    assert runtime.release_observed
    assert runtime.release_commanded
    assert runtime.state["release"]["mechanical_confirmation"] is False


class Parameter:
    def __init__(self, name, value, system=1, component=1):
        self.param_id = name
        self.param_value = value
        self.system = system
        self.component = component

    def get_srcSystem(self):
        return self.system

    def get_srcComponent(self):
        return self.component


def parameter_client(monkeypatch, messages):
    clock = [0.0]
    monkeypatch.setattr(mavlink_io.time, "monotonic", lambda: clock[0])
    client = MavlinkClient()
    requests = []
    client.connection = SimpleNamespace(mav=SimpleNamespace(
        param_request_read_send=lambda *args: requests.append(("read", args)),
        param_request_list_send=lambda *args: requests.append(("list", args)),
    ))
    queue = iter(messages)

    def recv(_types, timeout):
        clock[0] += timeout
        return next(queue, None)

    client.recv = recv
    return client, requests, clock


def test_parameter_batch_accepts_reordering_and_rejects_other_sources(monkeypatch):
    client, requests, _clock = parameter_client(monkeypatch, [
        Parameter("SERVO8_FUNCTION", 0, system=2),
        Parameter("SERVO8_FUNCTION", 0, component=190),
        Parameter("UNRELATED", 42),
        Parameter("SERVO8_MAX", 1900),
        Parameter(b"SERVO8_FUNCTION\x00", 58),
        Parameter("SERVO8_MIN", 1100),
    ])
    values = client.read_named_parameters(
        ["SERVO8_FUNCTION", "SERVO8_MIN", "SERVO8_MAX"], timeout=4.0,
    )
    assert values == {"SERVO8_FUNCTION": 58, "SERVO8_MIN": 1100, "SERVO8_MAX": 1900}
    assert all(kind == "read" for kind, _args in requests)


def test_delayed_parameter_uses_one_read_only_list_fallback(monkeypatch):
    client, requests, _clock = parameter_client(
        monkeypatch, [None] * 5 + [Parameter("SERVO8_FUNCTION", 58)],
    )
    assert client.read_named_parameters(["SERVO8_FUNCTION"], timeout=2.0) == {
        "SERVO8_FUNCTION": 58,
    }
    assert sum(kind == "list" for kind, _args in requests) == 1


def test_incomplete_parameter_read_has_one_deadline_and_no_write_api(monkeypatch):
    client, requests, clock = parameter_client(monkeypatch, [])
    with pytest.raises(MavlinkTimeout, match="SERVO8_FUNCTION"):
        client.read_named_parameters(["SERVO8_FUNCTION"], timeout=2.0)
    assert clock[0] <= 2.0
    assert sum(kind == "list" for kind, _args in requests) == 1


@pytest.mark.parametrize("value", [float("nan"), float("inf"), -float("inf")])
def test_nonfinite_parameter_is_never_used_to_verify_an_output(monkeypatch, value):
    client, _requests, _clock = parameter_client(monkeypatch, [Parameter("SERVO8_MIN", value)])
    with pytest.raises(ValueError, match="not finite"):
        client.read_named_parameters(["SERVO8_MIN"])
