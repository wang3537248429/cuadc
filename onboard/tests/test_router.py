from __future__ import annotations

from dataclasses import replace
import os
import pty
import select
import socket
import threading
import time
from pathlib import Path

from cuadc_onboard.config import OnboardConfig, RuntimeConfig
from cuadc_onboard.router import MavlinkRouter
from pymavlink import mavutil
import pytest


def open_pty_or_skip() -> tuple[int, int]:
    """串口路由测试需要一对伪终端；受限容器通常未暴露 /dev/pts。"""
    try:
        return pty.openpty()
    except OSError as error:
        pytest.skip(f"当前环境无法分配伪终端，跳过串口路由测试：{error}")


def reserve_udp_port() -> int:
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    probe.bind(("127.0.0.1", 0))
    port = int(probe.getsockname()[1])
    probe.close()
    return port


def wait_until(predicate, timeout: float = 3.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError("condition did not become true")


def read_pty(master_fd: int, expected: bytes, timeout: float = 2.0) -> bytes:
    received = bytearray()
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and expected not in received:
        readable, _, _ = select.select([master_fd], [], [], 0.1)
        if readable:
            received.extend(os.read(master_fd, 4096))
    return bytes(received)


def recv_udp_stream(
    udp_socket: socket.socket,
    expected: bytes,
    timeout: float = 2.0,
) -> tuple[bytes, tuple[str, int]]:
    received = bytearray()
    address = ("", 0)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and expected not in received:
        udp_socket.settimeout(max(0.01, deadline - time.monotonic()))
        payload, address = udp_socket.recvfrom(4096)
        received.extend(payload)
    return bytes(received), address


def test_router_keeps_serial_single_owner_and_routes_companion_over_usb(tmp_path: Path):
    master_fd, slave_fd = open_pty_or_skip()
    slave_path = os.ttyname(slave_fd)
    companion = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    companion.settimeout(2.0)
    companion_port = reserve_udp_port()
    router_port = reserve_udp_port()
    companion.bind(("127.0.0.1", companion_port))

    base = OnboardConfig()
    config = replace(
        base,
        flight_controller=replace(
            base.flight_controller,
            endpoint=f"udpin:127.0.0.1:{companion_port}",
        ),
        telemetry_router=replace(
            base.telemetry_router,
            serial_device=slave_path,
            serial_reconnect_s=0.05,
            companion_port=companion_port,
            router_local_port=router_port,
            log_tlog=False,
        ),
        runtime=RuntimeConfig(directory=str(tmp_path), status_hz=2.0),
    )
    config.validate()
    router = MavlinkRouter(config)
    thread = threading.Thread(target=router.run, daemon=True)
    thread.start()
    try:
        wait_until(lambda: bool(router.state["serial_connected"]))
        os.write(master_fd, b"FC-TELEMETRY")
        companion_payload, _ = recv_udp_stream(companion, b"FC-TELEMETRY")
        assert b"FC-TELEMETRY" in companion_payload

        companion.sendto(b"LOCAL-COMMAND", ("127.0.0.1", router_port))
        assert b"LOCAL-COMMAND" in read_pty(master_fd, b"LOCAL-COMMAND")

        encoder = mavutil.mavlink.MAVLink(None, srcSystem=1, srcComponent=191)
        report = encoder.named_value_int_encode(1000, b"CU_TASK", 2).pack(encoder)
        companion.sendto(report, ("127.0.0.1", router_port))
        assert report in read_pty(master_fd, report)
    finally:
        router.shutdown()
        thread.join(timeout=3.0)
        companion.close()
        os.close(master_fd)
        os.close(slave_fd)
    assert not thread.is_alive()
    assert router.state["bytes_fc_to_local"] >= len(b"FC-TELEMETRY")
    assert router.state["bytes_local_to_fc"] >= len(b"LOCAL-COMMAND") + len(report)


def test_router_blocks_external_prearm_check_commands_from_companion(tmp_path: Path):
    master_fd, slave_fd = open_pty_or_skip()
    slave_path = os.ttyname(slave_fd)
    companion = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    companion_port = reserve_udp_port()
    router_port = reserve_udp_port()
    companion.bind(("127.0.0.1", companion_port))

    base = OnboardConfig()
    config = replace(
        base,
        flight_controller=replace(
            base.flight_controller,
            endpoint=f"udpin:127.0.0.1:{companion_port}",
        ),
        telemetry_router=replace(
            base.telemetry_router,
            serial_device=slave_path,
            serial_reconnect_s=0.05,
            companion_port=companion_port,
            router_local_port=router_port,
            log_tlog=False,
        ),
        runtime=RuntimeConfig(directory=str(tmp_path), status_hz=2.0),
    )
    config.validate()
    router = MavlinkRouter(config)
    thread = threading.Thread(target=router.run, daemon=True)
    thread.start()
    try:
        wait_until(lambda: bool(router.state["serial_connected"]))
        encoder = mavutil.mavlink.MAVLink(
            None,
            srcSystem=255,
            srcComponent=190,
        )
        blocked_long = encoder.command_long_encode(
            1,
            1,
            mavutil.mavlink.MAV_CMD_RUN_PREARM_CHECKS,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        ).pack(encoder)
        blocked_int = encoder.command_int_encode(
            1,
            1,
            mavutil.mavlink.MAV_FRAME_GLOBAL,
            mavutil.mavlink.MAV_CMD_RUN_PREARM_CHECKS,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        ).pack(encoder)
        companion.sendto(blocked_long, ("127.0.0.1", router_port))
        companion.sendto(blocked_int, ("127.0.0.1", router_port))
        wait_until(
            lambda: router.state["blocked_local_prearm_check_commands"] == 2
        )
        assert read_pty(master_fd, blocked_long, timeout=0.25) == b""

        allowed = encoder.command_long_encode(
            1,
            1,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        ).pack(encoder)
        companion.sendto(allowed, ("127.0.0.1", router_port))
        assert allowed in read_pty(master_fd, allowed)
    finally:
        router.shutdown()
        thread.join(timeout=3.0)
        companion.close()
        os.close(master_fd)
        os.close(slave_fd)
    assert not thread.is_alive()
    assert router.state["blocked_local_prearm_check_commands"] == 2
    assert router.state["last_blocked_local_prearm_check_at"] is not None
    assert router.state["bytes_local_to_fc"] == len(allowed)
