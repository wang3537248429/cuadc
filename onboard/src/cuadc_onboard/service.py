from __future__ import annotations

import os
import socket
import time
from pathlib import Path


def systemd_notify(message: str) -> bool:
    """Send a notification without requiring the optional python-systemd wheel."""

    address = os.environ.get("NOTIFY_SOCKET")
    if not address:
        return False
    if address.startswith("@"):
        address = "\0" + address[1:]
    client = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    try:
        client.connect(address)
        client.sendall(message.encode("utf-8"))
        return True
    except OSError:
        return False
    finally:
        client.close()


def rotate_file(path: Path, max_bytes: int, backups: int) -> None:
    """Rotate one bounded runtime log without touching unrelated files."""

    try:
        if path.stat().st_size < max_bytes:
            return
    except FileNotFoundError:
        return
    oldest = path.with_name(f"{path.name}.{backups}")
    oldest.unlink(missing_ok=True)
    for index in range(backups - 1, 0, -1):
        source = path.with_name(f"{path.name}.{index}")
        if source.exists():
            source.replace(path.with_name(f"{path.name}.{index + 1}"))
    path.replace(path.with_name(f"{path.name}.1"))


class SystemdWatchdog:
    def __init__(self) -> None:
        configured = os.environ.get("WATCHDOG_USEC", "")
        try:
            timeout_s = int(configured) / 1_000_000.0
        except ValueError:
            timeout_s = 0.0
        expected_pid = os.environ.get("WATCHDOG_PID")
        if expected_pid and expected_pid != str(os.getpid()):
            timeout_s = 0.0
        self.interval_s = timeout_s / 2.0 if timeout_s > 0.0 else 0.0
        self.last_ping = -float("inf")

    def ready(self, status: str) -> None:
        systemd_notify(f"READY=1\nSTATUS={status}")
        self.ping(force=True)

    def ping(self, force: bool = False) -> None:
        now = time.monotonic()
        if self.interval_s <= 0.0:
            return
        if force or now - self.last_ping >= self.interval_s:
            systemd_notify("WATCHDOG=1")
            self.last_ping = now

    def status(self, text: str) -> None:
        systemd_notify(f"STATUS={text}")

    def stopping(self) -> None:
        systemd_notify("STOPPING=1")
