from __future__ import annotations

import os
import queue
import sys
import threading
import time
from pathlib import Path
from typing import Mapping

from pymavlink import mavutil

from .model import COMPANION_COMPONENT_ID, ReportAggregator, ReportSnapshot


def _windows_documents_dir() -> Path | None:
    if sys.platform != "win32":
        return None
    try:
        import ctypes

        buffer = ctypes.create_unicode_buffer(32768)
        # CSIDL_PERSONAL follows Windows Known Folder redirection, including
        # OneDrive and managed enterprise Documents folders.
        result = ctypes.windll.shell32.SHGetFolderPathW(None, 5, None, 0, buffer)
        return Path(buffer.value) if result == 0 and buffer.value else None
    except (AttributeError, OSError, ValueError):
        return None


def mission_planner_log_candidates(
    *,
    home: Path | None = None,
    environ: Mapping[str, str] | None = None,
    platform_name: str | None = None,
    executable_dir: Path | None = None,
) -> tuple[Path, ...]:
    """Return likely Mission Planner log directories in priority order.

    Mission Planner itself remains the only process that opens the telemetry
    link.  This monitor merely discovers and follows the log it is already
    writing, on Linux/Mono as well as native Windows.
    """

    env = os.environ if environ is None else environ
    platform_name = sys.platform if platform_name is None else platform_name
    home = Path.home() if home is None else Path(home)
    candidates: list[Path] = []

    def add(path: Path | None) -> None:
        if path is not None and path not in candidates:
            candidates.append(path)

    if platform_name == "win32":
        user_profile = Path(env.get("USERPROFILE", str(home)))
        if platform_name == sys.platform:
            documents = _windows_documents_dir()
            if documents is not None:
                add(documents / "Mission Planner" / "logs")
        one_drive = env.get("OneDrive") or env.get("OneDriveConsumer")
        if one_drive:
            add(Path(one_drive) / "Documents" / "Mission Planner" / "logs")
        add(user_profile / "Documents" / "Mission Planner" / "logs")
        local_app_data = env.get("LOCALAPPDATA")
        roaming_app_data = env.get("APPDATA")
        if local_app_data:
            add(Path(local_app_data) / "Mission Planner" / "logs")
        if roaming_app_data:
            add(Path(roaming_app_data) / "Mission Planner" / "logs")
    else:
        xdg_data_home = env.get("XDG_DATA_HOME")
        add(
            Path(xdg_data_home) / "Mission Planner" / "logs"
            if xdg_data_home
            else home / ".local" / "share" / "Mission Planner" / "logs"
        )
        add(home / "Mission Planner" / "logs")

    add(home / "Documents" / "Mission Planner" / "logs")
    if executable_dir is not None:
        add(Path(executable_dir) / "logs")
        add(Path(executable_dir) / "Mission Planner" / "logs")
    return tuple(candidates)


def newest_tlog(root: Path) -> Path | None:
    try:
        candidates = [path for path in root.rglob("*.tlog") if path.is_file()]
    except OSError:
        return None
    newest: tuple[float, Path] | None = None
    for path in candidates:
        try:
            modified_at = path.stat().st_mtime
        except OSError:
            continue
        if newest is None or modified_at > newest[0]:
            newest = (modified_at, path)
    return newest[1] if newest is not None else None


def default_tlog_root(candidates: tuple[Path, ...] | None = None) -> Path:
    candidates = candidates or mission_planner_log_candidates()
    roots_with_logs = [
        (newest_tlog(root), root)
        for root in candidates
        if root.is_dir()
    ]
    roots_with_logs = [(log, root) for log, root in roots_with_logs if log is not None]
    if roots_with_logs:
        return max(roots_with_logs, key=lambda item: item[0].stat().st_mtime)[1]
    for root in candidates:
        if root.is_dir():
            return root
    return candidates[0]


DEFAULT_TLOG_ROOT = default_tlog_root()


def scan_tlog(path: Path, component_id: int = COMPANION_COMPONENT_ID) -> ReportSnapshot:
    aggregator = ReportAggregator(component_id)
    aggregator.set_source(path)
    connection = mavutil.mavlink_connection(str(path), robust_parsing=True)
    try:
        while True:
            message = connection.recv_match(blocking=False)
            if message is None:
                break
            if message.get_type() != "BAD_DATA":
                aggregator.update(message)
    finally:
        connection.close()
    return aggregator.snapshot.clone()


class TlogFollower(threading.Thread):
    def __init__(
        self,
        output: "queue.Queue[tuple[str, object]]",
        root: Path = DEFAULT_TLOG_ROOT,
        component_id: int = COMPANION_COMPONENT_ID,
    ) -> None:
        super().__init__(name="cuadc-tlog-follower", daemon=True)
        self.output = output
        self.root = root
        self.component_id = component_id
        self.stop_event = threading.Event()

    def stop(self) -> None:
        self.stop_event.set()

    def _publish(self, kind: str, payload: object) -> None:
        try:
            self.output.put_nowait((kind, payload))
        except queue.Full:
            try:
                self.output.get_nowait()
            except queue.Empty:
                pass
            self.output.put_nowait((kind, payload))

    def run(self) -> None:
        current_path: Path | None = None
        connection = None
        aggregator = ReportAggregator(self.component_id)
        last_publish = 0.0
        last_search = 0.0
        observed_size = -1
        try:
            while not self.stop_event.is_set():
                try:
                    now = time.monotonic()
                    if current_path is None or now - last_search >= 2.0:
                        last_search = now
                        candidate = newest_tlog(self.root)
                        truncated = False
                        if candidate is not None and candidate == current_path:
                            try:
                                size = candidate.stat().st_size
                                truncated = observed_size >= 0 and size < observed_size
                                observed_size = size
                            except OSError:
                                candidate = None
                        if candidate is not None and (candidate != current_path or truncated):
                            if connection is not None:
                                connection.close()
                            current_path = candidate
                            observed_size = candidate.stat().st_size
                            aggregator = ReportAggregator(self.component_id)
                            aggregator.set_source(candidate)
                            connection = mavutil.mavlink_connection(
                                str(candidate), robust_parsing=True
                            )
                            self._publish("status", f"正在读取：{candidate}")
                        elif candidate is None:
                            if connection is not None:
                                connection.close()
                                connection = None
                            current_path = None
                            observed_size = -1
                            self._publish("status", "尚未找到 Mission Planner .tlog 日志")
                    changed = False
                    if connection is not None:
                        for _ in range(3000):
                            message = connection.recv_match(blocking=False)
                            if message is None:
                                break
                            if message.get_type() != "BAD_DATA" and aggregator.update(message):
                                changed = True
                    if changed or now - last_publish >= 1.0:
                        last_publish = now
                        self._publish("snapshot", aggregator.snapshot.clone())
                    self.stop_event.wait(0.12)
                except Exception as exc:
                    if connection is not None:
                        try:
                            connection.close()
                        except Exception:
                            pass
                    connection = None
                    current_path = None
                    observed_size = -1
                    self._publish(
                        "error",
                        f"Mission Planner 日志暂时不可读，正在重试：{type(exc).__name__}: {exc}",
                    )
                    self.stop_event.wait(1.0)
        finally:
            if connection is not None:
                connection.close()
