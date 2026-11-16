from __future__ import annotations

import math
import time
from typing import Iterable

from pymavlink import mavutil

from .config import COMPANION_UDP_PORT, PARAM_NAMES
from .mission import MissionItem


class MavlinkTimeout(RuntimeError):
    pass


class MavlinkClient:
    def __init__(
        self,
        endpoint: str = f"udpin:0.0.0.0:{COMPANION_UDP_PORT}",
        baud: int = 115200,
    ):
        self.endpoint = endpoint
        self.baud = baud
        self.connection = None
        self.initial_heartbeat = None
        self.target_system = 1
        self.target_component = 1

    def connect(self, timeout: float = 60.0) -> None:
        self.connection = mavutil.mavlink_connection(
            self.endpoint,
            baud=self.baud,
            source_system=191,
            source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER,
            autoreconnect=True,
        )
        deadline = time.monotonic() + timeout
        heartbeat = None
        while time.monotonic() < deadline:
            candidate = self.connection.recv_match(
                type="HEARTBEAT",
                blocking=True,
                timeout=min(0.5, max(0.0, deadline - time.monotonic())),
            )
            if candidate is None:
                continue
            try:
                source_system = int(candidate.get_srcSystem())
                source_component = int(candidate.get_srcComponent())
                autopilot = int(candidate.autopilot)
            except (AttributeError, TypeError, ValueError):
                continue
            # A router can also carry Mission Planner (SYSID 255), radios and
            # this companion's own heartbeat. Only AUTOPILOT1 with a real
            # autopilot type is allowed to select the command target.
            if (
                source_system > 0
                and source_component
                == mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
                and autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID
            ):
                heartbeat = candidate
                self.connection.target_system = source_system
                self.connection.target_component = source_component
                break
        if heartbeat is None:
            self.connection.close()
            self.connection = None
            raise MavlinkTimeout(
                f"no flight-controller AUTOPILOT1 heartbeat on {self.endpoint}"
            )
        self.initial_heartbeat = heartbeat
        self.target_system = int(self.connection.target_system)
        self.target_component = int(self.connection.target_component or 1)
        # Report companion telemetry as another component of the same vehicle,
        # so Mission Planner does not treat the Raspberry Pi as a second plane.
        self.connection.mav.srcSystem = self.target_system
        self.connection.mav.srcComponent = (
            mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER
        )
        # pymavlink helpers such as MAVFTP validate reply target fields against
        # these connection-level identities, not only against the MAVLink
        # encoder fields above.  Keep both representations synchronized or a
        # valid reply addressed to this onboard component is discarded as
        # belonging to another client.
        self.connection.source_system = int(self.connection.mav.srcSystem)
        self.connection.source_component = int(self.connection.mav.srcComponent)

    def recv(self, types, timeout: float = 1.0):
        if self.connection is None:
            raise RuntimeError("MAVLink is not connected")
        return self.connection.recv_match(type=types, blocking=True, timeout=timeout)

    def mission_message_is_for_this_client(self, message) -> bool:
        """Reject mission traffic belonging to another ground-station client.

        Mission Planner and the companion can share the router, but their
        mission protocol replies are addressed to different MAVLink component
        IDs. Consuming a reply for component 190 in the component-191 runtime
        can otherwise produce an incomplete or, worse, mis-associated mission
        readback.
        """

        if message is None or self.connection is None:
            return False
        try:
            if int(message.get_srcSystem()) != self.target_system:
                return False
            if int(message.get_srcComponent()) != self.target_component:
                return False
        except (AttributeError, TypeError, ValueError):
            return False
        local_system = int(self.connection.mav.srcSystem)
        local_component = int(self.connection.mav.srcComponent)
        target_system = getattr(message, "target_system", None)
        target_component = getattr(message, "target_component", None)
        if target_system is not None and int(target_system) not in (0, local_system):
            return False
        if target_component is not None and int(target_component) not in (
            0,
            local_component,
        ):
            return False
        mission_type = getattr(message, "mission_type", None)
        if mission_type is not None and int(mission_type) != int(
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION
        ):
            return False
        return True

    def heartbeat(self):
        return self.recv("HEARTBEAT", timeout=0.01)

    @staticmethod
    def is_armed(heartbeat) -> bool:
        return bool(
            heartbeat
            and heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        )

    @staticmethod
    def is_auto(heartbeat) -> bool:
        # ArduPlane AUTO custom mode is 10.
        return bool(
            heartbeat
            and int(heartbeat.custom_mode) == 10
            and int(heartbeat.base_mode)
            & mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        )

    def read_parameters(self, timeout: float = 12.0) -> dict[str, float]:
        assert self.connection is not None
        remaining = set(PARAM_NAMES)
        values: dict[str, float] = {}
        for name in remaining:
            self.connection.mav.param_request_read_send(
                self.target_system,
                self.target_component,
                name.encode("ascii"),
                -1,
            )
        deadline = time.monotonic() + timeout
        while remaining and time.monotonic() < deadline:
            message = self.recv("PARAM_VALUE", timeout=0.3)
            if message is None:
                continue
            name = message.param_id
            if isinstance(name, bytes):
                name = name.decode("ascii", errors="ignore")
            name = name.rstrip("\x00")
            if name in remaining:
                values[name] = float(message.param_value)
                remaining.remove(name)
        if remaining:
            raise MavlinkTimeout("missing parameters: " + ", ".join(sorted(remaining)))
        return values

    def read_parameter(self, name: str, timeout: float = 3.0) -> float:
        """Read one named parameter without requesting the full parameter list."""

        assert self.connection is not None
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.connection.mav.param_request_read_send(
                self.target_system,
                self.target_component,
                name.encode("ascii"),
                -1,
            )
            message = self.recv("PARAM_VALUE", timeout=min(0.5, timeout))
            if message is None:
                continue
            received_name = message.param_id
            if isinstance(received_name, bytes):
                received_name = received_name.decode("ascii", errors="ignore")
            if received_name.rstrip("\x00") == name:
                return float(message.param_value)
        raise MavlinkTimeout(f"parameter {name} was not received")

    def read_named_parameters(
        self, names: Iterable[str], timeout: float = 12.0,
    ) -> dict[str, float]:
        """Read an exact parameter set without discarding out-of-order replies.

        Some flight-controller builds delay individual reads while a full-list
        request still succeeds. Fall back to one read-only list request halfway
        through the same deadline; never extend the deadline or write a value.
        """
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise ValueError("parameter timeout must be finite and positive")
        assert self.connection is not None
        remaining = set(names)
        values: dict[str, float] = {}
        started = time.monotonic()
        deadline = started + timeout
        next_request = started
        list_requested = False
        while remaining and time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_request:
                for name in sorted(remaining):
                    self.connection.mav.param_request_read_send(
                        self.target_system, self.target_component,
                        name.encode("ascii"), -1,
                    )
                next_request = now + 1.0
            if not list_requested and now - started >= timeout / 2.0:
                self.connection.mav.param_request_list_send(
                    self.target_system, self.target_component,
                )
                list_requested = True
            message = self.recv("PARAM_VALUE", timeout=min(0.25, max(0.0, deadline - now)))
            if message is None:
                continue
            if (
                int(message.get_srcSystem()) != self.target_system
                or int(message.get_srcComponent()) != self.target_component
            ):
                continue
            name = message.param_id
            if isinstance(name, bytes):
                name = name.decode("ascii", errors="replace")
            name = name.rstrip("\x00")
            if name not in remaining:
                continue
            value = float(message.param_value)
            if not math.isfinite(value):
                raise ValueError(f"parameter {name} is not finite")
            values[name] = value
            remaining.remove(name)
        if remaining:
            raise MavlinkTimeout("missing parameters: " + ", ".join(sorted(remaining)))
        return values

    def set_parameters(
        self,
        values: dict[str, float],
        timeout_per_parameter: float = 3.0,
    ) -> None:
        """Set and confirm a small, explicit simulation parameter profile."""

        assert self.connection is not None
        for name, requested in values.items():
            confirmed = False
            for _attempt in range(3):
                self.connection.mav.param_set_send(
                    self.target_system,
                    self.target_component,
                    name.encode("ascii"),
                    float(requested),
                    mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
                )
                deadline = time.monotonic() + timeout_per_parameter
                while time.monotonic() < deadline:
                    message = self.recv("PARAM_VALUE", timeout=0.25)
                    if message is None:
                        continue
                    received_name = message.param_id
                    if isinstance(received_name, bytes):
                        received_name = received_name.decode("ascii", errors="ignore")
                    if received_name.rstrip("\x00") != name:
                        continue
                    if abs(float(message.param_value) - float(requested)) <= 1e-3:
                        confirmed = True
                    break
                if confirmed:
                    break
            if not confirmed:
                raise MavlinkTimeout(
                    f"parameter {name} did not confirm value {requested}"
                )

    def download_mission(self, timeout: float = 15.0) -> list[MissionItem]:
        assert self.connection is not None
        mission_type = mavutil.mavlink.MAV_MISSION_TYPE_MISSION
        self.connection.mav.mission_request_list_send(
            self.target_system, self.target_component, mission_type
        )
        count_message = None
        count_deadline = time.monotonic() + timeout
        while time.monotonic() < count_deadline:
            candidate = self.recv(
                "MISSION_COUNT",
                timeout=min(0.5, max(0.0, count_deadline - time.monotonic())),
            )
            if self.mission_message_is_for_this_client(candidate):
                count_message = candidate
                break
        if count_message is None:
            raise MavlinkTimeout("MISSION_COUNT not received")
        count = int(count_message.count)
        items: dict[int, MissionItem] = {}
        deadline = time.monotonic() + timeout
        while len(items) < count and time.monotonic() < deadline:
            missing = next(seq for seq in range(count) if seq not in items)
            self.connection.mav.mission_request_int_send(
                self.target_system, self.target_component, missing, mission_type
            )
            message = self.recv(["MISSION_ITEM_INT", "MISSION_ITEM"], timeout=0.6)
            if (
                self.mission_message_is_for_this_client(message)
                and 0 <= int(message.seq) < count
            ):
                items[int(message.seq)] = MissionItem.from_message(message)
        if len(items) != count:
            raise MavlinkTimeout(f"mission download incomplete ({len(items)}/{count})")
        self.connection.mav.mission_ack_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_MISSION_ACCEPTED,
            mission_type,
        )
        return [items[index] for index in range(count)]

    def write_partial(self, items: Iterable[MissionItem], timeout: float = 12.0) -> None:
        assert self.connection is not None
        patch = {item.seq: item for item in items}
        if not patch:
            return
        start, end = min(patch), max(patch)
        if set(patch) != set(range(start, end + 1)):
            raise ValueError("partial mission patch must have contiguous sequence numbers")
        mission_type = mavutil.mavlink.MAV_MISSION_TYPE_MISSION
        self.connection.mav.mission_write_partial_list_send(
            self.target_system, self.target_component, start, end, mission_type
        )
        deadline = time.monotonic() + timeout
        served: set[int] = set()
        while time.monotonic() < deadline:
            message = self.recv(["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"], timeout=0.5)
            if not self.mission_message_is_for_this_client(message):
                continue
            if message.get_type() == "MISSION_ACK":
                if int(message.type) != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                    raise RuntimeError(f"mission patch rejected: {message.type}")
                if served == set(patch):
                    return
                continue
            seq = int(message.seq)
            if seq not in patch:
                continue
            item = patch[seq]
            self.connection.mav.mission_item_int_send(
                self.target_system,
                self.target_component,
                item.seq,
                item.frame,
                item.command,
                item.current,
                item.autocontinue,
                item.param1,
                item.param2,
                item.param3,
                item.param4,
                int(round(item.x * 1e7)),
                int(round(item.y * 1e7)),
                item.z,
                mission_type,
            )
            served.add(seq)
        raise MavlinkTimeout("partial mission upload timed out")

    def set_current(self, seq: int) -> None:
        assert self.connection is not None
        self.connection.mav.mission_set_current_send(
            self.target_system,
            self.target_component,
            int(seq),
        )

    def set_servo(self, channel: int, pwm: int) -> None:
        assert self.connection is not None
        self.connection.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,
            float(channel),
            float(pwm),
            0,
            0,
            0,
            0,
            0,
        )

    def status_text(self, text: str, severity: int = mavutil.mavlink.MAV_SEVERITY_NOTICE) -> None:
        assert self.connection is not None
        encoded = text.encode("utf-8")[:50]
        try:
            self.connection.mav.statustext_send(severity, encoded, 0, 0)
        except TypeError:
            self.connection.mav.statustext_send(severity, encoded)

    def named_value_int(self, name: str, value: int, time_boot_ms: int) -> None:
        assert self.connection is not None
        encoded = name.encode("ascii")[:10]
        self.connection.mav.named_value_int_send(
            int(time_boot_ms) & 0xFFFFFFFF,
            encoded,
            int(value),
        )

    def named_value_float(self, name: str, value: float, time_boot_ms: int) -> None:
        assert self.connection is not None
        encoded = name.encode("ascii")[:10]
        self.connection.mav.named_value_float_send(
            int(time_boot_ms) & 0xFFFFFFFF,
            encoded,
            float(value),
        )

    def companion_heartbeat(self, status_code: int = 0) -> None:
        assert self.connection is not None
        self.connection.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            int(status_code),
            mavutil.mavlink.MAV_STATE_ACTIVE,
        )
