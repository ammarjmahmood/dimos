"""Receive timestamped RobStride status without transmitting CAN frames."""

from __future__ import annotations

import math
import socket
import struct
import time
from types import SimpleNamespace

TIMESTAMP_NS = 35


def decode(frame: bytes, received_ns: int) -> tuple[int, SimpleNamespace] | None:
    """Decode a standard Linux CAN frame carrying RobStride status."""
    if len(frame) != 16:
        return None
    can_id, size, payload = struct.unpack("=IB3x8s", frame)
    if not can_id & socket.CAN_EFF_FLAG or can_id & (socket.CAN_RTR_FLAG | socket.CAN_ERR_FLAG):
        return None
    identifier = can_id & socket.CAN_EFF_MASK
    motor_id = (identifier >> 8) & 255
    if size != 8 or (identifier >> 24) & 31 != 2 or motor_id not in range(1, 8):
        return None
    position, velocity, torque, temperature = struct.unpack(">HHHH", payload)
    velocity_max, torque_max = (50.0, 36.0) if motor_id <= 3 else (33.0, 14.0)
    return motor_id, SimpleNamespace(
        pos=(position / 32767.0 - 1.0) * 4.0 * math.pi,
        vel=(velocity / 32767.0 - 1.0) * velocity_max,
        torq=(torque / 32767.0 - 1.0) * torque_max,
        t_mos=temperature / 10.0,
        fault_bits=(identifier >> 16) & 63,
        mode_bits=(identifier >> 22) & 3,
        received_ns=received_ns,
    )


class SocketCANFeedback:
    """Own a receive only socket with kernel packet arrival timestamps."""

    def __init__(self, channel: str) -> None:
        self._socket = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self._states: dict[int, SimpleNamespace] = {}
        try:
            self._socket.setsockopt(socket.SOL_SOCKET, TIMESTAMP_NS, 1)
            self._socket.bind((channel,))
            self._socket.setblocking(False)
        except Exception:
            self._socket.close()
            raise

    def snapshot(self) -> dict[int, SimpleNamespace]:
        """Drain packets while retaining original kernel arrival times."""
        for _ in range(4096):
            try:
                frame, ancillary, flags, _ = self._socket.recvmsg(16, 128)
            except BlockingIOError:
                break
            if flags & (socket.MSG_TRUNC | socket.MSG_CTRUNC):
                raise RuntimeError("CAN status or timestamp was truncated")
            received = None
            for level, kind, data in ancillary:
                if level == socket.SOL_SOCKET and kind == TIMESTAMP_NS and len(data) >= 16:
                    seconds, nanoseconds = struct.unpack("=qq", data[:16])
                    received = seconds * 1_000_000_000 + nanoseconds
            if received is None:
                raise RuntimeError("CAN packet has no kernel receive timestamp")
            status = decode(frame, received)
            if status is not None:
                motor_id, state = status
                self._states[motor_id] = state
        else:
            raise RuntimeError("CAN receive queue exceeds the bounded status drain")
        return self._states.copy()

    def close(self) -> None:
        """Close the receiver without changing motor state."""
        self._socket.close()


def require_fresh(state: SimpleNamespace | None, timeout_s: float) -> None:
    """Reject absent, old or future packet timestamps."""
    if state is None:
        raise RuntimeError("Motor status has not arrived")
    age = (time.time_ns() - state.received_ns) / 1_000_000_000
    if age < 0.0 or age > timeout_s:
        raise RuntimeError("Motor status is stale")
