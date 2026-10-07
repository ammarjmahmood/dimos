"""Timestamp and wire decoding checks without a physical CAN interface."""

import math
import socket
import struct
import time
from types import SimpleNamespace

import pytest

from dimos.hardware.manipulators.rebot_rs.feedback import SocketCANFeedback, decode, require_fresh


def frame(motor_id: int = 7, flags: int = 0) -> bytes:
    identifier = socket.CAN_EFF_FLAG | (2 << 24) | (flags << 16) | (motor_id << 8) | 0xFE
    return struct.pack("=IB3x8s", identifier, 8, struct.pack(">HHHH", 32767, 32767, 32767, 390))


def test_zero_status_matches_pinned_motorbridge_decoder() -> None:
    result = decode(frame(), 123)
    assert result is not None
    motor_id, state = result
    assert motor_id == 7
    assert state.pos == state.vel == state.torq == 0.0
    assert state.t_mos == 39.0
    assert state.received_ns == 123


@pytest.mark.parametrize("payload", [b"", frame(8), struct.pack("=IB3x8s", 1, 8, b"\0" * 8)])
def test_unrelated_frames_are_not_motor_feedback(payload: bytes) -> None:
    assert decode(payload, 123) is None


@pytest.mark.parametrize("delta", [-1_000_000_000, 1_000_000_000])
def test_cached_or_future_timestamp_is_rejected(delta: int) -> None:
    with pytest.raises(RuntimeError, match="stale"):
        require_fresh(SimpleNamespace(received_ns=time.time_ns() + delta), 0.5)


def test_fault_and_mode_bits_survive_decoding() -> None:
    result = decode(frame(flags=0x84), time.time_ns())
    assert result is not None
    _, state = result
    assert state.fault_bits == 4
    assert state.mode_bits == 2
    require_fresh(state, 0.5)
    assert math.isfinite(state.pos)


def test_queued_packet_retains_kernel_timestamp(monkeypatch: pytest.MonkeyPatch) -> None:
    received = time.time_ns() - 2_000_000_000
    seconds, nanoseconds = divmod(received, 1_000_000_000)

    class Receiver:
        def __init__(self) -> None:
            self.pending = True
            self.closed = False

        def setsockopt(self, level: int, kind: int, value: int) -> None:
            assert (level, kind, value) == (socket.SOL_SOCKET, 35, 1)

        def bind(self, address: tuple[str]) -> None:
            assert address == ("can1",)

        def setblocking(self, value: bool) -> None:
            assert value is False

        def recvmsg(self, size: int, ancillary_size: int) -> tuple:
            if not self.pending:
                raise BlockingIOError
            self.pending = False
            return (
                frame(),
                [(socket.SOL_SOCKET, 35, struct.pack("=qq", seconds, nanoseconds))],
                0,
                (),
            )

        def close(self) -> None:
            self.closed = True

    receiver = Receiver()
    monkeypatch.setattr(socket, "socket", lambda *args: receiver)
    monitor = SocketCANFeedback("can1")
    state = monitor.snapshot()[7]
    assert state.received_ns == received
    with pytest.raises(RuntimeError, match="stale"):
        require_fresh(state, 0.5)
    monitor.close()
    assert receiver.closed
