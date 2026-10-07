"""Tests for the Seeed Studio reBot B601 RS adapter."""

from __future__ import annotations

import math
import sys
import time
from types import SimpleNamespace

import pytest

from dimos.hardware.manipulators.rebot_rs.adapter import MOTORS, RebotRSAdapter
from dimos.hardware.manipulators.spec import ControlMode, ManipulatorAdapter


class FakeMotor:
    """Predictable RobStride motor used without a CAN interface."""

    def __init__(self, motor_id: int, position: float = 0.0) -> None:
        self.motor_id = motor_id
        self.position = position
        self.velocity = 0.0
        self.torque = 0.0
        self.temperature = 35.0
        self.sent: list[tuple[float, float, float, float, float]] = []
        self.feedback = True

    def robstride_ping_host_id(self, host_id: int, timeout_ms: int) -> tuple[int, int]:
        return self.motor_id, host_id

    def robstride_get_param_f32(self, parameter: int, timeout_ms: int) -> float:
        return self.position

    def ensure_mode(self, mode: object, timeout_ms: int) -> None:
        return None

    def set_can_timeout_ms(self, timeout_ms: int) -> None:
        assert timeout_ms == 250

    def send_mit(
        self, position: float, velocity: float, kp: float, kd: float, torque: float
    ) -> None:
        self.sent.append((position, velocity, kp, kd, torque))
        self.position = position
        self.velocity = velocity

    def get_state(self) -> SimpleNamespace | None:
        if not self.feedback:
            return None
        return SimpleNamespace(
            pos=self.position, vel=self.velocity, torq=self.torque, t_mos=self.temperature
        )


class FakeController:
    """Motorbridge controller substitute that records lifecycle calls."""

    def __init__(self, channel: str, positions: list[float] | None = None) -> None:
        self.channel = channel
        self.positions = positions or [0.0] * 7
        self.motors: list[FakeMotor] = []
        self.enabled = False
        self.closed = False

    def add_robstride_motor(self, motor_id: int, host_id: int, model: str) -> FakeMotor:
        expected_model = "rs-06" if motor_id <= 3 else "rs-00"
        assert host_id == 0xFD
        assert model == expected_model
        motor = FakeMotor(motor_id, self.positions[motor_id - 1])
        self.motors.append(motor)
        return motor

    def enable_all(self) -> None:
        self.enabled = True

    def disable_all(self) -> None:
        self.enabled = False

    def close(self) -> None:
        self.closed = True


@pytest.fixture(autouse=True)
def fake_motorbridge(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setitem(
        sys.modules, "motorbridge", SimpleNamespace(Mode=SimpleNamespace(MIT="mit"))
    )


def adapter_with(positions: list[float] | None = None) -> tuple[RebotRSAdapter, FakeController]:
    controller = FakeController("can0", positions)
    adapter = RebotRSAdapter(controller_factory=lambda channel: controller, rate_hz=100.0)
    return adapter, controller


def test_registry_protocol_and_motor_topology() -> None:
    adapter, controller = adapter_with()
    assert isinstance(adapter, ManipulatorAdapter)
    assert adapter.connect()
    assert controller.channel == "can0"
    assert [
        (motor.motor_id, model) for motor, (_, model) in zip(controller.motors, MOTORS, strict=True)
    ] == list(MOTORS)
    assert not controller.enabled
    assert not adapter.read_enabled()
    assert adapter.get_dof() == 7
    assert adapter.get_info().model == "reBot Arm B601 RS"
    limits = adapter.get_limits()
    assert len(limits.position_lower) == len(limits.position_upper) == len(limits.velocity_max) == 7
    assert limits.position_lower == pytest.approx(
        [math.radians(value) for value in (-150.0, 0.0, 0.0, -90.0, -90.0, -180.0, 0.0)]
    )
    assert limits.position_upper == pytest.approx(
        [math.radians(value) for value in (150.0, 220.0, 220.0, 90.0, 90.0, 180.0, 345.0)]
    )
    adapter.disconnect()
    assert controller.closed


def test_connection_never_enables_and_activation_requires_zero() -> None:
    adapter, controller = adapter_with()
    adapter.connect()
    assert not adapter.activate()
    assert not controller.enabled
    assert adapter.confirm_zero_pose()
    assert not controller.enabled
    assert adapter.activate()
    assert controller.enabled
    assert adapter.read_enabled()
    adapter.disconnect()
    assert not controller.enabled


def test_bad_zero_is_rejected_and_reconnect_invalidates_confirmation() -> None:
    adapter, controller = adapter_with([0.0, 0.0, math.radians(8.0), 0.0, 0.0, 0.0, 0.0])
    adapter.connect()
    assert not adapter.confirm_zero_pose()
    assert "Joints 3" in adapter.read_error()[1]
    assert not adapter.activate()
    adapter.disconnect()
    controller.positions = [0.0] * 7
    assert adapter.connect()
    assert not adapter.activate()
    adapter.disconnect()


def test_position_stream_is_bounded_and_stop_holds() -> None:
    adapter, controller = adapter_with()
    adapter.connect()
    assert adapter.confirm_zero_pose()
    assert adapter.set_control_mode(ControlMode.SERVO_POSITION)
    assert adapter.activate()
    target = [0.05] * 6 + [0.1]
    assert adapter.write_joint_positions(target, velocity=0.5)
    assert not adapter.write_joint_positions([0.0] * 6)
    assert not adapter.write_joint_positions([math.inf] * 7)
    time.sleep(0.08)
    assert all(motor.sent for motor in controller.motors)
    assert max(adapter.read_joint_positions()) > 0.0
    assert adapter.write_stop()
    held = adapter.read_joint_positions()
    time.sleep(0.03)
    assert adapter.read_joint_positions() == pytest.approx(held, abs=0.01)
    adapter.disconnect()


def test_gripper_is_seventh_joint_and_limits_are_configurable() -> None:
    adapter, _ = adapter_with()
    adapter.connect()
    adapter.confirm_zero_pose()
    adapter.activate()
    assert adapter.write_gripper_position(math.radians(30.0))
    time.sleep(0.08)
    assert adapter.read_gripper_position() > 0.0
    assert not adapter.write_gripper_position(math.radians(500.0))
    adapter.disconnect()


def test_temperature_fault_latches_and_blocks_new_commands() -> None:
    adapter, controller = adapter_with()
    adapter.connect()
    adapter.confirm_zero_pose()
    adapter.activate()
    controller.motors[0].temperature = 120.0
    time.sleep(0.05)
    assert "temperature" in adapter.read_error()[1]
    assert not adapter.write_joint_positions([0.0] * 7)
    adapter.disconnect()
