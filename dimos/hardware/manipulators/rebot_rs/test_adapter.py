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
        self.configuration_calls = 0
        self.enabled = False
        self.torque_limit = 14.0

    def robstride_ping_host_id(self, host_id: int, timeout_ms: int) -> tuple[int, int]:
        return self.motor_id, host_id

    def robstride_get_param_f32(self, parameter: int, timeout_ms: int) -> float:
        return self.torque_limit if parameter == 0x700B else self.position

    def robstride_write_param_f32(self, parameter: int, value: float) -> None:
        assert parameter == 0x700B
        self.torque_limit = value

    def enable(self) -> None:
        self.enabled = True

    def disable(self) -> None:
        self.enabled = False

    def ensure_mode(self, mode: object, timeout_ms: int) -> None:
        self.configuration_calls += 1

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
        self.closed = False

    @property
    def enabled(self) -> bool:
        return any(motor.enabled for motor in self.motors)

    def add_robstride_motor(self, motor_id: int, host_id: int, model: str) -> FakeMotor:
        expected_model = "rs-06" if motor_id <= 3 else "rs-00"
        assert host_id == 0xFD
        assert model == expected_model
        motor = FakeMotor(motor_id, self.positions[motor_id - 1])
        self.motors.append(motor)
        return motor

    def enable_all(self) -> None:
        for motor in self.motors:
            motor.enable()

    def disable_all(self) -> None:
        for motor in self.motors:
            motor.disable()

    def close(self) -> None:
        self.closed = True


@pytest.fixture(autouse=True)
def fake_motorbridge(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setitem(
        sys.modules, "motorbridge", SimpleNamespace(Mode=SimpleNamespace(MIT="mit"))
    )


def adapter_with(positions: list[float] | None = None) -> tuple[RebotRSAdapter, FakeController]:
    controller = FakeController("can0", positions)
    adapter = RebotRSAdapter(
        controller_factory=lambda channel: controller,
        feedback_factory=lambda channel: FakeFeedback(controller),
        rate_hz=100.0,
    )
    return adapter, controller


def test_diagnostic_trajectory_reaches_target_with_bounded_speed_and_acceleration() -> None:
    adapter, controller = adapter_with()
    adapter._active_indices = (6,)
    adapter._velocity_scale = 1.0
    adapter._velocity_max[6] = math.radians(2.0)
    adapter._acceleration[6] = math.radians(5.0)
    adapter._targets[6] = math.radians(5.0)
    previous_rate = 0.0
    previous_position = 0.0
    for _ in range(500):
        adapter._advance()
        assert 0.0 <= adapter._commands[6] <= math.radians(5.0)
        assert adapter._commands[6] >= previous_position
        assert abs(adapter._rates[6]) <= math.radians(2.0) + 1e-12
        assert abs(adapter._rates[6] - previous_rate) <= math.radians(5.0) * adapter._period + 1e-12
        assert adapter._commands[:6] == [0.0] * 6
        previous_rate = adapter._rates[6]
        previous_position = adapter._commands[6]
    assert adapter._commands[6] == pytest.approx(math.radians(5.0))
    assert not controller.motors


class FakeFeedback:
    """Timestamped status source that can preserve a stale cached motor."""

    def __init__(self, controller: FakeController) -> None:
        self.controller = controller
        self.states: dict[int, SimpleNamespace] = {}
        self.closed = False

    def snapshot(self) -> dict[int, SimpleNamespace]:
        for motor in self.controller.motors:
            if motor.feedback:
                self.states[motor.motor_id] = SimpleNamespace(
                    pos=motor.position,
                    vel=motor.velocity,
                    torq=motor.torque,
                    t_mos=motor.temperature,
                    fault_bits=0,
                    mode_bits=2 if motor.enabled else 0,
                    received_ns=time.time_ns(),
                )
        return self.states.copy()

    def close(self) -> None:
        self.closed = True


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


def test_observed_physical_zero_readings_refuse_activation_without_writes() -> None:
    positions = [
        math.radians(value) for value in (2.527, 0.21, 0.176, 0.094, 9.268, -178.407, -8.598)
    ]
    adapter, controller = adapter_with(positions)
    adapter.connect()
    assert not adapter.confirm_zero_pose()
    assert adapter.read_error()[1] == "Joints 5, 6, 7 are not at the zero pose"
    assert not adapter.activate()
    assert not controller.enabled
    assert all(not motor.sent and not motor.configuration_calls for motor in controller.motors)
    adapter.disconnect()


def test_zero_drift_is_rechecked_before_configuration_or_enable() -> None:
    adapter, controller = adapter_with()
    adapter.connect()
    assert adapter.confirm_zero_pose()
    controller.motors[5].position = math.radians(178.0)
    assert not adapter.activate()
    assert not controller.enabled
    assert all(not motor.sent and not motor.configuration_calls for motor in controller.motors)
    assert not adapter._zero_confirmed
    adapter.disconnect()


def test_failed_zero_recheck_revokes_prior_confirmation() -> None:
    adapter, controller = adapter_with()
    adapter.connect()
    assert adapter.confirm_zero_pose()
    controller.motors[4].position = math.radians(10.0)
    assert not adapter.confirm_zero_pose()
    adapter.write_clear_errors()
    assert not adapter.activate()
    assert not controller.enabled
    adapter.disconnect()


def test_gripper_activation_never_enables_or_commands_arm_joints() -> None:
    adapter, controller = adapter_with()
    adapter.connect()
    assert adapter.confirm_zero_pose()
    assert adapter.activate_gripper()
    assert not any(motor.enabled or motor.sent for motor in controller.motors[:6])
    assert controller.motors[6].enabled
    assert controller.motors[6].torque_limit == 0.5
    assert adapter.write_gripper_position(math.radians(5.0))
    assert not adapter.write_gripper_position(math.radians(6.0))
    assert not adapter.write_joint_positions([0.01] + [0.0] * 6)
    time.sleep(0.05)
    assert not any(motor.enabled or motor.sent for motor in controller.motors[:6])
    assert abs(controller.motors[6].velocity) <= math.radians(2.0)
    adapter.disconnect()
    assert not controller.motors[6].enabled


def test_one_stale_motor_faults_even_while_other_motors_report() -> None:
    adapter, controller = adapter_with()
    adapter.connect()
    adapter.confirm_zero_pose()
    assert adapter.activate()
    controller.motors[2].feedback = False
    receiver = adapter._receiver
    receiver.states[3].received_ns = time.time_ns() - 1_000_000_000
    time.sleep(0.05)
    assert "stale" in adapter.read_error()[1]
    assert not controller.enabled
    assert not adapter.write_joint_positions([0.0] * 7)
    adapter.disconnect()


def test_initial_missing_status_blocks_activation() -> None:
    adapter, controller = adapter_with()
    adapter.connect()
    adapter.confirm_zero_pose()
    controller.motors[6].feedback = False
    assert not adapter.activate_gripper()
    assert not controller.enabled
    assert not any(motor.sent or motor.configuration_calls for motor in controller.motors)
    adapter.disconnect()


def test_partial_enable_error_disables_selected_motor() -> None:
    adapter, controller = adapter_with()
    adapter.connect()
    adapter.confirm_zero_pose()

    def fail_enable() -> None:
        controller.motors[6].enabled = True
        raise RuntimeError("Enable acknowledgement missing")

    controller.motors[6].enable = fail_enable
    assert not adapter.activate_gripper()
    assert not controller.enabled
    assert not any(motor.sent for motor in controller.motors[:6])
    assert "acknowledgement" in adapter.read_error()[1]
    adapter.disconnect()
