"""DimOS manipulator adapter for the Seeed Studio reBot Arm B601 RS."""

from __future__ import annotations

from collections.abc import Callable
import math
import threading
import time
from typing import Any

from dimos.hardware.manipulators.spec import ControlMode, ManipulatorInfo
from dimos.hardware.spec import JointLimits

MOTORS = tuple((motor_id, "rs-06" if motor_id <= 3 else "rs-00") for motor_id in range(1, 8))
HOST_ID = 0xFD
GAINS = (
    (50.0, 2.0),
    (100.0, 4.0),
    (100.0, 4.0),
    (40.0, 2.0),
    (40.0, 1.5),
    (40.0, 1.5),
    (12.0, 0.05),
)
ARM_LIMITS_DEG = (
    (-155.0, 155.0),
    (-3.0, 175.0),
    (-3.0, 175.0),
    (-85.0, 85.0),
    (-85.0, 85.0),
    (-179.0, 179.0),
)
ZERO_TOLERANCE = math.radians(6.0)
TRACKING_LIMIT = math.radians(15.0)
TRACKING_GRACE_S = 0.4
FEEDBACK_TIMEOUT_S = 0.5
TEMPERATURE_LIMIT_C = 80.0
DEFAULT_VELOCITY = tuple(
    math.radians(value) for value in (20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 150.0)
)
DEFAULT_ACCELERATION = tuple(
    math.radians(value) for value in (120.0, 120.0, 120.0, 120.0, 120.0, 120.0, 300.0)
)


class RebotRSAdapter:
    """Seven joint RobStride adapter with guarded activation and local streaming."""

    def __init__(
        self,
        channel: str = "can0",
        controller_factory: Callable[[str], Any] | None = None,
        gripper_limits_deg: tuple[float, float] = (-180.0, 360.0),
        rate_hz: float = 200.0,
        **_: object,
    ) -> None:
        if not math.isfinite(rate_hz) or not 50.0 <= rate_hz <= 500.0:
            raise ValueError("rate_hz must be between 50 and 500")
        if len(gripper_limits_deg) != 2 or not all(
            math.isfinite(value) for value in gripper_limits_deg
        ):
            raise ValueError("gripper_limits_deg must contain two finite values")
        if gripper_limits_deg[0] >= gripper_limits_deg[1]:
            raise ValueError("gripper lower limit must be below its upper limit")
        self._channel = channel
        self._factory = controller_factory
        self._period = 1.0 / rate_hz
        self._lower = [math.radians(pair[0]) for pair in ARM_LIMITS_DEG] + [
            math.radians(gripper_limits_deg[0])
        ]
        self._upper = [math.radians(pair[1]) for pair in ARM_LIMITS_DEG] + [
            math.radians(gripper_limits_deg[1])
        ]
        self._velocity_max = list(DEFAULT_VELOCITY)
        self._acceleration = list(DEFAULT_ACCELERATION)
        self._controller = None
        self._motors: list[Any] = []
        self._positions = [0.0] * 7
        self._velocities = [0.0] * 7
        self._efforts = [0.0] * 7
        self._temperatures: list[float | None] = [None] * 7
        self._commands = [0.0] * 7
        self._targets = [0.0] * 7
        self._rates = [0.0] * 7
        self._enabled = False
        self._zero_confirmed = False
        self._control_mode = ControlMode.POSITION
        self._error = ""
        self._feedback_at = 0.0
        self._tracking_since: float | None = None
        self._lock = threading.RLock()
        self._running = threading.Event()
        self._thread: threading.Thread | None = None

    def connect(self) -> bool:
        """Connect and verify all seven motors without enabling torque."""
        if self.is_connected():
            return True
        factory = self._factory
        if factory is None:
            from motorbridge import Controller

            factory = Controller
        controller = factory(self._channel)
        try:
            motors = [
                controller.add_robstride_motor(motor_id, HOST_ID, model)
                for motor_id, model in MOTORS
            ]
            for (motor_id, _), motor in zip(MOTORS, motors, strict=True):
                device_id, _ = motor.robstride_ping_host_id(HOST_ID, 700)
                if device_id != motor_id:
                    raise RuntimeError(f"Motor {motor_id} did not answer with its own ID")
            positions = [motor.robstride_get_param_f32(0x7019, 500) for motor in motors]
            if not all(math.isfinite(value) for value in positions):
                raise RuntimeError("Position feedback is not finite")
        except Exception:
            controller.close()
            raise
        with self._lock:
            self._controller = controller
            self._motors = motors
            self._positions = positions
            self._commands = positions.copy()
            self._targets = positions.copy()
            self._rates = [0.0] * 7
            self._feedback_at = time.monotonic()
            self._zero_confirmed = False
            self._enabled = False
            self._error = ""
        return True

    def disconnect(self) -> None:
        """Disable torque, close the bus and invalidate zero confirmation."""
        if self._enabled:
            self.deactivate()
        self._running.clear()
        if self._thread and self._thread is not threading.current_thread():
            self._thread.join(timeout=1.0)
        if self._controller is not None:
            self._controller.close()
        with self._lock:
            self._controller = None
            self._motors = []
            self._thread = None
            self._zero_confirmed = False
            self._enabled = False

    def is_connected(self) -> bool:
        """Return whether the SocketCAN controller is open."""
        return self._controller is not None

    def confirm_zero_pose(self) -> bool:
        """Confirm all motors are at their configured zero pose after connect."""
        if not self.is_connected() or self._enabled:
            return False
        positions = self._read_positions_direct()
        outside = [
            index + 1 for index, value in enumerate(positions) if abs(value) > ZERO_TOLERANCE
        ]
        if outside:
            self._error = "Joints " + ", ".join(map(str, outside)) + " are not at the zero pose"
            return False
        self._zero_confirmed = True
        self._error = ""
        return True

    def activate(self) -> bool:
        """Enable MIT position holding after zero pose confirmation."""
        return self.write_enable(True)

    def deactivate(self) -> bool:
        """Stop command progression and disable all motor torque."""
        if not self.is_connected():
            return False
        self.write_stop()
        self._running.clear()
        if self._thread and self._thread is not threading.current_thread():
            self._thread.join(timeout=1.0)
        self._controller.disable_all()
        with self._lock:
            self._enabled = False
            self._thread = None
        return True

    def get_info(self) -> ManipulatorInfo:
        """Return the adapter identity and seven owned joints."""
        return ManipulatorInfo(vendor="Seeed Studio", model="reBot Arm B601 RS", dof=7)

    def get_dof(self) -> int:
        """Return six arm joints plus the gripper motor."""
        return 7

    def get_limits(self) -> JointLimits:
        """Return local limits for all seven motor coordinates."""
        return JointLimits(self._lower.copy(), self._upper.copy(), self._velocity_max.copy())

    def set_control_mode(self, mode: ControlMode) -> bool:
        """Select planned or servo position mode while disabled."""
        if self._enabled or mode not in {ControlMode.POSITION, ControlMode.SERVO_POSITION}:
            return False
        self._control_mode = mode
        return True

    def get_control_mode(self) -> ControlMode:
        """Return the selected position control mode."""
        return self._control_mode

    def read_joint_positions(self) -> list[float]:
        """Return the latest seven motor positions."""
        if not self.is_connected():
            raise RuntimeError("Arm is disconnected")
        if not self._enabled:
            return self._read_positions_direct()
        with self._lock:
            return self._positions.copy()

    def read_joint_velocities(self) -> list[float]:
        """Return the latest seven motor velocities."""
        self._require_fresh_feedback()
        with self._lock:
            return self._velocities.copy()

    def read_joint_efforts(self) -> list[float]:
        """Return the latest seven motor torque estimates."""
        self._require_fresh_feedback()
        with self._lock:
            return self._efforts.copy()

    def read_state(self) -> dict[str, int]:
        """Return enabled state and DimOS control mode index."""
        return {"state": int(self._enabled), "mode": list(ControlMode).index(self._control_mode)}

    def read_error(self) -> tuple[int, str]:
        """Return the latched adapter error."""
        return (1, self._error) if self._error else (0, "")

    def write_joint_positions(self, positions: list[float], velocity: float = 1.0) -> bool:
        """Set a bounded seven joint target for the local streaming loop."""
        if (
            not self._enabled
            or self._error
            or len(positions) != 7
            or not math.isfinite(velocity)
            or not 0.0 < velocity <= 1.0
        ):
            return False
        if any(
            not math.isfinite(value) or value < lower or value > upper
            for value, lower, upper in zip(positions, self._lower, self._upper, strict=True)
        ):
            return False
        with self._lock:
            self._targets = positions.copy()
            self._velocity_scale = velocity
        return True

    def write_joint_velocities(self, velocities: list[float]) -> bool:
        """Reject velocity commands because this adapter streams position targets."""
        return False

    def write_stop(self) -> bool:
        """Brake command progression and hold the latest measured position."""
        if not self.is_connected():
            return False
        with self._lock:
            self._commands = self._positions.copy()
            self._targets = self._positions.copy()
            self._rates = [0.0] * 7
        return True

    def write_enable(self, enable: bool) -> bool:
        """Enable only after zero confirmation, or disable immediately."""
        if not self.is_connected():
            return False
        if not enable:
            return self.deactivate() if self._enabled else True
        if self._enabled:
            return True
        if not self._zero_confirmed or self._error:
            return False
        from motorbridge import Mode

        with self._lock:
            for motor in self._motors:
                motor.ensure_mode(Mode.MIT, 1000)
                motor.set_can_timeout_ms(250)
            positions = self._read_positions_direct()
            self._positions = positions
            self._commands = positions.copy()
            self._targets = positions.copy()
            self._rates = [0.0] * 7
            self._velocity_scale = 1.0
            for motor, position, gains in zip(self._motors, positions, GAINS, strict=True):
                motor.send_mit(position, 0.0, gains[0], gains[1], 0.0)
            self._controller.enable_all()
            self._enabled = True
            self._feedback_at = time.monotonic()
            self._tracking_since = None
        self._running.set()
        self._thread = threading.Thread(target=self._loop, name="rebot-rs-mit", daemon=True)
        self._thread.start()
        return True

    def read_enabled(self) -> bool:
        """Return whether motor torque is enabled."""
        return self._enabled

    def write_clear_errors(self) -> bool:
        """Clear a software error only while disabled."""
        if self._enabled:
            return False
        self._error = ""
        return True

    def read_cartesian_position(self) -> dict[str, float] | None:
        """Report that Cartesian feedback is not provided by this adapter."""
        return None

    def write_cartesian_position(self, pose: dict[str, float], velocity: float = 1.0) -> bool:
        """Reject Cartesian commands because planning belongs above the adapter."""
        return False

    def read_gripper_position(self) -> float | None:
        """Return the seventh motor coordinate in radians."""
        return self.read_joint_positions()[6]

    def write_gripper_position(self, position: float) -> bool:
        """Set only the gripper motor target in radians."""
        positions = self.read_joint_positions()
        positions[6] = position
        return self.write_joint_positions(positions)

    def read_force_torque(self) -> list[float] | None:
        """Report that no Cartesian force torque sensor is available."""
        return None

    def _read_positions_direct(self) -> list[float]:
        positions = [motor.robstride_get_param_f32(0x7019, 500) for motor in self._motors]
        if not all(math.isfinite(value) for value in positions):
            raise RuntimeError("Position feedback is not finite")
        with self._lock:
            self._positions = positions
            self._feedback_at = time.monotonic()
        return positions.copy()

    def _require_fresh_feedback(self) -> None:
        if not self.is_connected():
            raise RuntimeError("Arm is disconnected")
        if time.monotonic() - self._feedback_at > FEEDBACK_TIMEOUT_S:
            raise RuntimeError("Motor feedback is stale")

    def _loop(self) -> None:
        next_tick = time.perf_counter()
        while self._running.is_set():
            try:
                with self._lock:
                    self._advance()
                    for motor, position, rate, gains in zip(
                        self._motors, self._commands, self._rates, GAINS, strict=True
                    ):
                        motor.send_mit(position, rate, gains[0], gains[1], 0.0)
                    self._feedback()
            except (RuntimeError, ValueError, OSError) as error:
                self._fail(f"Motor communication failed: {error}")
            next_tick += self._period
            delay = next_tick - time.perf_counter()
            if delay > 0.0:
                time.sleep(delay)
            else:
                next_tick = time.perf_counter()

    def _advance(self) -> None:
        for index in range(7):
            error = self._targets[index] - self._commands[index]
            acceleration = self._acceleration[index]
            maximum = self._velocity_max[index] * self._velocity_scale
            desired = (
                math.copysign(min(maximum, math.sqrt(2.0 * acceleration * abs(error))), error)
                if error
                else 0.0
            )
            change = max(
                -acceleration * self._period,
                min(acceleration * self._period, desired - self._rates[index]),
            )
            self._rates[index] += change
            step = self._rates[index] * self._period
            if abs(step) >= abs(error) and (
                not error or math.copysign(1.0, step) == math.copysign(1.0, error)
            ):
                self._commands[index] = self._targets[index]
                self._rates[index] = 0.0
            else:
                self._commands[index] += step

    def _feedback(self) -> None:
        now = time.monotonic()
        fresh = False
        for index, motor in enumerate(self._motors):
            state = motor.get_state()
            if state is None or not math.isfinite(state.pos):
                continue
            self._positions[index] = state.pos
            self._velocities[index] = state.vel if math.isfinite(state.vel) else 0.0
            self._efforts[index] = state.torq if math.isfinite(state.torq) else 0.0
            if math.isfinite(state.t_mos):
                self._temperatures[index] = state.t_mos
            fresh = True
        if fresh:
            self._feedback_at = now
        elif now - self._feedback_at > FEEDBACK_TIMEOUT_S:
            raise RuntimeError("Motor feedback stopped arriving")
        if any(value is not None and value > TEMPERATURE_LIMIT_C for value in self._temperatures):
            raise RuntimeError("A motor is above the temperature limit")
        error = max(
            abs(measured - commanded)
            for measured, commanded in zip(self._positions[:6], self._commands[:6], strict=True)
        )
        if error > TRACKING_LIMIT:
            if self._tracking_since is None:
                self._tracking_since = now
            elif now - self._tracking_since > TRACKING_GRACE_S:
                raise RuntimeError("An arm joint is not following its command")
        else:
            self._tracking_since = None

    def _fail(self, reason: str) -> None:
        with self._lock:
            self._error = reason
            self._commands = self._positions.copy()
            self._targets = self._positions.copy()
            self._rates = [0.0] * 7
        self._running.clear()
