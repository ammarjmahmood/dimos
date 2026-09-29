# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""DeepRobotics M20 public actor: 57 observations, 12 leg and 4 wheel actions.

Contract ported from sdk_deploy m20_policy_runner.hpp at 2367375f922f.
Upstream attribution/license: robot/deeprobotics/m20/assets/LICENSE.sdk_deploy.
Run at 50 Hz; the motor driver applies PD at the physics rate.
"""

from __future__ import annotations

from pathlib import Path
import threading
from typing import TYPE_CHECKING

import numpy as np
from numpy.typing import NDArray
import onnxruntime as ort  # type: ignore[import-untyped]

from dimos.control.hardware_interface import ConnectedWholeBody
from dimos.control.task import BaseControlTask, CoordinatorState, JointCommandOutput, ResourceClaim
from dimos.hardware.whole_body.spec import MotorCommand
from dimos.protocol.service.spec import BaseConfig
from dimos.robot.deeprobotics.m20.constants import HOME, KD, KP, POLICY_JOINTS

if TYPE_CHECKING:
    from dimos.control.coordinator import TaskConfig
    from dimos.control.hardware_interface import ConnectedHardware
    from dimos.msgs.geometry_msgs.Twist import Twist

ACTION_SCALE = (0.125, 0.25, 0.25) * 4 + (5.0,) * 4


class M20LocomotionConfig(BaseConfig):
    model_path: Path
    hardware_id: str = "m20"
    command_timeout: float = 0.5
    max_velocity: tuple[float, float, float] = (0.7, 0.5, 0.7)


class M20LocomotionTask(BaseControlTask):
    def __init__(self, name: str, config: M20LocomotionConfig, priority: int = 50) -> None:
        self._name = name
        self.config = config
        self.joint_names = [f"{config.hardware_id}/{joint}" for joint in POLICY_JOINTS]
        self._claim = ResourceClaim(frozenset(self.joint_names), priority=priority, mode=None)
        if config.command_timeout <= 0 or not all(v > 0 for v in config.max_velocity):
            raise ValueError("M20 command timeout and velocity limits must be positive")
        options = ort.SessionOptions()  # type: ignore[attr-defined]
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        self._session = ort.InferenceSession(
            str(config.model_path), sess_options=options, providers=["CPUExecutionProvider"]
        )
        inputs, outputs = self._session.get_inputs(), self._session.get_outputs()
        if len(inputs) != 1 or inputs[0].name != "obs" or inputs[0].shape != [1, 57]:
            raise ValueError("M20 actor must accept obs[1,57]")
        if len(outputs) != 1 or outputs[0].name != "actions" or outputs[0].shape != [1, 16]:
            raise ValueError("M20 actor must return actions[1,16]")
        self._lock = threading.RLock()
        self._active = False
        self._stopping = False
        self._last_action = np.zeros(16, dtype=np.float32)
        self._command = np.zeros(3, dtype=np.float32)
        self._command_time = float("-inf")
        self._home = np.asarray(HOME, dtype=np.float32)
        self._scale = np.asarray(ACTION_SCALE, dtype=np.float32)

    def claim(self) -> ResourceClaim:
        return self._claim

    def is_active(self) -> bool:
        with self._lock:
            return self._active or self._stopping

    def start(self) -> None:
        self.reset_runtime_state(reactivate=True)

    def stop(self) -> None:
        with self._lock:
            self._active = False
            self._stopping = True  # Dispatch damping once through normal arbitration.
            self._command.fill(0)

    def reset_runtime_state(self, reactivate: bool | None = None) -> bool:
        with self._lock:
            self._last_action.fill(0)
            self._command.fill(0)
            self._command_time = float("-inf")
            self._stopping = False
            if reactivate is not None:
                self._active = reactivate
            return True

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        # The actor's previous-action history is invalid after another task intervenes.
        self.reset_runtime_state()

    def on_twist_command(self, twist: Twist, t_now: float) -> bool:
        value = np.array([twist.linear.x, twist.linear.y, twist.angular.z], dtype=np.float32)
        if not np.isfinite(value).all():
            return False
        with self._lock:
            self._command = np.clip(
                value, -np.asarray(self.config.max_velocity), self.config.max_velocity
            ).astype(np.float32)
            self._command_time = t_now
        return True

    def _observation(self, state: CoordinatorState) -> NDArray[np.float32]:
        imu = state.imu[self.config.hardware_id]
        w, x, y, z = imu.quaternion
        obs = np.empty(57, dtype=np.float32)
        obs[:3] = np.asarray(imu.gyroscope) * 0.25
        obs[3:6] = (2 * (w * y - x * z), -2 * (y * z + w * x), -(w * w - x * x - y * y + z * z))
        obs[6:9] = (
            self._command
            if 0 <= state.t_now - self._command_time <= self.config.command_timeout
            else 0
        )
        obs[9:25] = [state.joints.joint_positions[name] for name in self.joint_names]
        obs[9:25] -= self._home
        obs[21:25] = 0.0  # Wheel angles are unobserved; wheel velocities are observed.
        obs[25:41] = (
            np.asarray([state.joints.joint_velocities[name] for name in self.joint_names]) * 0.05
        )
        obs[41:57] = self._last_action
        if not np.isfinite(obs).all():
            raise ValueError("M20 observations must be finite")
        return obs

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        with self._lock:
            if self._stopping:
                self._stopping = False
                commands = [MotorCommand(q=0, dq=0, kp=0, kd=kd) for kd in KD]
            elif not self._active:
                return None
            else:
                try:
                    action = np.asarray(
                        self._session.run(["actions"], {"obs": self._observation(state)[None]})[0],
                        dtype=np.float32,
                    ).reshape(16)
                    if not np.isfinite(action).all():
                        raise ValueError("M20 actor produced nonfinite actions")
                except Exception:
                    self.stop()
                    raise
                self._last_action[:] = action
                targets = self._home + action * self._scale
                commands = [
                    MotorCommand(
                        q=float(targets[i]) if i < 12 else 0.0,
                        dq=float(targets[i]) if i >= 12 else 0.0,
                        kp=KP[i],
                        kd=KD[i],
                        tau=0.0,
                    )
                    for i in range(16)
                ]
            return JointCommandOutput(
                joint_names=self.joint_names, motor_commands=commands, mode=None
            )


def create_task(cfg: TaskConfig, hardware: dict[str, ConnectedHardware]) -> M20LocomotionTask:
    config = M20LocomotionConfig.model_validate(cfg.params)
    device = hardware.get(config.hardware_id)
    expected = {f"{config.hardware_id}/{joint}" for joint in POLICY_JOINTS}
    if not isinstance(device, ConnectedWholeBody) or set(device.joint_names) != expected:
        raise ValueError("M20 locomotion requires the 16 named M20 whole-body joints")
    if set(cfg.joint_names) != expected:
        raise ValueError("M20 locomotion must claim all 16 M20 joints")
    return M20LocomotionTask(cfg.name, config, cfg.priority)
