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

"""Robosuite controller plugin for motor firmware, not robot policy execution."""

from __future__ import annotations

from typing import Any

import numpy as np
from numpy.typing import NDArray
from robosuite.controllers.composite.composite_controller import (
    CompositeController,
    register_composite_controller,
)

from dimos.sim2.spec import RobotConfig


@register_composite_controller
class MotorFirmware(CompositeController):  # type: ignore[misc]  # Upstream is untyped.
    name = "DIMOS_MOTORS"

    def load_controller_config(
        self, part_controller_config: Any, composite_controller_specific_config: Any = None
    ) -> None:
        definition: RobotConfig = composite_controller_specific_config["definition"]
        self.definition = definition
        model = self.sim.model
        owners = [
            self.robot_model if j.gripper is None else self.grippers[j.gripper]
            for j in definition.joints
        ]
        self.qpos = np.array(
            [
                model.get_joint_qpos_addr(owner.correct_naming(j.model_name))
                for owner, j in zip(owners, definition.joints, strict=True)
            ]
        )
        self.dofs = np.array(
            [
                model.get_joint_qvel_addr(owner.correct_naming(j.model_name))
                for owner, j in zip(owners, definition.joints, strict=True)
            ]
        )
        self.actuators = np.array(
            [
                model.actuator_name2id(owner.correct_naming(j.actuator))
                for owner, j in zip(owners, definition.joints, strict=True)
            ]
        )
        self.scale = np.array([j.scale for j in definition.joints])
        self.offset = np.array([j.offset for j in definition.joints])
        self.position = np.array([j.mode == "position" for j in definition.joints])
        self.ctrl_scale = np.array([j.ctrl_scale for j in definition.joints])
        self.ctrl_offset = np.array([j.ctrl_offset for j in definition.joints])
        self.home = np.array([[j.home, 0, j.kp, j.kd, 0] for j in definition.joints])
        self.reset()

    def reset(self) -> None:
        self.command = self.home.copy()

    def update_state(self) -> None:
        """No policy/IK state: the servo reads current joint state at each substep."""

    def set_goal(self, all_action: NDArray[np.float64]) -> None:
        command = np.asarray(all_action, dtype=np.float64).reshape(self.home.shape)
        if not np.isfinite(command).all():
            raise ValueError("motor commands must be finite")
        self.command[:] = command

    def run_controller(self, enabled_parts: dict[str, bool]) -> dict[str, NDArray[np.float64]]:
        q, dq, kp, kd, feedforward = self.command.T
        measured = self.sim.data.qpos[self.qpos]
        velocity = self.sim.data.qvel[self.dofs]
        effort = kp * (q * self.scale + self.offset - measured)
        effort += kd * (dq * self.scale - velocity) + feedforward
        position = q * self.ctrl_scale + self.ctrl_offset
        if not enabled_parts["motors"]:
            effort[:] = 0
            position = (measured - self.offset) / self.scale * self.ctrl_scale + self.ctrl_offset
        values = np.where(self.position, position, effort)
        ranges = self.sim.model.actuator_ctrlrange[self.actuators]
        values = np.where(
            self.sim.model.actuator_ctrllimited[self.actuators],
            np.clip(values, ranges[:, 0], ranges[:, 1]),
            values,
        )
        return {"motors": values}

    @property
    def action_limits(self) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        return np.full(self.home.size, -np.inf), np.full(self.home.size, np.inf)
