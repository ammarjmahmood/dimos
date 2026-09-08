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

"""One robosuite runtime class for configured DimOS motor interfaces."""

from __future__ import annotations

from typing import Any

import numpy as np
from numpy.typing import NDArray
from robosuite.controllers import composite_controller_factory
from robosuite.models.robots import create_robot
from robosuite.robots.robot import Robot

from dimos.sim2.control.firmware import MotorFirmware
from dimos.sim2.spec import RobotConfig


class MotorRobot(Robot):  # type: ignore[misc]  # Upstream is untyped.
    def __init__(self, definition: RobotConfig, robot_id: str) -> None:
        super().__init__(
            robot_type=definition.model.__name__,
            idn=robot_id,
            composite_controller_config={"type": MotorFirmware.name, "body_parts": {}},
            base_type=None,
            gripper_type=None,
        )
        self.definition = definition
        self.enabled = True

    def load_model(self) -> None:
        # These robot-local models include their mechanical assembly. The
        # factory and namespace are upstream; no second robot registry exists.
        self.robot_model = create_robot(self.name, idn=self.idn)

    def setup_references(self) -> None:
        self.root = self.sim.model.body_name2id(self.robot_model.root_body)
        self._load_controller()

    def _load_controller(self) -> None:
        self.composite_controller = composite_controller_factory(
            MotorFirmware.name, self.sim, self.robot_model, {}
        )
        self.composite_controller.load_controller_config({}, {"definition": self.definition})

    def reset(self, deterministic: bool = False, rng: Any = None) -> None:
        servo = self.composite_controller
        servo.reset()
        self.enabled = True
        self.sim.data.qpos[servo.qpos] = servo.home[:, 0] * servo.scale + servo.offset
        self.sim.data.ctrl[servo.actuators] = np.where(
            servo.position, servo.home[:, 0] * servo.ctrl_scale + servo.ctrl_offset, 0
        )

    def control(self, action: NDArray[np.float64], policy_step: bool = False) -> None:
        if policy_step:
            self.composite_controller.set_goal(action)
        applied = self.composite_controller.run_controller({"motors": self.enabled})
        self.sim.data.ctrl[self.composite_controller.actuators] = applied["motors"]

    @property
    def action_dim(self) -> int:
        return 5 * len(self.definition.joints)
