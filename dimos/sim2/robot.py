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

"""DimOS motor commands on upstream fixed-base and legged robot lifecycles."""

from __future__ import annotations

from collections.abc import Callable
from typing import Any
import xml.etree.ElementTree as ET

import numpy as np
from numpy.typing import NDArray
from robosuite.controllers import composite_controller_factory
from robosuite.robots import ROBOT_CLASS_MAPPING
from robosuite.robots.fixed_base_robot import FixedBaseRobot
from robosuite.robots.legged_robot import LeggedRobot
from robosuite.utils.mjcf_utils import array_to_string

from dimos.sim2.control.firmware import MotorFirmware
from dimos.sim2.scene import quaternion
from dimos.sim2.sensors.spec import Camera, Imu, Mount
from dimos.sim2.spec import RobotConfig


def motor_controller_config(definition: RobotConfig) -> dict[str, Any]:
    return {
        "type": MotorFirmware.name,
        "body_parts": {},
        "composite_controller_specific_configs": {"definition": definition},
    }


class MotorRobot:
    """Override actuator semantics, not upstream model/hand/observation ownership."""

    sim: Any
    robot_model: Any
    composite_controller: MotorFirmware
    gripper: dict[str, Any]
    init_qpos: NDArray[np.float64]
    recent_qpos: Any
    recent_actions: Any
    recent_torques: Any
    _joint_positions: Any
    _ref_joint_vel_indexes: list[int]

    def __init__(
        self,
        robot_type: str,
        idn: str | int = "0",
        composite_controller_config: dict[str, Any] | None = None,
        **kwargs: Any,
    ) -> None:
        if (
            composite_controller_config is None
            or composite_controller_config["type"] != MotorFirmware.name
        ):
            raise ValueError("DimOS motor robots require the DIMOS_MOTORS controller")
        self.definition: RobotConfig = composite_controller_config[
            "composite_controller_specific_configs"
        ]["definition"]
        self.enabled = True
        super().__init__(  # type: ignore[call-arg]  # Upstream robot mixin.
            robot_type=robot_type,
            idn=idn,
            composite_controller_config=composite_controller_config,
            **kwargs,
        )

    def load_model(self) -> None:
        super().load_model()  # type: ignore[misc]
        joints = {
            self.robot_model.correct_naming(j.model_name): j.home * j.scale + j.offset
            for j in self.definition.joints
            if j.gripper is None
        }
        self.init_qpos = np.array([joints[name] for name in self.robot_model.joints])
        model = self.robot_model
        root = model.worldbody.find("body")
        if root is None or model.root_body != model.correct_naming(self.definition.root_body):
            raise ValueError("model root disagrees with the device definition")
        if not self.definition.floating:
            root.set("mocap", "true")
        keyframes = model.root.find("keyframe")
        if keyframes is not None:
            model.root.remove(keyframes)
        for sensor in self.definition.sensors:
            attachment = sensor.camera if isinstance(sensor, Camera) else sensor.site
            tag = "camera" if isinstance(sensor, Camera) else "site"
            name = model.correct_naming(sensor.model_name)
            if isinstance(attachment, Mount):
                body = model.worldbody.find(
                    f".//body[@name='{model.correct_naming(attachment.link)}']"
                )
                if body is None:
                    raise ValueError(f"{sensor.name}: unknown mount {attachment.link!r}")
                attrs = {
                    "name": name,
                    "pos": array_to_string(attachment.xyz),
                    "quat": array_to_string(quaternion(attachment.rpy)),
                }
                if isinstance(sensor, Camera):
                    fovy = sensor.fovy if sensor.fovy is not None else 60.0
                    ET.SubElement(body, tag, **attrs, fovy=str(fovy))
                else:
                    ET.SubElement(body, tag, **attrs, size=".001", rgba="0 0 0 0")
            elif model.worldbody.find(f".//{tag}[@name='{name}']") is None:
                raise ValueError(f"{sensor.name}: unknown {tag} {attachment!r}")
            if isinstance(sensor, Imu):
                prefix = model.naming_prefix + f"sensor/{sensor.name}"
                ET.SubElement(model.sensor, "gyro", name=prefix + "/gyro", site=name)
                ET.SubElement(model.sensor, "accelerometer", name=prefix + "/accel", site=name)
                ET.SubElement(
                    model.sensor,
                    "framequat",
                    name=prefix + "/quat",
                    objtype="site",
                    objname=name,
                )

    def setup_references(self) -> None:
        super().setup_references()  # type: ignore[misc]
        self.root = self.sim.model.body_name2id(self.robot_model.root_body)

    def _load_controller(self) -> None:
        self.composite_controller = composite_controller_factory(
            MotorFirmware.name, self.sim, self.robot_model, self.gripper
        )
        self.composite_controller.load_controller_config({}, {"definition": self.definition})

    def reset(self, deterministic: bool = False, rng: Any = None) -> None:
        super().reset(deterministic=deterministic, rng=rng)  # type: ignore[misc]
        servo = self.composite_controller
        self.enabled = True
        self.sim.data.ctrl[servo.actuators] = np.where(
            servo.position, servo.home[:, 0] * servo.ctrl_scale + servo.ctrl_offset, 0
        )

    def control(self, action: NDArray[np.float64], policy_step: bool = False) -> None:
        if policy_step:
            self.composite_controller.set_goal(action)
        applied = self.composite_controller.run_controller({"motors": self.enabled})
        self.sim.data.ctrl[self.composite_controller.actuators] = applied["motors"]
        if policy_step:
            self.recent_qpos.push(self._joint_positions)
            self.recent_actions.push(np.asarray(action).ravel())
            self.recent_torques.push(self.sim.data.qfrc_actuator[self._ref_joint_vel_indexes])

    @property
    def action_dim(self) -> int:
        return 5 * len(self.definition.joints)


class MotorManipulator(MotorRobot, FixedBaseRobot):  # type: ignore[misc]
    """Standard fixed-base robot with externally supplied motor commands."""


class MotorLegged(MotorRobot, LeggedRobot):  # type: ignore[misc]
    """Standard legged robot with externally supplied whole-body motor commands."""


def register_robot(runtime: type[MotorRobot]) -> Callable[[type[Any]], type[Any]]:
    """Register a custom model in the existing upstream runtime registry."""

    def register(model: type[Any]) -> type[Any]:
        ROBOT_CLASS_MAPPING[model.__name__] = runtime
        return model

    return register
