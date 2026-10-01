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

"""xArm7 with its native position servos, gripper units and wrist RGB-D camera."""

import math
import xml.etree.ElementTree as ET

import numpy as np
from numpy.typing import NDArray
from robosuite.models.grippers import register_gripper
from robosuite.models.grippers.gripper_model import GripperModel

from dimos.robot.manipulators.xarm.config import XARM7_SIM_HOME
from dimos.sim2.models import AuthoredModel, ManipulatorModel
from dimos.sim2.robot import MotorManipulator, register_robot
from dimos.sim2.sensors.spec import Camera
from dimos.sim2.spec import ControlInterface, Joint, RobotConfig
from dimos.utils.data import LfsPath


def _keep_component_assets(root: ET.Element) -> None:
    """The source xArm has mesh/material assets only; keep each component's closure."""
    worldbody = root.find("worldbody")
    asset = root.find("asset")
    assert worldbody is not None and asset is not None
    used = {
        kind: {geom.get(kind) for geom in worldbody.iter("geom")} for kind in ("mesh", "material")
    }
    for element in list(asset):
        if element.tag in used and element.get("name") not in used[element.tag]:
            asset.remove(element)


@register_gripper
class XArm7Hand(AuthoredModel, GripperModel):  # type: ignore[misc]
    """The existing xArm gripper, extracted from its one authoritative MJCF."""

    def __init__(self, idn: str = "0") -> None:
        super().__init__(LfsPath("xarm7/xarm7.xml"), idn)

    def _prepare_xml(self) -> None:
        worldbody = self.root.find("worldbody")
        assert worldbody is not None
        hand = worldbody.find(".//body[@name='xarm_gripper_base_link']")
        assert hand is not None
        worldbody.clear()
        worldbody.append(hand)
        actuator = self.root.find("actuator")
        assert actuator is not None
        for element in list(actuator):
            if element.get("name") != "gripper":
                actuator.remove(element)
        site = {"size": "0.001", "rgba": "0 0 0 0", "group": "5"}
        ET.SubElement(hand, "site", {"name": "ft_frame", **site})
        eef = ET.SubElement(hand, "body", name="eef", pos="0 0 .172")
        for name in ("grip_site", "grip_site_cylinder", "ee", "ee_x", "ee_y", "ee_z"):
            ET.SubElement(eef, "site", {"name": name, **site})
        sensors = self.root.find("sensor")
        assert sensors is not None
        ET.SubElement(sensors, "force", name="force_ee", site="ft_frame")
        ET.SubElement(sensors, "torque", name="torque_ee", site="ft_frame")
        _keep_component_assets(self.root)

    @property
    def naming_prefix(self) -> str:
        return f"gripper{self.idn}_"

    @property
    def init_qpos(self) -> NDArray[np.float64]:
        return np.zeros(6)

    @property
    def dof(self) -> int:
        return 1

    def format_action(self, action: NDArray[np.float64]) -> NDArray[np.float64]:
        return np.asarray(action).reshape(1).clip(-1, 1)

    @property
    def _important_geoms(self) -> dict[str, list[str]]:
        left = ["left_finger_pad_1", "left_finger_pad_2"]
        right = ["right_finger_pad_1", "right_finger_pad_2"]
        return {
            "left_finger": left,
            "right_finger": right,
            "left_fingerpad": left,
            "right_fingerpad": right,
        }


@register_robot(MotorManipulator)
class XArm7Model(ManipulatorModel):
    arms = ("right",)
    arm_type = "single"
    _eef_name = {"right": "link7"}
    default_gripper = {"right": "XArm7Hand"}
    base_xpos_offset = {"table": lambda length: (-length / 2, 0, 0.8), "empty": (0, 0, 0.12)}

    def __init__(self, idn: str = "0") -> None:
        super().__init__(LfsPath("xarm7/xarm7.xml"), idn)

    def _prepare_xml(self) -> None:
        root = self.root.find("./worldbody/body")
        assert root is not None
        # The source's 12 cm pedestal placement is not the robot's base offset.
        root.set("pos", "0 0 0")
        link = self.root.find(".//body[@name='link7']")
        assert link is not None
        hand = link.find("body[@name='xarm_gripper_base_link']")
        assert hand is not None
        link.remove(hand)
        for name in ("contact", "tendon", "equality"):
            element = self.root.find(name)
            assert element is not None
            element.clear()
        actuator = self.root.find("actuator")
        assert actuator is not None
        motor = actuator.find("general[@name='gripper']")
        assert motor is not None
        actuator.remove(motor)
        _keep_component_assets(self.root)


XARM7 = RobotConfig(
    model=XArm7Model,
    root_body="link_base",
    control=ControlInterface.MANIPULATOR,
    joints=(
        *(
            Joint(
                name=f"joint{i}",
                model_name=f"joint{i}",
                actuator=f"act{i}",
                home=home,
                mode="position",
                lower=lo,
                upper=hi,
            )
            for i, home, lo, hi in zip(
                range(1, 8),
                XARM7_SIM_HOME,
                (
                    -2 * math.pi,
                    -2.059,
                    -2 * math.pi,
                    -0.19198,
                    -2 * math.pi,
                    -1.69297,
                    -2 * math.pi,
                ),
                (2 * math.pi, 2.0944, 2 * math.pi, 3.927, 2 * math.pi, math.pi, 2 * math.pi),
                strict=True,
            )
        ),
        Joint(
            name="arm/gripper",
            model_name="left_driver_joint",
            actuator="gripper",
            home=850.0,
            mode="position",
            scale=-0.001,
            offset=0.85,
            ctrl_scale=-0.3,
            ctrl_offset=255.0,
            lower=0.0,
            upper=850.0,
            max_velocity=0.0,
            gripper="right",
        ),
    ),
    sensors=(Camera("wrist_camera", camera="wrist_camera"),),
)
