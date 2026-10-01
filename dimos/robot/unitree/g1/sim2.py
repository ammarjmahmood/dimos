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

"""G1 GR00T device configuration. Policies remain in ControlCoordinator."""

from pathlib import Path

from dimos.robot.unitree.g1.control_config import (
    G1_GROOT_HOME,
    G1_GROOT_KD,
    G1_GROOT_KP,
    g1_joints,
)
from dimos.sim2.models import ManipulatorModel
from dimos.sim2.robot import MotorLegged, register_robot
from dimos.sim2.sensors.lidar.models.fibonacci import Fibonacci
from dimos.sim2.sensors.spec import Camera, Imu, Lidar
from dimos.sim2.spec import ControlInterface, Joint, RobotConfig
from dimos.utils.data import LfsPath


@register_robot(MotorLegged)
class G1Model(ManipulatorModel):
    arms = ("right", "left")
    arm_type = "bimanual"
    _eef_name = {"right": "right_wrist_yaw_link", "left": "left_wrist_yaw_link"}
    default_gripper = {"right": "EndEffectorFrame", "left": "EndEffectorFrame"}
    joint_groups = {
        "arms": tuple(
            f"{side}_{joint}_joint"
            for side in ("right", "left")
            for joint in (
                "shoulder_pitch",
                "shoulder_roll",
                "shoulder_yaw",
                "elbow",
                "wrist_roll",
                "wrist_pitch",
                "wrist_yaw",
            )
        ),
        "torso": ("waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"),
        "legs": tuple(
            f"{side}_{joint}_joint"
            for side in ("left", "right")
            for joint in ("hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll")
        ),
    }
    actuator_groups = joint_groups

    def __init__(self, idn: str = "0") -> None:
        super().__init__(
            Path(__file__).parent / "assets" / "g1_29dof.xml",
            idn,
            meshdir=LfsPath("g1_urdf/meshes"),
        )


G1_GROOT = RobotConfig(
    model=G1Model,
    root_body="pelvis",
    floating=True,
    spawn_height=0.793,
    control=ControlInterface.WHOLE_BODY,
    joints=tuple(
        Joint(
            name=name,
            model_name=name.split("/", 1)[1] + "_joint",
            actuator=name.split("/", 1)[1] + "_joint",
            home=home,
            kp=kp,
            kd=kd,
        )
        for name, home, kp, kd in zip(
            g1_joints,
            G1_GROOT_HOME,
            G1_GROOT_KP,
            G1_GROOT_KD,
            strict=True,
        )
    ),
    sensors=(
        Imu("imu", site="control_imu"),
        Lidar(
            "lidar",
            "mid360_link",
            Fibonacci,
            rate_hz=10.0,
            output_frame="sensor",
            maximum_world_elevation=0.0,
        ),
        Camera("camera", camera="front_camera"),
    ),
)
