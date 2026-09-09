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

"""M20 public SDK robot with ideal sensors; locomotion lives in ControlCoordinator."""

from math import pi
from pathlib import Path

from dimos.control.tasks.m20_locomotion_task.m20_locomotion_task import HOME, KD, KP, POLICY_JOINTS
from dimos.sim2.models import RobotModel
from dimos.sim2.robot import MotorLegged, register_robot
from dimos.sim2.sensors.lidar.models.fibonacci import Fibonacci
from dimos.sim2.sensors.spec import Camera, Imu, Lidar, Mount
from dimos.sim2.spec import ControlInterface, Joint, RobotConfig
from dimos.utils.data import LfsPath

ASSETS = Path(__file__).parent / "assets"
POLICY_PATH = LfsPath("m20_sdk/policy.onnx")


@register_robot(MotorLegged)
class M20Model(RobotModel):
    joint_groups = {
        "legs": tuple(name for name in POLICY_JOINTS if "wheel" not in name),
        "base": tuple(name for name in POLICY_JOINTS if "wheel" in name),
    }
    actuator_groups = joint_groups

    def __init__(self, idn: str = "0") -> None:
        super().__init__(ASSETS / "m20.xml", idn, meshdir=LfsPath("m20_sdk/meshes"))


M20 = RobotConfig(
    model=M20Model,
    root_body="base_link",
    floating=True,
    spawn_height=0.6,
    control=ControlInterface.WHOLE_BODY,
    joints=tuple(
        Joint(
            name=f"m20/{name}",
            model_name=name,
            actuator=name,
            home=home,
            kp=kp,
            kd=kd,
        )
        for name, home, kp, kd in zip(POLICY_JOINTS, HOME, KP, KD, strict=True)
    ),
    sensors=(
        Imu("imu", Mount("base_link", xyz=(0.0632, -0.0268, 0.0435))),
        Lidar(
            "lidar",
            Mount("base_link", xyz=(0.2, 0, 0.15)),
            Fibonacci(elevation_min=-45, elevation_max=45, max_range=25),
            output_frame="sensor",
        ),
        Camera("camera", Mount("base_link", xyz=(0.39, 0, 0.08), rpy=(pi / 2, 0, -pi / 2))),
    ),
)
