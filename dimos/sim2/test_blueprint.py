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

from dataclasses import replace

import pytest

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.robot.manipulators.xarm.sim2 import XARM7
from dimos.sim2.blueprint import simulation
from dimos.sim2.scene import scene_path
from dimos.sim2.sensors.spec import Camera, Mount
from dimos.sim2.spec import RobotInstance


def test_multiple_robots_and_rgb_cameras_have_separate_typed_ports():
    rgb = replace(XARM7, sensors=(Camera("front", Mount("link7"), depth=False),))
    devices = simulation(
        scene=scene_path(None, "workbench.xml"),
        robots={"left": RobotInstance(rgb), "right": RobotInstance(rgb)},
        viewer=False,
    )
    blueprint = devices.blueprint
    parsed = BlueprintConfigParser(blueprint).parse(environ={})
    left = next(atom for atom in blueprint.active_blueprints if atom.name == "left_front")
    assert {stream.name for stream in left.streams} == {"color_image", "camera_info", "tf"}
    assert blueprint.remapping_map[("left_front", "color_image")] == "left/front/color_image"
    assert blueprint.remapping_map[("right_front", "color_image")] == "right/front/color_image"
    assert blueprint.remapping_map[("left_connection", "joint_command")] == "left/joint_command"
    assert blueprint.remapping_map[("right_connection", "joint_command")] == "right/joint_command"
    sensor = parsed.module_kwargs("left_front")["sensor"]
    assert isinstance(sensor, Camera)
    assert sensor == rgb.sensors[0]
    assert devices.hardware["left"].address == "sim/left"
    assert devices.hardware["right"].address == "sim/right"
    assert devices.hardware["left"].adapter_kwargs["definition"] is rgb


def test_second_camera_does_not_require_shared_module_changes():
    robot = XARM7.with_sensor(Camera("front", Mount("link_base"), depth=False))
    blueprint = simulation(
        scene=scene_path(None, "workbench.xml"),
        robots={"arm": RobotInstance(robot)},
        viewer=False,
    ).blueprint
    assert blueprint.remapping_map[("arm_front", "color_image")] == "arm/front/color_image"
    assert (
        blueprint.remapping_map[("arm_wrist_camera", "color_image")]
        == "arm/wrist_camera/color_image"
    )


def test_duplicate_sensor_names_fail_before_deployment():
    with pytest.raises(ValueError, match="sensor names must be nonempty and unique"):
        replace(XARM7, sensors=(*XARM7.sensors, *XARM7.sensors))
