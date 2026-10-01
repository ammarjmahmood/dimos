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

"""Simulation xArm perception manipulation blueprints."""

from __future__ import annotations

from pathlib import Path

from dimos.control.coordinator import TaskConfig
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.manipulation.grasping.heuristic_grasp import HeuristicGraspModule
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.manipulation_skills import ManipulationSkills
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.perception.experimental.object_scene_registration import ObjectSceneRegistrationModule
from dimos.robot.manipulators.common.blueprints import coordinator, trajectory_task
from dimos.robot.manipulators.xarm.config import (
    XARM7_POUR_SIM_PATH,
    XARM7_SIM_PATH,
    make_xarm7_sim_hardware,
    make_xarm7_sim_module_kwargs,
    make_xarm7_sim_robot_config,
)
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.visualization.rerun.bridge import RerunBridgeModule


def _perception_sim(scene: Path) -> tuple[Blueprint, ...]:
    """The xArm7 perception stack simulated in the MuJoCo scene at ``scene``."""
    hardware = make_xarm7_sim_hardware(scene)
    return (
        ManipulationModule.blueprint(
            model=make_xarm7_sim_robot_config(),
            planning_timeout=10.0,
            visualization={"backend": "viser"},
        ),
        ManipulationSkills.blueprint(),
        PickAndPlaceModule.blueprint(planning_frame="world"),
        HeuristicGraspModule.blueprint(),
        MujocoSimModule.blueprint(**make_xarm7_sim_module_kwargs(scene)),
        ObjectSceneRegistrationModule.blueprint(
            target_frame="world",
            detector_backend="moondream",
            segmentation_backend="edgetam",
            detect_on_request=True,
        ),
        coordinator(
            hardware=[hardware],
            tasks=[
                trajectory_task(hardware),
                TaskConfig(
                    name="arm_gripper",
                    type="gripper",
                    joint_names=["arm/gripper"],
                    priority=20,
                ),
            ],
        ),
        RerunBridgeModule.blueprint(),
    )


xarm_perception_sim = autoconnect(*_perception_sim(XARM7_SIM_PATH))

xarm_pour_sim = autoconnect(*_perception_sim(XARM7_POUR_SIM_PATH))
