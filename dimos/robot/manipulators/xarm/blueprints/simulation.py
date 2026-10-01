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

from robosuite.environments.manipulation.lift import Lift

from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.manipulation.grasping.heuristic_grasp import HeuristicGraspModule
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.manipulation_skills import ManipulationSkills
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.perception.experimental.object_scene_registration import ObjectSceneRegistrationModule
from dimos.robot.manipulators.common.blueprints import coordinator, trajectory_task
from dimos.robot.manipulators.xarm.config import (
    XARM7_SIM_BASE_POSE,
    make_xarm7_sim_robot_config,
)
from dimos.robot.manipulators.xarm.sim2 import XARM7, XArm7Model
from dimos.sim2.blueprint import Simulation, simulation
from dimos.sim2.module import SimulationModule
from dimos.sim2.spec import RobosuiteTask, RobotInstance
from dimos.visualization.rerun.bridge import RerunBridgeModule


def _perception_stack(devices: Simulation, base_pose: PoseStamped) -> Blueprint:
    """The same manipulation stack for authored scenes and upstream tasks."""
    hardware = devices.hardware["arm"]
    return autoconnect(
        ManipulationModule.blueprint(
            model=make_xarm7_sim_robot_config(base_pose=base_pose),
            planning_timeout=10.0,
            visualization={"backend": "viser"},
        ),
        ManipulationSkills.blueprint(),
        PickAndPlaceModule.blueprint(planning_frame="world"),
        HeuristicGraspModule.blueprint(),
        devices.blueprint,
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
    ).lifetime_dependencies([(ControlCoordinator, SimulationModule)])


_simulation = simulation(
    scene=Path(__file__).resolve().parents[1] / "assets" / "table.xml",
    robots={"arm": RobotInstance(XARM7, xyz=(0.0, 0.0, 0.12))},
    sim_id="xarm7",
    timestep=0.002,
)
xarm_perception_sim = autoconnect(
    _perception_stack(_simulation, XARM7_SIM_BASE_POSE),
)

# Lift positions the robot using this model's table offset, not an authored spawn.
_lift_table_size = (0.8, 0.8, 0.05)
_lift_base_offset = XArm7Model.base_xpos_offset["table"]
assert callable(_lift_base_offset)
_lift = simulation(
    scene=RobosuiteTask(Lift, {"table_full_size": _lift_table_size, "seed": 42}),
    robots={"arm": RobotInstance(XARM7)},
    sim_id="xarm7-lift",
    timestep=0.002,
)
xarm_robosuite_lift = autoconnect(
    _perception_stack(
        _lift,
        PoseStamped(
            frame_id="world",
            position=Vector3(*_lift_base_offset(_lift_table_size[0])),
        ),
    )
)
