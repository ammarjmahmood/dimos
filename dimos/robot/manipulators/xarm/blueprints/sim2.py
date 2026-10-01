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


"""xArm perception manipulation blueprints on sim2 devices."""

from __future__ import annotations

from dimos.control.coordinator import TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.manipulation.grasping.heuristic_grasp import HeuristicGraspModule
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.manipulation_skills import ManipulationSkills
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.perception.experimental.object_scene_registration import ObjectSceneRegistrationModule
from dimos.robot.manipulators.common.blueprints import coordinator, trajectory_task
from dimos.robot.manipulators.xarm.sim2 import xarm7_simulation
from dimos.visualization.rerun.bridge import RerunBridgeModule

if global_config.simulation and global_config.simulation != "mujoco":
    raise ValueError("xarm-perception-sim2 supports --simulation mujoco")

# The scene is always simulated: this stack has no real-hardware form, and
# --scene-package / --scene-spawn pick where the arm stands.
_xarm7 = xarm7_simulation(global_config.scene_package, global_config.scene_spawn_pose)

# xarm-perception-sim on sim2 devices. The wrist camera stamps its images with
# the "arm/wrist_camera_optical" frame and publishes world -> that frame on tf,
# which is the one lookup scene registration needs for target_frame="world".
xarm_perception_sim2 = autoconnect(
    _xarm7.devices,
    ManipulationModule.blueprint(
        model=_xarm7.model,
        planning_timeout=10.0,
        visualization={"backend": "viser"},
    ),
    ManipulationSkills.blueprint(),
    PickAndPlaceModule.blueprint(planning_frame="world"),
    HeuristicGraspModule.blueprint(),
    ObjectSceneRegistrationModule.blueprint(
        target_frame="world",
        detector_backend="moondream",
        segmentation_backend="edgetam",
        detect_on_request=True,
    ),
    coordinator(
        hardware=[_xarm7.hardware],
        tasks=[
            trajectory_task(_xarm7.hardware),
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
