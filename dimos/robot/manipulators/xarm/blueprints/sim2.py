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
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.global_config import global_config
from dimos.manipulation.grasping.heuristic_grasp import HeuristicGraspModule
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.manipulation_skills import ManipulationSkills
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.manipulation.planning.utils.point_cloud_self_filter import PointCloudSelfFilter
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.perception.experimental.object_scene_registration import ObjectSceneRegistrationModule
from dimos.robot.manipulators.common.blueprints import coordinator, trajectory_task
from dimos.robot.manipulators.xarm.config import XARM7_COLLISION_LINKS
from dimos.robot.manipulators.xarm.sim2 import XARM7_MAPPING, xarm7_simulation
from dimos.visualization.rerun.bridge import RerunBridgeModule
from dimos.visualization.rerun.constants import ViewerBackend
from dimos.visualization.rerun.urdf_robot import (
    RobotModelUrdf,
    UrdfRobotJointStateRerunFactory,
    UrdfRobotStaticRerunFactory,
    bare_joint_name_mapper,
)

if global_config.simulation and global_config.simulation != "mujoco":
    raise ValueError("xarm-perception-sim2 supports --simulation mujoco")

# One resolution for the whole mapping chain. The self filter's clear mask, the
# mapper's cells and the planner's octree must all agree: a mismatched mask
# names cells the map does not hold, and a mismatched octree does not line up
# with what was mapped. 1 cm, finer than xarm-grasp's 2.5 cm, because the
# planner collides the gripper with every cell it is given: on a cluttered
# table a 2.5 cm cell adds 1.25 cm to each face of a neighbour, which is most
# of the gap a tabletop pick has. The mapper's fine layer cannot serve instead:
# it is emitted only inside the current local region, not the whole workspace.
XARM7_SIM2_VOXEL_SIZE = 0.01

# The scene is always simulated: this stack has no real-hardware form, and
# --scene-package / --scene-spawn pick where the arm stands. The wrist camera
# publishes a point cloud for the mapping chain, and the planner publishes every
# collision link on tf: the self filter drops a whole cloud when one is missing.
_xarm7 = xarm7_simulation(
    global_config.scene_package,
    global_config.scene_spawn_pose,
    robot=XARM7_MAPPING,
    tf_extra_links=XARM7_COLLISION_LINKS,
)

# Where the arm's URDF meshes live in Rerun, under the same root as the tf tree.
# The root follows the planner's "link_base" frame, which it publishes because
# link_base is among the collision links above, so the meshes stand wherever the
# scene spawns the arm. Nothing here is pickled into the bridge by reference:
# a worker that imported this module would rebuild _xarm7 from its own, bare,
# global config and get the small workbench's spawn instead of the scene's.
XARM7_RERUN_ROOT = "world/xarm7"
XARM7_RERUN_PARENT_FRAME = "tf#/link_base"
_XARM7_URDF = RobotModelUrdf(_xarm7.model.model)
_XARM7_JOINT_STATE_ENTITY = "world/coordinator_joint_state"


def rerun_bridge(viewer: ViewerBackend) -> tuple[Blueprint, ...]:
    """The Rerun bridge for this stack, or nothing at all under ``--viewer none``.

    The arm is drawn from its URDF, posed by the coordinator's joint state. The
    mapping chain's intermediate clouds are never logged: the raw wrist cloud,
    the self filter's output and clear mask, and the mapper's local maps are
    each a full point cloud at 10 Hz, and the voxel map already shows the result.
    """
    if viewer == "none":
        return ()
    return (
        RerunBridgeModule.blueprint(
            static={
                XARM7_RERUN_ROOT: UrdfRobotStaticRerunFactory(
                    urdf_path=_XARM7_URDF,
                    root_path=XARM7_RERUN_ROOT,
                    parent_frame=XARM7_RERUN_PARENT_FRAME,
                )
            },
            visual_override={
                _XARM7_JOINT_STATE_ENTITY: UrdfRobotJointStateRerunFactory(
                    urdf_path=_XARM7_URDF,
                    root_path=XARM7_RERUN_ROOT,
                    joint_name_mapper=bare_joint_name_mapper,
                ),
                "world/wrist_camera/pointcloud": None,
                "world/filtered_pointcloud": None,
                "world/voxel_clear_mask": None,
                "world/local_map": None,
                "world/local_map_fine": None,
            },
            max_hz={
                _XARM7_JOINT_STATE_ENTITY: 20.0,
                "world/global_map": 2.0,
                "world/depth_image": 2.0,
            },
        ),
    )


# xarm-perception-sim on sim2 devices. The wrist camera stamps its images and
# cloud with the "arm/wrist_camera_optical" frame and publishes world -> that
# frame on tf, which is the one lookup scene registration needs for
# target_frame="world". The planner publishes world -> each arm link, so the
# self filter reaches every link from the camera through "world" alone.
xarm_perception_sim2 = autoconnect(
    _xarm7.devices,
    ManipulationModule.blueprint(
        model=_xarm7.model,
        planning_timeout=10.0,
        visualization={"backend": "viser"},
        world_frame="world",
        voxel_map_resolution=XARM7_SIM2_VOXEL_SIZE,
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
    # Wrist camera -> self filter -> mapper -> the planner's octree obstacle.
    # The wrist camera sees the arm itself, so the arm's returns must be dropped
    # before mapping and the volume it occupies erased from the map: ray tracing
    # cannot clear what the arm permanently occludes.
    PointCloudSelfFilter.blueprint(
        model=_xarm7.model.model,
        voxel_size=XARM7_SIM2_VOXEL_SIZE,
        world_frame="world",
        # The planner publishes robot TF at 10 Hz and the camera at 10 Hz, so the
        # stock 20 ms tolerance cannot bracket a publish period and drops most
        # clouds. One full period admits them all, and the arm holds still while
        # scanning, so a transform a period old describes the same pose.
        tf_tolerance_s=0.1,
        tf_forward_tolerance_s=0.1,
    ),
    # Tabletop reach, not a room-scale lidar sweep.
    RayTracingVoxelMap.blueprint(
        voxel_size=XARM7_SIM2_VOXEL_SIZE,
        world_frame="world",
        max_range=2.0,
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
    *rerun_bridge(global_config.viewer),
).remappings(
    [
        # The camera's cloud gets its own topic: scene registration also
        # publishes a "pointcloud" (its detected objects), and that must not be
        # mapped as if the camera had seen it.
        ("arm_wrist_camera", "pointcloud", "wrist_camera/pointcloud"),
        (PointCloudSelfFilter, "pointcloud", "wrist_camera/pointcloud"),
        # The two edges whose names differ: self filter -> mapper, and mapper ->
        # the planner's octree.
        (RayTracingVoxelMap, "lidar", "filtered_pointcloud"),
        (ManipulationModule, "voxel_map", "global_map"),
    ]
)
