# Copyright 2025-2026 Dimensional Inc.
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
"""The dual OpenYAM grasping stack, on hardware by default.

```bash
dimos run dual-openyam-grasp --left-can-port follower_l --right-can-port follower_r
dimos run dual-openyam-grasp ... --graspgen                  # GraspGenX grasps
dimos run dual-openyam-grasp ... --record ""                 # no recording
dimos run dual-openyam-grasp                                 # in-memory arms, no CAN
```

The port of the xArm grasp stack: coordinator, planner, pick-and-place, scene
registration and a grasp provider, with a fixed depth camera over the table
feeding perception and one more on each wrist. Both arms are planning groups
with their own grippers, so ``pick_object`` takes ``left_manipulator`` or
``right_manipulator``.

Every run records the policy-training streams (joint states, planned and
accepted joint commands, all three cameras, TF) to
``recordings/<run-id>/memory.db`` unless ``--record ""`` turns it off.

The grasp provider is chosen at import time from ``global_config.graspgen``,
as the xArm stack chooses sim or hardware: the heuristic top-down grasp by
default, GraspGenX with ``--graspgen``.
"""

from __future__ import annotations

from dataclasses import replace
import math

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.manipulation.grasping.grasp_gen_x.module import GraspGenXModule
from dimos.manipulation.grasping.heuristic_grasp import HeuristicGraspModule
from dimos.manipulation.manipulation_skills import ManipulationSkills
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.perception.experimental.object_scene_registration import ObjectSceneRegistrationModule
from dimos.robot.manipulators.common.blueprints import planner
from dimos.robot.manipulators.dual_openyam.blueprints.basic import (
    DualOpenYamCoordinator,
    dual_openyam_gripper_task,
    dual_openyam_trajectory_task,
)
from dimos.robot.manipulators.dual_openyam.config import (
    DUAL_OPENYAM_SIDES,
    dual_openyam_model_config,
)

# {side}_grasp_frame is 10 cm below the gripper link on its axis. The finger
# pads (tip_left.stl, tip_right.stl at the URDF's closed zero position) meet on
# that axis from 12.7 to 14.7 cm below the gripper link; plan to the pad centre.
DUAL_OPENYAM_TCP_OFFSET = (0.0, 0.0, -0.037)

# The same gripper in GraspGenX's convention: origin on the gripper link,
# approach along +Z (the URDF's -Z), jaws closing along X (the URDF's Y). The
# fingers slide 4.7 cm each, so the open and half-open sweep volumes share
# their centre; the pads are up to 2.8 cm wide and 2 cm tall.
DUAL_OPENYAM_GRIPPER_SWEEP_VOLUME = {
    "extents_open": (0.094, 0.028, 0.020),
    "offset_open": (0.0, 0.0, 0.137),
    "extents_half_open": (0.047, 0.028, 0.020),
    "offset_half_open": (0.0, 0.0, 0.137),
    "fingertip_depth": 0.1468,
}
# GraspGenX frame -> {side}_tcp: swap the X and Y axes and flip Z, then move
# 13.7 cm along the approach to the pad centre.
DUAL_OPENYAM_GRASP_FRAME_TO_TCP = (
    (0.0, 1.0, 0.0, 0.0),
    (1.0, 0.0, 0.0, 0.0),
    (0.0, 0.0, -1.0, 0.137),
    (0.0, 0.0, 0.0, 1.0),
)

# Pink's defaults do not converge on this model; the WebXR teleop uses these.
DUAL_OPENYAM_GRASP_PINK = PinkKinematicsConfig(
    dt=0.01,
    position_cost=8.0,
    orientation_cost=2.0,
    posture_cost=0.01,
    joint_limit_posture_margin=0.3,
    lm_damping=0.01,
    gain=1.0,
)

# Fixed camera on the centre post, in the frame midway between the arm bases
# (x forward, y toward the left arm, z up). Solved from four 60 mm AprilTags
# taped to the table at tape-measured positions, 3.7 px reprojection RMS.
DUAL_OPENYAM_CAMERA_TRANSFORM = Transform(
    translation=Vector3(x=0.0160, y=-0.0057, z=0.4598),
    rotation=Quaternion(-0.00254, 0.51604, 0.00190, 0.85656),  # xyzw, pitched 62 deg down
    frame_id="world",
    child_frame_id="camera_link",
)

# RealSense D405 serials on the benchmark rig; override per camera with
# --realsensecamera.serial-number, --left_wrist/realsensecamera.serial-number
# and --right_wrist/realsensecamera.serial-number.
DUAL_OPENYAM_OVERHEAD_CAMERA_SERIAL = "230322272156"
DUAL_OPENYAM_WRIST_CAMERA_SERIALS = {"left": "260322276650", "right": "260322272983"}

# Streams a run keeps for ACT and VLA training: the joint states, the plans
# the planner sent and the commands the hardware accepted, every camera's
# colour, depth and intrinsics, and TF. Globs on the stream names.
DUAL_OPENYAM_RECORD_TOPICS = ",".join(
    [
        "coordinator_joint_state",
        "planned_joint_trajectory",
        "applied_joint_position_command",
        "color_image",
        "depth_image",
        "camera_info",
        "tf",
        *(
            f"{side}_wrist/{stream}"
            for side in DUAL_OPENYAM_SIDES
            for stream in ("color_image", "depth_image", "camera_info", "tf")
        ),
    ]
)

DUAL_OPENYAM_GRASP_PROMPTS = [
    "soup can",
    "mustard bottle",
    "cracker box",
    "banana",
    "plate",
    "toothpaste",
    "toy block",
    "mug",
    "marker",
    "towel",
]


def dual_openyam_grasp_model_config() -> RobotModelConfig:
    """Planning model in the world frame with a fingertip TCP per arm."""
    config = dual_openyam_model_config(base_pose=PoseStamped(frame_id="world"))
    model = config.model
    for side in DUAL_OPENYAM_SIDES:
        model = model.with_fixed_frame(
            f"{side}_tcp", f"{side}_grasp_frame", xyz=DUAL_OPENYAM_TCP_OFFSET
        )
    config.model = model
    config.planning_groups = [
        replace(group, tip_link=f"{group.name.split('_')[0]}_tcp")
        for group in config.planning_groups
    ]
    return config


def dual_openyam_grasp_provider(graspgen: bool) -> Blueprint:
    if graspgen:
        return GraspGenXModule.blueprint(
            gripper=DUAL_OPENYAM_GRIPPER_SWEEP_VOLUME,
            grasp_frame_to_tcp=DUAL_OPENYAM_GRASP_FRAME_TO_TCP,
        )
    # The half turn about Y turns the top-down grasp into the OpenYAM grasp
    # frame, which points back at the wrist; the extra yaws matter because the
    # two arms accept different wrist bands over the same object.
    return HeuristicGraspModule.blueprint(tool_rotation_rpy=(0.0, math.pi, 0.0), yaw_candidates=8)


def dual_openyam_wrist_camera(side: str) -> Blueprint:
    """A wrist D405 under its own namespace, so its streams and frames never
    collide with the overhead camera that feeds perception."""
    if side not in DUAL_OPENYAM_SIDES:
        raise ValueError(f"side must be 'left' or 'right', got {side!r}")
    return RealSenseCamera.blueprint(
        width=640,
        height=480,
        fps=15,
        enable_pointcloud=False,
        serial_number=DUAL_OPENYAM_WRIST_CAMERA_SERIALS[side],
    ).namespace(f"{side}_wrist")


def dual_openyam_grasp_modules(*, graspgen: bool) -> tuple[Blueprint, ...]:
    return (
        planner(
            model=dual_openyam_grasp_model_config(),
            kinematics=DUAL_OPENYAM_GRASP_PINK,
            default_speed_scale=0.25,
            static_transforms=[DUAL_OPENYAM_CAMERA_TRANSFORM],
            visualization={"backend": "viser"},
            world_frame="world",
        ),
        ManipulationSkills.blueprint(),
        PickAndPlaceModule.blueprint(planning_frame="world", pregrasp_along_tool_z=True),
        dual_openyam_grasp_provider(graspgen),
        RealSenseCamera.blueprint(
            width=640,
            height=480,
            fps=30,
            enable_pointcloud=True,
            serial_number=DUAL_OPENYAM_OVERHEAD_CAMERA_SERIAL,
        ),
        dual_openyam_wrist_camera("left"),
        dual_openyam_wrist_camera("right"),
        ObjectSceneRegistrationModule.blueprint(
            target_frame="world",
            detector_backend="moondream",
            segmentation_backend="edgetam",
            detect_on_request=True,
            distance_threshold=0.08,
            min_detections_for_permanent=3,
            max_distance=1.5,
            use_aabb=True,
            max_obstacle_width=0.06,
        ),
        DualOpenYamCoordinator.blueprint(
            instance_name="ControlCoordinator",
            tasks=[
                dual_openyam_trajectory_task(),
                dual_openyam_gripper_task("left"),
                dual_openyam_gripper_task("right"),
            ],
        ),
    )


def dual_openyam_grasp_blueprint(*, graspgen: bool) -> Blueprint:
    return autoconnect(*dual_openyam_grasp_modules(graspgen=graspgen)).global_config(
        record="sqlite", record_topics=DUAL_OPENYAM_RECORD_TOPICS
    )


# Assigned through autoconnect so the registry generator sees it.
dual_openyam_grasp = autoconnect(
    *dual_openyam_grasp_modules(graspgen=bool(global_config.graspgen))
).global_config(record="sqlite", record_topics=DUAL_OPENYAM_RECORD_TOPICS)
