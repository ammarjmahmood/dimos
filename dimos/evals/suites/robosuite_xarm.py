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

"""One skills-enabled task per exported robosuite scene; filter with --tags <scene>."""

import json

from dimos.evals.environments.lib.robosuite_grading import (
    door,
    lift,
    nut_assembly,
    pick_place,
    stack,
    sustained,
    tool_hang,
)
from dimos.evals.environments.mujoco_sim import MujocoEnvironment
from dimos.evals.suites.mujoco_xarm import PERCEPTION_MODULES
from dimos.evals.types import EvalCase, Suite
from dimos.utils.data import LfsPath

GUIDANCE = (
    "Locate the objects with the wrist camera and use the robot's manipulation skills. "
    "The arm base is mounted at world z=0.912 m. Read the current robot pose; "
    "preserve its orientation for top-down moves. Keep the final result steady for "
    "at least two seconds before finishing."
)

TOOL_SITES = (
    "stand_mount_site",
    "frame_tip_site",
    "frame_mount_site",
    "frame_intersection_site",
    "frame_hang_site",
    "tool_hole1_center",
)
TOOL_GEOMS = (
    "stand_base",
    "stand_wall0",
    "stand_wall1",
    "stand_wall2",
    "stand_wall3",
    "tool_hole1_hc_0",
    "tool_hole1_hc_4",
)


def environment(
    scene: str,
    bodies: tuple[str, ...],
    *,
    sites: tuple[str, ...] = (),
    geoms: tuple[str, ...] = (),
) -> MujocoEnvironment:
    return MujocoEnvironment(
        blueprint=["xarm-perception-sim", "mcp-server", "observe-skill"],
        disable=PERCEPTION_MODULES,
        scene=LfsPath(f"robosuite_xarm/{scene}/scene.xml"),
        base_height=0.912,
        tracked_bodies=bodies,
        module_env={
            "MUJOCOSIMMODULE__EVALUATION_ROBOT_BODY": "link_base",
            "MUJOCOSIMMODULE__EVALUATION_SITES": json.dumps(sites),
            "MUJOCOSIMMODULE__EVALUATION_GEOMS": json.dumps(geoms),
        },
        ready_streams=("color_image", "coordinator_joint_state", "evaluation_state"),
        recorded_topics=(
            "color_image",
            "camera_info",
            "coordinator_joint_state",
            "tf",
            "evaluation_state",
        ),
    )


SUITE: Suite = [
    EvalCase(
        id="robosuite_xarm_lift_cube",
        inputs=(
            "Pick up the red cube and hold it at least 5 cm above its resting position. "
            "The table top is at world z=0.80 m; the cube is 4 cm wide. " + GUIDANCE
        ),
        environment=environment("lift", ("cube_main",), sites=("table_top",)),
        grade=sustained(lift, ("cube_main",)),
        timeout_s=600.0,
        tags=frozenset({"mujoco", "robosuite", "manipulation", "lift"}),
    ),
    EvalCase(
        id="robosuite_xarm_open_door",
        inputs=(
            "Open the door by at least 0.3 radians (about 17 degrees), then release it "
            "and leave it open. The door has a movable handle. " + GUIDANCE
        ),
        environment=environment("door", ("Door_frame", "Door_door")),
        grade=sustained(door, ("Door_door",)),
        timeout_s=900.0,
        tags=frozenset({"mujoco", "robosuite", "manipulation", "door"}),
    ),
    EvalCase(
        id="robosuite_xarm_place_can",
        inputs=(
            "Move the soda can from the source bin to the destination compartment marked "
            "by the transparent can. Place it upright near the marker's center, release it, "
            "and move the gripper away. The bin floors are at world z=0.82 m. " + GUIDANCE
        ),
        environment=environment("pick_place", ("Can_main", "VisualCan_main")),
        grade=sustained(pick_place, ("Can_main",)),
        timeout_s=900.0,
        tags=frozenset({"mujoco", "robosuite", "manipulation", "pick_place"}),
    ),
    EvalCase(
        id="robosuite_xarm_stack_cubes",
        inputs=(
            "Stack the smaller red cube centrally on top of the larger green cube. "
            "Leave both cubes upright on the table, release the red cube, and move the "
            "gripper away. The table top is at world z=0.80 m. " + GUIDANCE
        ),
        environment=environment("stack", ("cubeA_main", "cubeB_main"), sites=("table_top",)),
        grade=sustained(stack, ("cubeA_main", "cubeB_main")),
        timeout_s=900.0,
        tags=frozenset({"mujoco", "robosuite", "manipulation", "stack"}),
    ),
    EvalCase(
        id="robosuite_xarm_hang_tool",
        inputs=(
            "Insert the hook frame into the upright stand, then hang the wrench by its "
            "larger hole on the horizontal hook. Release all pieces and move the gripper "
            "away so the wrench hangs unsupported by the robot. The table top is at "
            "world z=0.80 m. " + GUIDANCE
        ),
        environment=environment(
            "tool_hang",
            ("stand_root", "frame_root", "tool_root"),
            sites=TOOL_SITES,
            geoms=TOOL_GEOMS,
        ),
        grade=sustained(tool_hang, ("stand_root", "frame_root", "tool_root")),
        timeout_s=1200.0,
        tags=frozenset({"mujoco", "robosuite", "manipulation", "tool_hang"}),
    ),
    EvalCase(
        id="robosuite_xarm_assemble_square_nut",
        inputs=(
            "Pick up the square nut and lower its hole over the matching square peg until "
            "the nut rests flat on the table. Release it and move the gripper away. "
            "The table top is at world z=0.82 m. " + GUIDANCE
        ),
        environment=environment("nut_assembly", ("SquareNut_main", "peg1"), sites=("table_top",)),
        grade=sustained(nut_assembly, ("SquareNut_main",)),
        timeout_s=900.0,
        tags=frozenset({"mujoco", "robosuite", "manipulation", "nut_assembly"}),
    ),
]
