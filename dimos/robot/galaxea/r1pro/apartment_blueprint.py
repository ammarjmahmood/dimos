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

"""Interactive everyday-object apartment with ACT skills and full-map KronkNav."""

from dataclasses import replace
from pathlib import Path

from dimos.constants import RECORDINGS_DIR
from dimos.control.components import make_twist_base_joints
from dimos.control.coordinator import TaskConfig
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.navigation.global_planner.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.robot.galaxea.r1pro.apartment_coordinator import R1ProApartmentCoordinator
from dimos.robot.galaxea.r1pro.apartment_navigation import (
    APARTMENT_FRAME,
    APARTMENT_NAV_TASK,
    ApartmentNavigation,
)
from dimos.robot.galaxea.r1pro.apartment_sim import R1ProApartmentSim
from dimos.robot.galaxea.r1pro.apartment_skills import R1ProApartmentSkills
from dimos.robot.galaxea.r1pro.navigation_blueprint import CONTROLLER_ARTIFACT
from dimos.robot.galaxea.r1pro.primitive_blueprint import (
    PRIMITIVE_BASE_ID,
    build_primitive_blueprint,
)

APARTMENT_PROMPT = """You control an R1Pro in a physical MuJoCo apartment simulation.
Start idle and inspect get_scene before acting. Every launch has a new seed and randomized props.
Use object IDs, kind, color and robot-relative coordinates to resolve the requested item and hand.
If no hand was requested, use arm=auto; the skill compares measured reachability and policy coverage.
For placement with two occupied hands, use the hand holding the requested object.
The named household kinds are cup, bottle, drink_carton, glue_stick and toy_block.
Measured positions and supports are simulator ground truth. Never substitute another object or hand.
pick_object means approach, grasp, lift and HOLD. It does not place or release.
place_object places the already held object in the requested support region and releases it.
go_to navigates to dining_table, kitchen or worktable, preserving every held item. It never releases.
For 'take X to Y', pick X, wait for completion, navigate to Y, and wait. Place only if requested.
Use get_surfaces to inspect physical support regions. Tray placement and unloading use explicit skills.
For every accepted action call wait_for_action until terminal completion before the next action.
Acceptance is not success. Report physical failures; never claim an item was moved just from a command.
On failure stop the sequence. If recovery_required is true, call recover_action once and wait for
its result. It preserves a confirmed hold or restores an empty failed arm through DimOS planning.
Report whether recovery succeeded. Do not silently retry the grasp, change targets, or reset the scene.
ACT performs grasps and placements; DimOS handles positioning and KronkNav/holonomic base execution.
Keep responses brief and grounded in the tool results."""


def build_apartment_blueprint(policies: Path, *, agent: bool = False) -> Blueprint:
    stack = build_primitive_blueprint(
        policies,
        agent=agent,
        simulator=R1ProApartmentSim,
        skills=R1ProApartmentSkills,
        coordinator=R1ProApartmentCoordinator,
        system_prompt=APARTMENT_PROMPT,
    )
    atoms = []
    for atom in stack.blueprints:
        kwargs = dict(atom.kwargs)
        if atom.module is R1ProApartmentSim and (policies / "workspace.json").exists():
            kwargs["workspace_file"] = policies / "workspace.json"
        if atom.module is R1ProApartmentCoordinator:
            kwargs["tasks"] = [
                *kwargs["tasks"],
                TaskConfig(
                    name=APARTMENT_NAV_TASK,
                    type="holonomic_pose_follower",
                    joint_names=make_twist_base_joints(PRIMITIVE_BASE_ID),
                    priority=30,
                    params={
                        "artifact_path": str(CONTROLLER_ARTIFACT),
                        "speed": 0.12,
                        "lookahead": 0.025,
                        "regulate_horizon": 0.2,
                        "goal_tolerance": 0.01,
                        "orientation_tolerance": 0.01,
                        "approach_decel": 0.12,
                        "stop_hold_s": 0.5,
                    },
                ),
            ]
        atoms.append(replace(atom, kwargs=kwargs))
    return (
        autoconnect(
            replace(stack, blueprints=tuple(atoms)),
            ApartmentNavigation.blueprint(),
            MLSPlannerNative.blueprint(
                world_frame="world",
                base_frame=APARTMENT_FRAME,
                voxel_size=0.06,
                robot_height=1.70,
                surface_closing_radius=0.12,
                node_spacing_m=0.3,
                wall_clearance_m=0.35,
                wall_buffer_m=0.65,
                wall_buffer_weight=20.0,
                step_threshold_m=0.07,
                goal_tolerance=0.01,
                viz_publish_hz=1.0,
                worker_threads=2,
            ),
        )
        .remappings(
            [
                (MLSPlannerNative, "path", "planned_path"),
                (MLSPlannerNative, "tf", "navigation_tf"),
                (ApartmentNavigation, "base_odom", f"/{PRIMITIVE_BASE_ID}/odom"),
                (R1ProApartmentCoordinator, "path", "execution_path"),
            ]
        )
        .global_config(transport="zenoh", viewer="none", simulation="mujoco", n_workers=7)
    )


r1pro_apartment_sim = build_apartment_blueprint(
    RECORDINGS_DIR / "r1pro-act-task/policy-apartment-preview"
).global_config()
r1pro_apartment_sim_agent = build_apartment_blueprint(
    RECORDINGS_DIR / "r1pro-act-task/policy-apartment-preview", agent=True
).global_config()
