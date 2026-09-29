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

"""Independent left/right ACT pick and place with SDK feedback base execution."""

from dataclasses import replace
from pathlib import Path

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.constants import RECORDINGS_DIR
from dimos.control.components import HardwareComponent, HardwareType, make_twist_base_joints
from dimos.control.coordinator import TaskConfig
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.imitation.policy.skills import PolicySkills
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.planning.planners.config import RRTConnectPlannerConfig
from dimos.robot.assets.model import PlanarBaseDefinition
from dimos.robot.galaxea.r1pro.config import (
    R1PRO_MODEL,
    R1PRO_PLANAR_BASE,
    R1PRO_UPPER_BODY_PLANNING_JOINTS,
    make_r1pro_planar_model_config,
)
from dimos.robot.galaxea.r1pro.grasping_blueprint import build_r1pro_manipulation
from dimos.robot.galaxea.r1pro.object_primitives import primitive_profile
from dimos.robot.galaxea.r1pro.primitive_coordinator import R1ProPrimitiveCoordinator
from dimos.robot.galaxea.r1pro.primitive_policies import (
    R1ProLeftPickPolicy,
    R1ProLeftPlacePolicy,
    R1ProRightPickPolicy,
    R1ProRightPlacePolicy,
)
from dimos.robot.galaxea.r1pro.primitive_sim import R1ProPrimitiveSim
from dimos.robot.galaxea.r1pro.primitive_skills import R1ProPrimitiveSkills
from dimos.simulation.engines.mujoco_sim_module import SimCameraSpec

PRIMITIVE_BASE_ID = "r1pro_primitive_base"
PRIMITIVE_BASE_TASK = "base_trajectory"
POLICIES = {
    ("pick", "right"): R1ProRightPickPolicy,
    ("place", "right"): R1ProRightPlacePolicy,
    ("pick", "left"): R1ProLeftPickPolicy,
    ("place", "left"): R1ProLeftPlacePolicy,
}


def build_primitive_blueprint(
    policies: Path,
    *,
    agent: bool = False,
    simulator: type[R1ProPrimitiveSim] = R1ProPrimitiveSim,
    skills: type[R1ProPrimitiveSkills] = R1ProPrimitiveSkills,
    coordinator: type[R1ProPrimitiveCoordinator] = R1ProPrimitiveCoordinator,
    system_prompt: str | None = None,
) -> Blueprint:
    base_joints = make_twist_base_joints(PRIMITIVE_BASE_ID)
    source = build_r1pro_manipulation(
        scene_path=RECORDINGS_DIR / "r1pro-primitives/scene.xml",
        artifact=str(policies / "pick-right"),
        device="cuda",
        headless=False,
        simulator=simulator,
        policy_module=R1ProRightPickPolicy,
        task_description="Pick the selected object and hold it.",
        prepare_scene_on_build=True,
        background_camera_rendering=True,
        coordinator_type=coordinator,
        velocity_base=HardwareComponent(
            hardware_id=PRIMITIVE_BASE_ID,
            hardware_type=HardwareType.BASE,
            joints=base_joints,
            adapter_type="transport_lcm",
            auto_enable=True,
        ),
        navigation_task=TaskConfig(
            name=PRIMITIVE_BASE_TASK,
            type="planar_base_trajectory",
            joint_names=base_joints,
            priority=30,
            params={
                "max_linear": 0.15,
                "max_angular": 0.12,
                "position_goal_tolerance": 0.005,
                "orientation_goal_tolerance": 0.005,
                "settle_timeout": 10.0,
                "stop_hold_s": 0.5,
            },
        ),
        viewer_lookat=(0.35, 0, 0.8),
        viewer_distance=2.4,
        viewer_azimuth=145,
        viewer_elevation=-35,
    )
    atoms = []
    for atom in source.blueprints:
        kwargs = dict(atom.kwargs)
        if atom.module in (R1ProRightPickPolicy, PolicySkills):
            continue
        if atom.module is simulator:
            kwargs["extra_cameras"] = [
                SimCameraSpec(
                    name=f"{arm}_wrist", stream=f"{arm}_wrist", width=160, height=160, fps=40
                )
                for arm in ("right", "left")
            ]
        elif atom.module is coordinator:
            tasks = []
            for task in kwargs["tasks"]:
                if task.name == "policy_rollout":
                    continue
                if task.name == "tray_manipulation":
                    task = replace(task, name="joint_trajectory")
                tasks.append(task)
            for arm in ("right", "left"):
                joints = list(primitive_profile("pick", arm).action.demonstration.joints)
                tasks.append(
                    TaskConfig(
                        name=f"primitive_{arm}",
                        type="trajectory",
                        joint_names=joints,
                        priority=30,
                        params={
                            "start_position_tolerance": 0.05,
                            "velocity_limits": dict(zip(joints, [2.0] * 7 + [0.25], strict=True)),
                        },
                    )
                )
            kwargs["tasks"] = tasks
        atoms.append(replace(atom, kwargs=kwargs))
    stack = replace(source, blueprints=tuple(atoms))
    model = make_r1pro_planar_model_config()
    # Limit the planner itself so its timed trajectories match this loaded sim base.
    base = PlanarBaseDefinition(
        root_link=R1PRO_PLANAR_BASE.root_link,
        joint_names=R1PRO_PLANAR_BASE.joint_names,
        velocity_limits=(0.12, 0.12, 0.12),
        acceleration_limits=(0.12, 0.12, 0.08),
    )
    model.model = R1PRO_MODEL.with_planar_base(base)
    modules = [
        stack,
        skills.blueprint(),
        McpServer.blueprint(),
        ManipulationModule.blueprint(
            model=model,
            # The native RRT rejects a recorded collision-free single-arm recovery
            # start on this planar model. The SDK's shared planner preserves the
            # full measured state and validates the same path successfully.
            planner=RRTConnectPlannerConfig(),
            joint_state_aliases=dict(zip(base_joints, R1PRO_PLANAR_BASE.joint_names, strict=True)),
            trajectory_tasks={
                "joint_trajectory": list(R1PRO_UPPER_BODY_PLANNING_JOINTS),
                PRIMITIVE_BASE_TASK: list(R1PRO_PLANAR_BASE.joint_names),
            },
            visualization={"backend": "none"},
        ),
    ]
    for (primitive, arm), cls in POLICIES.items():
        modules.append(
            cls.blueprint(
                artifact=str(policies / f"{primitive}-{arm}"),
                task="Pick the selected object and hold it."
                if primitive == "pick"
                else "Place the held object at the selected supported goal.",
                device="cuda",
                trajectory_task_name=f"primitive_{arm}",
                startup_timeout=180,
                max_execution_horizon_s=1.5,
                extra_env={
                    "OPENBLAS_NUM_THREADS": "1",
                    "OMP_NUM_THREADS": "4",
                    "MKL_NUM_THREADS": "4",
                },
            )
        )
    if agent:
        modules.append(McpClient.blueprint(system_prompt=system_prompt or PRIMITIVE_PROMPT))
    return (
        autoconnect(*modules)
        .remappings(
            [
                (simulator, "base_cmd_vel", f"/{PRIMITIVE_BASE_ID}/cmd_vel"),
                (simulator, "base_odom", f"/{PRIMITIVE_BASE_ID}/odom"),
                *(
                    (skills, f"_{primitive}_{arm}", cls)
                    for (primitive, arm), cls in POLICIES.items()
                ),
            ]
        )
        .global_config(transport="zenoh", viewer="none", simulation="mujoco", n_workers=6)
    )


PRIMITIVE_PROMPT = """You control a simulated R1Pro through independent ACT primitives. Start idle.
Read get_scene to resolve exact IDs, shapes, positions, support regions and each hand's held object.
Rightmost is minimum left_m; furthest is maximum distance_m. Tray contents are valid pick sources.
Preserve the requested hand and object. If ambiguous, ask; never substitute another target.
pick_object grasps, lifts and HOLDS. Stop after it completes unless placement was explicitly requested.
place_object operates only on an already held object, at an explicit available support region.
For "put X in Y", compose pick then place, waiting after each. For "pick X", never infer a destination.
The hands can hold different objects. Execute requested actions sequentially; do not reset the other hand.
Use wait_for_action until terminal completion. Accepted/running is not success. On failure stop the
sequence and report the measured error. Do not retry or reset implicitly. recover_action preserves a
confirmed hold or restores only the empty failed arm through the SDK. It never retries a grasp.
stop_action holds position;
reset_scene discards progress and requires an explicit user reset request. Regions currently cover
the physical table and tray. A full region refuses placement without rearranging. Do not claim unseen
objects, unvalidated support heights, or room navigation. Grasp and place are ACT; base prepositioning
is classical SDK execution. Keep replies brief."""

# The registry scanner recognizes terminal blueprint methods on factory results.
r1pro_primitives_sim = build_primitive_blueprint(
    RECORDINGS_DIR / "r1pro-act-task/policy-primitives-preview"
).global_config()
r1pro_primitives_sim_agent = build_primitive_blueprint(
    RECORDINGS_DIR / "r1pro-act-task/policy-primitives-preview", agent=True
).global_config()
