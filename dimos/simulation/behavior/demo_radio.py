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

"""Development-only stationary SDK motion and contact interaction; no MCP required."""

import argparse
from collections.abc import Callable, Sequence
import json
import math
from pathlib import Path
import threading
import time
from typing import Any

from dimos.control.components import HardwareComponent, HardwareType
from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.transport import ZenohTransport
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.planning.planners.roboplan_config import RoboPlanPlannerConfig
from dimos.manipulation.sdk import Arm
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.porcelain.dimos import Dimos
from dimos.protocol.pubsub.impl.zenohpubsub import Topic as ZenohTopic
from dimos.robot.galaxea.r1pro.joints import UPPER_BODY_JOINTS, coordinator_name
from dimos.simulation.behavior.connection import BehaviorConnection
from dimos.simulation.behavior.probe import BehaviorProbe
from dimos.simulation.behavior.r1pro_bridge import BehaviorCoordinator, BehaviorR1ProBridge
from dimos.simulation.behavior.r1pro_model import MODEL_JOINTS, simulation_model_config
from dimos.simulation.behavior.types import ControlMode, TaskSelection


def radio_blueprint(
    task: TaskSelection,
    *,
    arm: str = "left_arm",
    spawn_position: tuple[float, float, float] | None = None,
    spawn_yaw: float = 0.0,
) -> Blueprint:
    if arm not in ("left_arm", "right_arm"):
        raise ValueError("Choose left_arm or right_arm")
    model = simulation_model_config()
    model.planning_groups = [g for g in model.planning_groups if g.name in (arm, "torso")]
    # One selected arm per development instance; the scalar SDK gripper binding
    # refers only to this arm, never to both physical grippers.
    model.gripper_hardware_id = "r1pro"
    side = arm.split("_", 1)[0]
    # The simulator's coupled gripper controller consumes the first finger's
    # target and physically drives both fingers. The SDK sends one scalar.
    gripper = [coordinator_name(f"{side}_gripper_finger_joint1")]
    upper = [coordinator_name(n) for n in UPPER_BODY_JOINTS]
    return (
        autoconnect(
            BehaviorConnection.blueprint(
                task=task,
                allow_task_changes=False,
                spawn_position=spawn_position,
                spawn_yaw=spawn_yaw,
                development_task_spawn=spawn_position is not None,
                # Development control needs steady feedback, rather than first-use
                # JIT compilation blocking the single simulator thread.
                extra_env={"TORCH_COMPILE_DISABLE": "1"},
            ),
            BehaviorProbe.blueprint(),
            BehaviorR1ProBridge.blueprint(command_joints=MODEL_JOINTS),
            BehaviorCoordinator.blueprint(
                instance_name="ControlCoordinator",
                tick_rate=30,
                hardware=[
                    HardwareComponent(
                        hardware_id="r1pro",
                        hardware_type=HardwareType.WHOLE_BODY,
                        joints=[coordinator_name(n) for n in MODEL_JOINTS],
                        adapter_type="transport_lcm",
                    )
                ],
                tasks=[
                    joint_trajectory_task(upper),
                    TaskConfig(
                        name="r1pro_gripper",
                        type="gripper",
                        joint_names=gripper,
                        priority=20,
                    ),
                ],
            ).remappings([(BehaviorCoordinator, "joint_command", "coordinator_joint_command")]),
            ManipulationModule.blueprint(
                model=model,
                planner=RoboPlanPlannerConfig(),
                planning_timeout=10.0,
                trajectory_tasks={"joint_trajectory": upper},
            ).remappings(
                [
                    (ManipulationModule, "coordinator_joint_state", "planning_joint_state"),
                    (ManipulationModule, "tf", "planning_tf"),
                ]
            ),
        )
        .transports(
            {
                ("motor_states", JointState): ZenohTransport.spec(
                    ZenohTopic("dimos/r1pro/motor_states", JointState)
                ),
                ("motor_command", MotorCommandArray): ZenohTransport.spec(
                    ZenohTopic("dimos/r1pro/motor_command", MotorCommandArray)
                ),
            }
        )
        .global_config(transport="zenoh", n_workers=4)
    )


def wait_for_measured_pose(
    arm: Arm, position: Sequence[float], timeout: float, tolerance: float = 0.005
) -> list[float]:
    """Require fresh encoder-derived FK after the trajectory clock completes."""
    deadline = time.monotonic() + timeout
    while True:
        state = arm.state()
        pose = state.end_effector_pose
        if state.joints is not None and pose is not None:
            measured = list(pose.position.to_tuple())
            if math.dist(measured, position) <= tolerance:
                return measured
        if time.monotonic() >= deadline:
            raise TimeoutError("Trajectory returned but fresh measured target pose was not reached")
        time.sleep(0.02)


def press_toggle(
    arm: Arm,
    contact_position: Sequence[float],
    approach_direction: Sequence[float],
    simulation_step: Callable[[], int],
    *,
    orientation: Sequence[float] | None = None,
    auxiliary_groups: Sequence[str] = (),
    approach_distance: float = 0.03,
    hold_steps: int = 8,
    timeout: float = 30.0,
) -> dict[str, Any]:
    """Approach, physically hold contact, and retract using pose-level SDK calls.

    The supplied target is the gripper-link pose, not the finger-tip pose. Target
    provenance and contact geometry must be established by the caller. This
    performs no symbolic state mutation and makes no claim of evaluator success.
    """
    if len(contact_position) != 3 or len(approach_direction) != 3:
        raise ValueError("Target and approach require three coordinates")
    if not all(math.isfinite(v) for v in (*contact_position, *approach_direction)):
        raise ValueError("Target and approach must be finite")
    norm = math.sqrt(sum(v * v for v in approach_direction))
    if norm == 0 or not 0 < approach_distance <= 0.1 or hold_steps < 5:
        raise ValueError("Use a nonzero approach, distance <= 0.1 m and at least five hold steps")
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError("Use a finite positive timeout")
    direction = [v / norm for v in approach_direction]
    precontact = [
        p - approach_distance * v for p, v in zip(contact_position, direction, strict=True)
    ]
    deadline = time.monotonic() + timeout

    def remaining() -> float:
        value = deadline - time.monotonic()
        if value <= 0:
            raise TimeoutError("Physical interaction deadline elapsed")
        return value

    try:
        if auxiliary_groups:
            arm.move_pose(
                precontact,
                orientation=orientation,
                auxiliary_groups=auxiliary_groups,
                timeout=remaining(),
                speed_scale=0.15,
            )
        else:
            arm.move_pose(
                precontact, orientation=orientation, timeout=remaining(), speed_scale=0.15
            )
        measured_precontact = wait_for_measured_pose(arm, precontact, remaining())
        arm.move_linear(
            *(approach_distance * v for v in direction),
            auxiliary_groups=auxiliary_groups,
            check_collision=True,
            timeout=remaining(),
            speed_scale=0.1,
        )
        measured_contact = wait_for_measured_pose(arm, contact_position, remaining())
        first_step = simulation_step()
        while simulation_step() - first_step < hold_steps:
            time.sleep(min(0.02, remaining()))
        arm.move_linear(
            *(-approach_distance * v for v in direction),
            auxiliary_groups=auxiliary_groups,
            check_collision=True,
            timeout=remaining(),
            speed_scale=0.1,
        )
        measured_return = wait_for_measured_pose(arm, precontact, remaining())
        return {
            "development_only": True,
            "motion_completed": True,
            "hold_steps": hold_steps,
            "measured_precontact": measured_precontact,
            "measured_contact": measured_contact,
            "measured_return": measured_return,
        }
    except Exception as error:
        # Timeout of the calling Python process never proves remote motion stopped.
        try:
            result = arm.rpc.cancel()
        except Exception as cancellation_error:
            raise RuntimeError(
                f"Interaction failed ({error}); cancellation unconfirmed: {cancellation_error}"
            ) from error
        raise RuntimeError(
            f"Interaction failed ({error}); cancellation result: {result!r}"
        ) from error


def wait_operation(connection: Any, operation: str, timeout: float = 600.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        status = connection.get_status()
        if status.error:
            raise RuntimeError(status.error)
        result = connection.get_operation(operation)
        if result.state != "running":
            if result.state != "succeeded":
                raise RuntimeError(result.model_dump_json())
            return
        time.sleep(0.1)
    connection.cancel_operation(operation)
    raise TimeoutError(f"Operation {operation} timed out; cancellation requested")


def set_gripper_and_wait(arm: Arm, opening: float, timeout: float = 10.0) -> float:
    """Distinguish SDK command acceptance from measured gripper travel."""
    arm.set_gripper_position(opening)
    deadline = time.monotonic() + timeout
    while True:
        measured = arm.state().gripper_position
        if measured is not None and abs(measured - opening) < 0.1:
            return measured
        if time.monotonic() >= deadline:
            raise TimeoutError(
                f"Gripper command accepted but measured target {opening} not reached"
            )
        time.sleep(0.05)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", choices=["motion", "press", "serve"], default="motion")
    parser.add_argument("--scene", default="house_double_floor_lower")
    parser.add_argument("--instance", type=int, default=0)
    parser.add_argument("--arm", choices=["left_arm", "right_arm"], default="left_arm")
    parser.add_argument("--spawn-position", type=float, nargs=3)
    parser.add_argument("--spawn-yaw", type=float, default=0.0)
    parser.add_argument(
        "--auxiliary-torso",
        action="store_true",
        help="Allow torso joints to assist precontact IK; base remains frozen",
    )
    parser.add_argument(
        "--target", type=Path, help="Explicit gripper pose/approach JSON with provenance"
    )
    parser.add_argument(
        "--connect", action="store_true", help="Use an already running development coordinator"
    )
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument(
        "--development-diagnostics",
        action="store_true",
        help="Record privileged simulator geometry for development debugging only",
    )
    args = parser.parse_args()
    task = TaskSelection(scene=args.scene, activity="turning_on_radio", instance=args.instance)
    app = Dimos.connect() if args.connect else Dimos()
    report: dict[str, Any] = {
        "development_only": True,
        "task": task.model_dump(),
        "stage": args.stage,
        "torch_compilation": "disabled in development radio composition",
    }
    try:
        if not args.connect:
            app.run(
                radio_blueprint(
                    task,
                    arm=args.arm,
                    spawn_position=tuple(args.spawn_position) if args.spawn_position else None,
                    spawn_yaw=args.spawn_yaw,
                )
            )
        sim: Any = app.get_module("BehaviorConnection")
        wait_operation(sim, sim.take_control(ControlMode.DIMOS))
        arm = Arm.from_app(app, group=args.arm)
        report["capabilities"] = sim.describe()
        report["planning_collision_scope"] = (
            "Robot/self collision model; scene physics remains in simulator. Scene obstacles are not automatically registered in this development composition."
        )
        report["before"] = repr(arm.state())
        report["planning_groups_before"] = repr(arm.rpc.get_state())
        report["auxiliary_torso"] = args.auxiliary_torso
        if args.development_diagnostics:
            report["privileged_before"] = sim.get_ground_truth()
        if args.stage == "motion":
            report["gripper_closed"] = set_gripper_and_wait(arm, 0.0)
            report["gripper_open"] = set_gripper_and_wait(arm, 1.0)
            arm.move_linear(dx=-0.01, check_collision=True, timeout=30.0, speed_scale=0.1)
            report["outward"] = repr(arm.state())
            arm.move_linear(dx=0.01, check_collision=True, timeout=30.0, speed_scale=0.1)
            report["after"] = repr(arm.state())
        elif args.stage == "press":
            if args.target is None:
                raise ValueError("Press requires explicit --target and target provenance")
            target = json.loads(args.target.read_text())
            if not target.get("provenance"):
                raise ValueError("Target must label sensor-derived or oracle-assisted provenance")
            report["target"] = target
            report["gripper_closed"] = set_gripper_and_wait(arm, 0.0)
            report["interaction"] = press_toggle(
                arm,
                target["position"],
                target["approach"],
                lambda: sim.get_status().episode.step,
                orientation=target.get("orientation"),
                auxiliary_groups=("torso",) if args.auxiliary_torso else (),
            )
        else:
            report["ready"] = True
            args.report.write_text(json.dumps(report, indent=2))
            threading.Event().wait()
        # This is an evaluator result, separate from SDK motion completion.
        report["evaluator"] = sim.get_status().episode.model_dump()
        report["motion_completed"] = True
    except Exception as error:
        report["error"] = str(error)
        raise
    finally:
        if "sim" in locals():
            try:
                report["evaluator"] = sim.get_status().episode.model_dump()
                if args.development_diagnostics:
                    report["privileged_after"] = sim.get_ground_truth()
            except Exception as error:
                report["evaluator_error"] = str(error)
        args.report.write_text(json.dumps(report, indent=2))
        app.stop()


if __name__ == "__main__":
    main()
