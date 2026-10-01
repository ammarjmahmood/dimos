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

"""Hermetic checks for development-only collision admission and stale scenes."""

import copy

import pytest

from dimos.control.tasks.trajectory_task.trajectory_task import JointTrajectoryTask
from dimos.manipulation.manipulation_spec import (
    ExecutionResult,
    ExecutionStatus,
    ManipulationSpec,
    PlanResult,
    PlanStatus,
)
from dimos.manipulation.planning.spec.enums import PlanningStatus
from dimos.manipulation.planning.spec.models import GeneratedPlan
from dimos.manipulation.planning.spec.protocols import WorldSpec
from dimos.manipulation.sdk import Arm, MotionError
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.simulation.behavior.radio_motion import (
    DevelopmentRadioMotion,
    RadioCoordinator,
    anchored_trajectory,
    assert_scene_unchanged,
    trajectory_digest,
    validate_development_trajectory,
)


@pytest.fixture
def trajectory():
    return JointTrajectory(
        joint_names=["arm", "torso"],
        points=[TrajectoryPoint(0.0, [0.0, 0.0]), TrajectoryPoint(1.0, [0.2, 0.1])],
    )


def test_validation_checks_effective_anchor_and_each_interpolated_edge(mocker, trajectory):
    world = mocker.Mock(spec=WorldSpec)
    world.check_config_collision_free.return_value = True
    world.check_edge_collision_free.return_value = True
    state = JointState(name=["base", "arm", "torso"], position=[3.0, 0.0, 0.0])
    effective = anchored_trajectory(trajectory, {"arm": 0.01, "torso": 0.02})
    result = validate_development_trajectory(world, state, effective, ["arm", "torso"])
    assert result["effective_digest"] == trajectory_digest(effective)
    assert trajectory.points[0].positions == [0.0, 0.0]
    assert world.check_edge_collision_free.call_count == 2
    world.check_edge_collision_free.assert_called_with(mocker.ANY, mocker.ANY, step_size=0.01)
    states = [call.args[0] for call in world.check_config_collision_free.call_args_list]
    assert [q.position for q in states] == [[3.0, 0.01, 0.02], [3.0, 0.2, 0.1]]


def test_inside_radio_path_rejected_before_authorization(mocker, trajectory):
    world = mocker.Mock(spec=WorldSpec)
    world.check_config_collision_free.return_value = False
    with pytest.raises(RuntimeError, match="intersects"):
        validate_development_trajectory(
            world,
            JointState(name=["arm", "torso"], position=[0.0, 0.0]),
            trajectory,
            ["arm", "torso"],
        )


def test_colliding_intermediate_edge_rejected_even_when_endpoints_clear(mocker, trajectory):
    world = mocker.Mock(spec=WorldSpec)
    world.check_config_collision_free.return_value = True
    world.check_edge_collision_free.side_effect = [True, False]
    with pytest.raises(RuntimeError, match="intersects"):
        validate_development_trajectory(
            world,
            JointState(name=["arm", "torso"], position=[0.0, 0.0]),
            trajectory,
            ["arm", "torso"],
        )


def test_unselected_base_in_trajectory_rejected(mocker, trajectory):
    with pytest.raises(ValueError, match="selected joints"):
        validate_development_trajectory(
            mocker.Mock(spec=WorldSpec), JointState(), trajectory, ["arm"]
        )


def test_digest_detects_effective_anchor_and_mutated_path(trajectory):
    original = trajectory_digest(trajectory)
    effective = anchored_trajectory(trajectory, {"arm": 0.01, "torso": 0.0})
    assert trajectory_digest(effective) != original
    trajectory.points[-1].positions[0] += 0.01
    assert trajectory_digest(trajectory) != original


@pytest.mark.parametrize("change", ["position", "orientation", "scale", "name", "model"])
def test_scene_move_or_replacement_rejects_old_validation(change):
    obj = {
        "name": "radio",
        "model": "wxnicr",
        "scale": [1.0, 1.0, 1.0],
        "position": [0.0, 0.0, 0.0],
        "orientation": [0.0, 0.0, 0.0, 1.0],
    }
    current = copy.deepcopy(obj)
    current[change] = {
        "position": [0.0, 0.003, 0.0],
        "orientation": [0.0, 0.0, 0.1, 0.995],
        "scale": [0.5, 0.5, 0.5],
        "name": "replacement",
        "model": "other",
    }[change]
    with pytest.raises(RuntimeError, match="scene"):
        assert_scene_unchanged({"radio": obj}, {"radio": current})


def test_quaternion_sign_change_does_not_reject_identical_scene():
    obj = {"name": "radio", "position": [0.0, 0.0, 0.0], "orientation": [0.0, 0.0, 0.0, 1.0]}
    assert_scene_unchanged({"radio": obj}, {"radio": {**obj, "orientation": [0.0, 0.0, 0.0, -1.0]}})


@pytest.fixture
def coordinator(mocker):
    module = RadioCoordinator()
    task = mocker.Mock(spec=JointTrajectoryTask)
    task._commanded_positions = {"arm": 0.01, "torso": 0.02}
    module._trajectory_task = task
    mocker.patch.object(module, "get_joint_positions", return_value={"arm": 0.01, "torso": 0.02})
    try:
        yield module, task
    finally:
        module.stop()


def test_dispatch_accepts_only_the_validated_effective_trajectory(coordinator, trajectory):
    module, task = coordinator
    effective = module.prepare_development_trajectory(trajectory)
    module.authorize_development_trajectory(
        trajectory_digest(trajectory), trajectory_digest(effective)
    )
    module.task_invoke("joint_trajectory", "execute", {"trajectory": trajectory})
    task.execute.assert_called_once_with(trajectory, {"arm": 0.01, "torso": 0.02})
    module.task_invoke("joint_trajectory", "execute", {"trajectory": trajectory})
    assert task.execute.call_count == 1  # authorization is consumed exactly once


@pytest.mark.parametrize("change", ["anchor", "path", "expired"])
def test_dispatch_rejects_changed_anchor_path_or_expired_ticket(
    coordinator, trajectory, mocker, change
):
    module, task = coordinator
    effective = module.prepare_development_trajectory(trajectory)
    module.authorize_development_trajectory(
        trajectory_digest(trajectory), trajectory_digest(effective)
    )
    if change == "anchor":
        task._commanded_positions["arm"] += 0.001
    elif change == "path":
        trajectory.points[-1].positions[0] += 0.001
    else:
        mocker.patch(
            "dimos.simulation.behavior.radio_motion.time.monotonic",
            return_value=module._radio_ticket[2] + 2.0,
        )
    result = module.task_invoke("joint_trajectory", "execute", {"trajectory": trajectory})
    assert result.status.name == "INVALID_TRAJECTORY"
    task.execute.assert_not_called()


@pytest.fixture
def checked_motion(mocker, trajectory):
    arm = mocker.Mock(spec=Arm)
    arm.info = mocker.Mock(id="left_arm")
    arm.rpc = mocker.Mock(spec=ManipulationSpec)
    plan = GeneratedPlan(
        group_ids=("left_arm", "torso"), trajectory=trajectory, status=PlanningStatus.SUCCESS
    )
    arm.rpc.plan_to_poses.return_value = PlanResult(PlanStatus.SUCCEEDED, plan=plan)
    arm.rpc.execute.return_value = ExecutionResult(ExecutionStatus.COMPLETED)
    coordinator = mocker.Mock(spec=RadioCoordinator)
    coordinator.prepare_development_trajectory.return_value = trajectory
    world = mocker.Mock(spec=WorldSpec)
    world.check_config_collision_free.return_value = True
    world.check_edge_collision_free.return_value = True
    state = JointState(name=["base", "arm", "torso"], position=[3.0, 0.0, 0.0])
    state_reader = mocker.Mock(return_value=state)
    obj = {"name": "radio", "position": [0.0, 0.0, 0.0], "orientation": [0.0, 0.0, 0.0, 1.0]}
    scene = mocker.Mock(return_value={"radio": obj})
    motion = DevelopmentRadioMotion(
        arm,
        coordinator,
        mocker.Mock(),
        world,
        state_reader,
        scene,
        {"radio": obj},
        ["arm", "torso"],
        mocker.Mock(),
        [],
        ("torso",),
    )
    return motion, arm, scene, state_reader, plan


def test_same_plan_id_is_executed_after_validation(checked_motion):
    motion, arm, _, _, plan = checked_motion
    motion.move([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 10.0, False)
    assert arm.rpc.execute.call_args.kwargs["plan_id"] == plan.plan_id
    assert motion.evidence[0]["plan_id"] == plan.plan_id
    arm.rpc.plan_to_poses.assert_called_once()


def test_scene_changes_after_check_prevent_authorization_and_execution(checked_motion):
    motion, arm, scene, _, _ = checked_motion
    scene.side_effect = [
        motion.reference,
        {"radio": {**motion.reference["radio"], "position": [0.0, 0.003, 0.0]}},
    ]
    with pytest.raises(RuntimeError, match="stale"):
        motion.move([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 10.0, False)
    motion.coordinator.authorize_development_trajectory.assert_not_called()
    arm.rpc.execute.assert_not_called()


def test_pending_plan_replacement_is_reported_without_replanning(checked_motion):
    motion, arm, _, _, _ = checked_motion
    arm.rpc.execute.return_value = ExecutionResult(
        ExecutionStatus.REJECTED, "Pending plan was replaced"
    )
    with pytest.raises(MotionError, match="replaced"):
        motion.move([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 10.0, False)
    arm.rpc.plan_to_poses.assert_called_once()
    arm.rpc.execute.assert_called_once()


def test_start_state_drift_prevents_execution(checked_motion):
    motion, arm, _, state_reader, _ = checked_motion
    state_reader.side_effect = [
        JointState(name=["base", "arm", "torso"], position=[3.0, 0.0, 0.0]),
        JointState(name=["base", "arm", "torso"], position=[3.0, 0.003, 0.0]),
    ]
    with pytest.raises(RuntimeError, match="start changed"):
        motion.move([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 10.0, False)
    arm.rpc.execute.assert_not_called()
