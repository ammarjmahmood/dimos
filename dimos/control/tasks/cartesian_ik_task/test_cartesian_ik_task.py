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

"""Behavior tests for the absolute Cartesian Pink task leaf."""

from pathlib import Path

import attrs
import pytest
from pytest_mock import MockerFixture

from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.control.tasks.cartesian_ik_task.cartesian_ik_task import (
    CartesianIKTask,
    CartesianIKTaskConfig,
)
from dimos.control.tasks.pose_target_ik import PinkPoseTargetSolver
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.assets.model import RobotModel


def _config(target_frames: tuple[str, ...] = ("tool",)) -> CartesianIKTaskConfig:
    return CartesianIKTaskConfig(
        joint_names=("arm/joint",),
        robot_model=RobotModelConfig(
            model=RobotModel.from_file(Path("fake.urdf")),
            joint_names=["arm/joint"],
        ),
        target_frames=target_frames,
    )


@pytest.mark.parametrize("target_frames", [(), ("tool", "other")])
def test_cartesian_config_requires_exactly_one_target_frame(
    target_frames: tuple[str, ...],
) -> None:
    with pytest.raises(ValueError):
        _config(target_frames)


def test_cartesian_leaf_maps_absolute_pose_to_configured_frame(
    mocker: MockerFixture,
) -> None:
    solver = mocker.Mock(spec=PinkPoseTargetSolver)
    solver.step.return_value = JointState(name=["arm/joint"], position=[0.01])
    task = CartesianIKTask("cartesian", _config(), solver=solver)
    target = PoseStamped(position=Vector3(0.4, 0.2, 0.1), frame_id="world")

    task.on_cartesian_command(target, t_now=2.0)
    output = task.compute(
        CoordinatorState(
            joints=JointStateSnapshot(joint_positions={"arm/joint": 0.0}),
            t_now=2.0,
            dt=0.01,
        )
    )

    assert output is not None
    assert solver.step.call_args.args[0] == {"tool": target}


def test_cartesian_leaf_clears_after_timeout(mocker: MockerFixture) -> None:
    solver = mocker.Mock(spec=PinkPoseTargetSolver)
    task = CartesianIKTask("cartesian", _config(), solver=solver)
    task.on_cartesian_command(PoseStamped(), t_now=1.0)

    output = task.compute(
        CoordinatorState(
            joints=JointStateSnapshot(joint_positions={"arm/joint": 0.0}),
            t_now=2.0,
            dt=0.01,
        )
    )

    assert output is None
    assert not task.is_active()
    solver.step.assert_not_called()


def test_cartesian_clear_reseeds_command_from_feedback(mocker: MockerFixture) -> None:
    solver = mocker.Mock(spec=PinkPoseTargetSolver)
    solver.step.return_value = JointState(name=["arm/joint"], position=[0.1])
    task = CartesianIKTask("cartesian", _config(), solver=solver)
    state = CoordinatorState(
        joints=JointStateSnapshot(joint_positions={"arm/joint": 0.0}),
        t_now=1.0,
        dt=0.01,
    )
    task.on_cartesian_command(PoseStamped(), t_now=1.0)
    assert task.compute(state) is not None

    task.clear()
    task.on_cartesian_command(PoseStamped(), t_now=1.1)
    solver.step.return_value = JointState(name=["arm/joint"], position=[0.01])
    assert task.compute(state) is not None

    assert solver.reset.call_count == 1


def test_idle_feedback_uses_existing_solver_and_snapshot_timestamp(mocker: MockerFixture) -> None:
    solver = mocker.Mock(spec=PinkPoseTargetSolver)
    pose = PoseStamped(position=Vector3(0.4, 0.2, 0.1))
    solver.frame_poses.return_value = {"tool": pose}
    task = CartesianIKTask("cartesian", _config(), solver=solver)
    state = CoordinatorState(
        joints=JointStateSnapshot(
            joint_positions={"arm/joint": 0.2},
            timestamp=42.0,
        )
    )
    feedback = task.get_feedback(state)
    assert feedback["t"] == 42.0
    assert feedback["ee_pose"] is pose
    assert feedback["positions"] == {"arm/joint": 0.2}
    assert feedback["tracking"] is False
    measured, frames = solver.frame_poses.call_args.args
    assert measured.position == [0.2] and frames == ("tool",)
    solver.step.assert_not_called()


def test_cancel_acknowledges_without_measured_state(mocker: MockerFixture) -> None:
    solver = mocker.Mock(spec=PinkPoseTargetSolver)
    task = CartesianIKTask("cartesian", _config(), solver=solver)
    task.on_cartesian_command(PoseStamped(), t_now=1.0)
    assert task.cancel()
    assert not task.is_tracking()
    solver.reset.assert_called_once()
    solver.frame_poses.assert_not_called()


def test_feedback_correction_stays_inside_task_and_preserves_requested_target(
    mocker: MockerFixture,
) -> None:
    solver = mocker.Mock(spec=PinkPoseTargetSolver)
    solver.step.return_value = JointState(name=["arm/joint"], position=[0.1])
    solver.frame_poses.side_effect = lambda joints, frames: {
        "tool": PoseStamped(position=Vector3(0, 0, 0.4 + joints.position[0]))
    }
    task = CartesianIKTask(
        "cartesian", attrs.evolve(_config(), feedback_correction=True), solver=solver
    )
    target = PoseStamped(position=Vector3(0, 0, 0.6))
    state = CoordinatorState(
        joints=JointStateSnapshot(joint_positions={"arm/joint": 0}), t_now=1, dt=0.01
    )
    task.on_cartesian_command(target, t_now=1)
    task.compute(state)
    task.compute(state)
    assert solver.step.call_args.args[0]["tool"].z == pytest.approx(0.7)
    assert target.z == pytest.approx(0.6)
    task.cancel()
    task.on_cartesian_command(target, t_now=1.1)
    task.compute(state)
    assert solver.step.call_args.args[0]["tool"].z == pytest.approx(0.6)
