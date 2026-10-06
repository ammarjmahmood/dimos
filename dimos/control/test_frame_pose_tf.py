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

"""The coordinator publishes tasks' measured frame poses on tf."""

from __future__ import annotations

import threading
from unittest.mock import MagicMock

import pytest

from dimos.control.components import HardwareComponent, HardwareType, make_joints
from dimos.control.coordinator import ControlCoordinator
from dimos.control.hardware_interface import ConnectedHardware
from dimos.control.task import BaseControlTask, CoordinatorState, ResourceClaim
from dimos.control.tick_loop import TickLoop
from dimos.hardware.manipulators.spec import ManipulatorAdapter
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.manipulators.common.coordinators import ArmTwistCoordinator

JOINTS = make_joints("arm", 2)


class _PoseTask(BaseControlTask):
    """Reports a frame whose x is the first measured joint, like FK would."""

    def __init__(self, name: str, frame: str, fail: bool = False) -> None:
        self._name = name
        self._frame = frame
        self._fail = fail

    def claim(self) -> ResourceClaim:
        return ResourceClaim(joints=frozenset())

    def is_active(self) -> bool:
        return False

    def compute(self, state: CoordinatorState) -> None:
        return None

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        pass

    def measured_frame_poses(self, state: CoordinatorState) -> dict[str, PoseStamped]:
        if self._fail:
            raise RuntimeError("no model")
        x = state.joints.get_position(JOINTS[0])
        return {self._frame: PoseStamped(frame_id="link_base", position=(x, 0.0, 0.5))}


def _tick_loop(tasks, publish_tf_callback, positions=(0.25, 0.5)) -> TickLoop:  # type: ignore[no-untyped-def]
    adapter = MagicMock(spec=ManipulatorAdapter)
    adapter.read_joint_positions.return_value = list(positions)
    adapter.read_joint_velocities.return_value = [0.0, 0.0]
    adapter.read_joint_efforts.return_value = [0.0, 0.0]
    hardware = ConnectedHardware(
        adapter,
        HardwareComponent(hardware_id="arm", hardware_type=HardwareType.MANIPULATOR, joints=JOINTS),
    )
    return TickLoop(
        tick_rate=100.0,
        hardware={"arm": hardware},
        hardware_lock=threading.Lock(),
        tasks={task.name: task for task in tasks},
        task_lock=threading.Lock(),
        joint_to_hardware={},
        publish_callback=MagicMock(),
        publish_tf_callback=publish_tf_callback,
    )


def test_measured_frame_poses_are_published_as_world_tf() -> None:
    published: list[TFMessage] = []
    loop = _tick_loop([_PoseTask("ik", "link_tcp")], published.append, positions=(0.25, 0.5))
    loop._tick()

    (message,) = published
    (transform,) = message.transforms
    assert (transform.frame_id, transform.child_frame_id) == ("world", "link_tcp")
    assert transform.translation.to_numpy().tolist() == [0.25, 0.0, 0.5]
    assert transform.ts == loop._publish_callback.call_args.args[0].ts  # joint-state time


def test_frame_poses_are_rate_limited() -> None:
    published: list[TFMessage] = []
    loop = _tick_loop([_PoseTask("ik", "link_tcp")], published.append)
    for _ in range(5):
        loop._tick()  # all within one 30 Hz period
    assert len(published) == 1


def test_a_failing_task_does_not_hide_the_others() -> None:
    published: list[TFMessage] = []
    tasks = [_PoseTask("broken", "a", fail=True), _PoseTask("left", "left/link_tcp")]
    _tick_loop(tasks, published.append)._tick()
    assert [t.child_frame_id for t in published[0].transforms] == ["left/link_tcp"]


def test_nothing_is_computed_without_a_tf_callback() -> None:
    task = _PoseTask("ik", "link_tcp")
    task.measured_frame_poses = MagicMock()  # type: ignore[method-assign]
    _tick_loop([task], None)._tick()
    task.measured_frame_poses.assert_not_called()


def test_tasks_without_a_model_publish_nothing() -> None:
    published: list[TFMessage] = []

    class _Plain(_PoseTask):
        measured_frame_poses = BaseControlTask.measured_frame_poses

    _tick_loop([_Plain("joint", "unused")], published.append)._tick()
    assert published == []


def test_only_coordinators_declaring_tf_can_publish_frame_poses() -> None:
    plain = ControlCoordinator(publish_frame_poses=True)
    arm = ArmTwistCoordinator(publish_frame_poses=True)
    try:
        with pytest.raises(ValueError, match="add `tf: Out\\[TFMessage\\]`"):
            plain._frame_pose_port()
        assert arm._frame_pose_port() is arm.tf
        assert "tf" not in plain.outputs
    finally:
        plain.stop()
        arm.stop()
