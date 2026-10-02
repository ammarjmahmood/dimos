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

import json
import time
from unittest.mock import MagicMock

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dimos.control.manipulation_control import ManipulationControl, delta_pose
from dimos.control.manipulation_types import DeltaTarget, JointTarget, StopCommand
from dimos.control.tasks.trajectory_task.trajectory_task import (
    TrajectoryExecutionResult,
    TrajectoryExecutionStatus,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.tf2_msgs.TFMessage import TFMessage


@pytest.fixture
def control(monkeypatch):
    module = ManipulationControl()
    module._joint_names = ("j1", "j2")
    module._limits = [(-1, 1), (-2, 2)]
    module._base = np.eye(4)
    module._base[:3, 3] = [0.1, 0.2, 0.3]
    module._base_inv = np.linalg.inv(module._base)
    module._solver = MagicMock()
    module._solver.frame_poses.return_value = {
        "link_tcp": PoseStamped(position=(0.4, 0.2, 0.6)),
        "link7": PoseStamped(position=(0.4, 0.2, 0.7)),
    }
    module.coordinator = MagicMock()
    module.coordinator.task_invoke.return_value = TrajectoryExecutionResult(
        TrajectoryExecutionStatus.ACCEPTED
    )
    published = []
    monkeypatch.setattr(
        module, "_emit", lambda kind, value, ts=None: published.append((kind, value))
    )
    module._on_state(
        JointState(ts=time.time(), name=["j1", "j2", "arm/gripper"], position=[0.1, 0.2, 0.85])
    )
    yield module, published
    module.stop()


def test_delta_uses_fixed_base_axes_and_left_composed_rotation():
    current = np.eye(4)
    current[:3, 3] = [1, 2, 3]
    current[:3, :3] = Rotation.from_euler("y", 0.7).as_matrix()
    target = delta_pose(current, (0, 0, 0.05), (0.2, 0, 0))
    np.testing.assert_allclose(target[:3, 3], [1, 2, 3.05])
    np.testing.assert_allclose(
        target[:3, :3], (Rotation.from_euler("x", 0.2) * Rotation.from_euler("y", 0.7)).as_matrix()
    )
    np.testing.assert_allclose(current[:3, 3], [1, 2, 3])


def test_duplicate_delta_does_not_rebase_on_new_feedback(control):
    module, published = control
    module._base[:3, :3] = Rotation.from_euler("z", np.pi / 2).as_matrix()
    module._base_inv = np.linalg.inv(module._base)
    current = module._base.copy()
    command = DeltaTarget(id="a", kind="delta", xyz=(0.05, 0, 0))
    module._accept(command, module._state, current, 0)
    expected = module._active.target.copy()
    np.testing.assert_allclose(expected[:3, 3], [0.1, 0.25, 0.3])
    moved = current.copy()
    moved[0, 3] += 1
    module._accept(command, module._state, moved, 1)
    np.testing.assert_allclose(module._active.target, expected)
    module._solver.reset.assert_called_once()
    module._accept(DeltaTarget(id="a", kind="delta", xyz=(0.1, 0, 0)), module._state, moved, 2)
    assert published[-1][1]["reason"] == "id_conflict"


def test_joint_commands_validate_limits_and_use_measured_completion(control):
    module, _ = control
    module._accept(
        JointTarget(id="a", kind="joints", positions=[0.4, 0.5]), module._state, np.eye(4), 0
    )
    module._advance(module._state, np.eye(4), 1)
    assert module._results["a"]["status"] == "running"
    measured = JointState(name=["j1", "j2", "arm/gripper"], position=[0.4, 0.5, 0.85])
    module._advance(measured, np.eye(4), 2)
    module._advance(measured, np.eye(4), 2.3)
    assert module._results["a"]["status"] == "succeeded"
    with pytest.raises(ValueError, match="outside"):
        module._accept(
            JointTarget(id="bad", kind="joints", positions=[2, 0]), measured, np.eye(4), 3
        )


@pytest.mark.parametrize("feedback", ["fresh", "stale", "missing", "invalid"])
def test_stop_bypasses_full_motion_queue_and_fk(control, feedback):
    module, _ = control
    module._accept(
        JointTarget(id="active", kind="joints", positions=[0.4, 0.5]), module._state, np.eye(4), 0
    )
    module._accept(DeltaTarget(id="busy", kind="delta"), module._state, np.eye(4), 1)
    assert module._results["busy"]["status"] == "rejected"
    for i in range(32):
        module._on_command(DeltaTarget(id=f"queued-{i}", kind="delta"))
    assert module._queue.full()
    if feedback == "stale":
        module._received = time.monotonic() - 10
    elif feedback == "missing":
        module._state = None
    elif feedback == "invalid":
        module._state = JointState(name=["j1"], position=[float("nan")])
    module._solver.frame_poses.side_effect = AssertionError("stop must not depend on FK")
    module._on_command(StopCommand(id="stop", kind="stop"))
    module._tick()
    assert module._active is None and module._queue.empty()
    assert module._results["active"]["status"] == "cancelled"
    assert module._results["stop"]["status"] == "succeeded"
    assert all(module._results[f"queued-{i}"]["status"] == "rejected" for i in range(32))
    module._on_command(StopCommand(id="stop", kind="stop"))
    module._tick()
    assert [call.args[1] for call in module.coordinator.task_invoke.call_args_list] == [
        "execute",
        "cancel",
    ]


def test_stop_failure_has_a_cached_error_result(control):
    module, _ = control
    module.coordinator.task_invoke.side_effect = ConnectionError("stop not acknowledged")
    stop = StopCommand(id="stop", kind="stop")
    module._on_command(stop)
    with pytest.raises(ConnectionError):
        module._tick()
    assert module._results["stop"]["status"] == "error"
    module._on_command(stop)
    module._tick()
    assert module.coordinator.task_invoke.call_count == 1


def test_timeout_cancels_without_claiming_success(control):
    module, _ = control
    module._accept(
        JointTarget(id="a", kind="joints", positions=[0.4, 0.5], timeout_s=1),
        module._state,
        np.eye(4),
        0,
    )
    module._advance(module._state, np.eye(4), 1.1)
    assert module._results["a"]["status"] == "timed_out"
    assert module._active is None


def test_stale_feedback_stops_and_never_replays_queued_motion(control):
    module, _ = control
    module._accept(
        JointTarget(id="a", kind="joints", positions=[0.4, 0.5]), module._state, np.eye(4), 0
    )
    pending = DeltaTarget(id="queued", kind="delta", xyz=(0, 0, 0.05))
    module._on_command(pending)
    module._received = time.monotonic() - 10
    module._tick()
    assert module._results["a"]["status"] == "error"
    assert module._results["queued"]["status"] == "rejected"
    module._on_state(
        JointState(ts=time.time(), name=["j1", "j2", "arm/gripper"], position=[0.1, 0.2, 0.85])
    )
    module._on_command(pending)
    module._tick()
    assert module._active is None


def test_delta_limit_rejected_before_ik(control):
    module, _ = control
    module._on_command(DeltaTarget(id="far", kind="delta", xyz=(0, 0, 0.5)))
    module._tick()
    assert module._results["far"]["status"] == "rejected"
    module._solver.step.assert_not_called()


def test_full_motion_queue_returns_rejection(control):
    module, published = control
    for i in range(32):
        module._on_command(DeltaTarget(id=str(i), kind="delta"))
    module._on_command(DeltaTarget(id="overflow", kind="delta"))
    assert published[-1][1]["id"] == "overflow"
    assert "queue full" in published[-1][1]["reason"]


def test_rpc_failure_cancels_and_caches_status(control):
    module, _ = control
    module.coordinator.task_invoke.side_effect = [ConnectionError("reply lost"), object()]
    command = JointTarget(id="a", kind="joints", positions=[0.3, 0.4])
    module._on_command(command)
    module._tick()
    assert module._results["a"]["status"] == "error"
    assert [call.args[1] for call in module.coordinator.task_invoke.call_args_list] == [
        "execute",
        "cancel",
    ]
    module._on_command(command)
    module._tick()
    assert module.coordinator.task_invoke.call_count == 2


def test_camera_poses_are_independent_and_exclude_object_truth(control):
    module, published = control
    ts = time.time()
    module._on_tf(
        TFMessage(
            Transform(
                translation=Vector3(0.4, 0.2, 0.7),
                frame_id="world",
                child_frame_id="wrist_camera_color_optical_frame",
                ts=ts,
            )
        )
    )
    module._on_tf(
        TFMessage(
            Transform(
                translation=Vector3(1.7, -0.9, 1.8),
                frame_id="world",
                child_frame_id="env_camera_color_optical_frame",
                ts=ts + 0.01,
            ),
            Transform(translation=Vector3(9, 9, 9), frame_id="world", child_frame_id="cup", ts=ts),
        )
    )
    module._tick()
    results = dict(published)
    assert results["camera_pose"]["t"] == ts
    assert results["overview_camera_pose"]["t"] == ts + 0.01
    np.testing.assert_allclose(results["state"]["ee_pose"]["xyz"], [0.3, 0, 0.3])
    np.testing.assert_allclose(results["camera_pose"]["xyz"], [0.3, 0, 0.4])
    np.testing.assert_allclose(results["overview_camera_pose"]["xyz"], [1.6, -1.1, 1.5])
    assert "cup" not in json.dumps(results)


def test_info_and_status_keepalive_are_not_sent_at_control_rate(control, monkeypatch):
    module, published = control
    clock = [time.monotonic()]
    monkeypatch.setattr("dimos.control.manipulation_control.time.monotonic", lambda: clock[0])
    module._status(StopCommand(id="done", kind="stop"), "succeeded")
    module._tick()
    clock[0] += 0.1
    module._tick()
    assert sum(kind == "info" for kind, _ in published) == 1
    assert sum(kind == "status" for kind, _ in published) == 1
    clock[0] += 1
    module._on_state(
        JointState(ts=time.time(), name=["j1", "j2", "arm/gripper"], position=[0.1, 0.2, 0.85])
    )
    module._tick()
    assert sum(kind == "info" for kind, _ in published) == 2
    assert sum(kind == "status" for kind, _ in published) == 2
