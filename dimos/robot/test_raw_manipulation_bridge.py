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

import io
import json
import time
from unittest.mock import MagicMock

import numpy as np
from PIL import Image as PILImage
import pytest
from scipy.spatial.transform import Rotation

from dimos.control.tasks.trajectory_task.trajectory_task import (
    TrajectoryCancellationResult,
    TrajectoryCancellationStatus,
    TrajectoryExecutionResult,
    TrajectoryExecutionStatus,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.raw_manipulation_bridge import RawManipulationBridge, depth_f32
from dimos.utils.transform_utils import pose_to_matrix


@pytest.fixture
def bridge():
    module = RawManipulationBridge()
    module._topics = MagicMock()
    module.coordinator = MagicMock()
    feedback = {
        "t": time.time(),
        "positions": {"j1": 0.1, "j2": 0.2, "arm/gripper": 0.85},
        "velocities": {},
        "ee_pose": PoseStamped(position=(0.4, 0.2, 0.6)),
        "tracking": False,
    }
    targets = []

    def invoke(task, method, kwargs=None):
        if method == "get_control_info":
            return {
                "joint_names": ["j1", "j2"],
                "joint_limits": [(-1, 1), (-2, 2)],
                "base_frame": "base_link",
                "base_pose": PoseStamped(position=(0.1, 0.2, 0.3)),
                "ee_frame": "tool",
                "max_joint_velocity_rad_s": 0.5,
            }
        if method == "get_limits":
            return {"arm/gripper": (0.0, 0.85)}
        if method == "get_feedback":
            assert kwargs == {"state": None}
            return feedback
        if method == "cancel":
            feedback["tracking"] = False
            return True
        if method == "on_cartesian_command":
            targets.append(kwargs["pose"])
            feedback["tracking"] = True
            return True
        if method == "set_position":
            return True
        raise AssertionError(method)

    module.coordinator.task_invoke.side_effect = invoke
    module.coordinator.cancel_trajectory.return_value = TrajectoryCancellationResult(
        TrajectoryCancellationStatus.CANCELLED
    )
    module.coordinator.execute_trajectory.return_value = TrajectoryExecutionResult(
        TrajectoryExecutionStatus.ACCEPTED
    )
    module._load_info()
    yield module, feedback, targets
    module.stop()


def send(module, **command):
    module._on_command(json.dumps(command).encode(), None)


@pytest.mark.parametrize(
    "payload",
    [
        {"kind": "delta", "xyz": [0, 0, float("nan")]},
        {"kind": "delta", "rpy": [0, float("inf"), 0]},
        {"kind": "delta", "xyz": [0, 1]},
        {"kind": "delta", "xyz": [0, 0, 0.21]},
        {"kind": "delta", "rpy": [0, 0, 2]},
        {"kind": "delta", "frame": "tool"},
        {"kind": "joints", "positions": ["0.1", "0.2"]},
        {"kind": "joints", "positions": [True, 0]},
        {"kind": "joints", "positions": [0]},
        {"kind": "joints", "positions": [2, 0]},
        {"kind": "gripper", "position": 1.0},
        {"kind": "gripper", "opening": 0.5},
        {"kind": "delta", "timeout_s": 31},
        {"kind": "stop", "xyz": [0, 0, 1]},
        {"kind": "delta", "id": "old-protocol"},
    ],
)
def test_invalid_inputs_are_dropped_without_command_replies(bridge, payload):
    module, _, _ = bridge
    module.coordinator.reset_mock()
    send(module, **payload)
    assert module._pending_arm is None and module._pending_gripper is None
    module.coordinator.task_invoke.assert_not_called()
    module._topics.put.assert_not_called()


def test_delta_is_resolved_once_and_refreshes_same_target(bridge):
    module, feedback, targets = bridge
    send(module, kind="delta", xyz=[0, 0, 0.05])
    module._tick()
    assert targets[-1].z == pytest.approx(0.65)
    feedback["ee_pose"] = PoseStamped(position=(0.4, 0.2, 0.63))
    module._tick()
    assert len(targets) == 2 and targets[-1].z == pytest.approx(0.65)
    module.coordinator.execute_trajectory.assert_not_called()
    assert all(call.args[0] != "arm/status/json" for call in module._topics.put.call_args_list)


def test_rotated_base_deltas_use_base_axes(bridge):
    module, feedback, targets = bridge
    module._base[:3, :3] = Rotation.from_euler("z", np.pi / 2).as_matrix()
    module._base_inv = np.linalg.inv(module._base)
    feedback["ee_pose"] = PoseStamped(
        position=(0.1, 0.2, 0.3), orientation=Quaternion.from_euler(Vector3(0, 0.7, 0))
    )
    send(module, kind="delta", xyz=[0.05, 0, 0], rpy=[0.2, 0, 0])
    module._tick()
    target = pose_to_matrix(targets[-1])
    np.testing.assert_allclose(target[:3, 3], [0.1, 0.25, 0.3])
    base = module._base[:3, :3]
    expected = (
        base
        @ Rotation.from_euler("x", 0.2).as_matrix()
        @ base.T
        @ Rotation.from_euler("y", 0.7).as_matrix()
    )
    np.testing.assert_allclose(target[:3, :3], expected, atol=1e-12)


def test_latest_arm_input_replaces_target_using_current_measurement(bridge):
    module, feedback, targets = bridge
    send(module, kind="delta", xyz=[0, 0, 0.01])
    send(module, kind="delta", xyz=[0, 0, 0.04])
    module._tick()
    assert len(targets) == 1 and targets[-1].z == pytest.approx(0.64)
    feedback["ee_pose"] = PoseStamped(position=(0.4, 0.2, 0.62))
    send(module, kind="delta", xyz=[0, 0, -0.02])
    module._tick()
    assert targets[-1].z == pytest.approx(0.60)


def test_joints_use_existing_trajectory_and_gripper_is_independent(bridge):
    module, _, targets = bridge
    send(module, kind="joints", positions=[0.3, 0.4])
    module._tick()
    trajectory = module.coordinator.execute_trajectory.call_args.args[0]
    assert trajectory.joint_names == ["j1", "j2"]
    assert trajectory.points[0].positions == [0.3, 0.4]
    send(module, kind="delta", xyz=[0, 0, 0.02])
    module._tick()
    target = targets[-1]
    send(module, kind="gripper", position=0.425)
    module._tick()
    module.coordinator.task_invoke.assert_any_call(
        "arm_gripper", "set_position", {"values": [0.425]}
    )
    assert module._target is target


def test_stop_is_serviced_without_info_or_feedback_and_clears_pending(bridge):
    module, _, targets = bridge
    for _ in range(100):
        send(module, kind="delta", xyz=[0, 0, 0.02])
    send(module, kind="gripper", position=0.4)
    send(module, kind="stop")
    module._info = None
    module.coordinator.task_invoke.side_effect = (
        lambda task, method, kwargs=None: True
        if method == "cancel"
        else pytest.fail("stop requested feedback")
    )
    module._tick()
    assert not targets and module._pending_arm is None and module._pending_gripper is None
    module.coordinator.cancel_trajectory.assert_called_once()


def test_lease_is_not_extended_by_renewal(bridge, monkeypatch):
    module, _, _ = bridge
    clock = [10.0]
    monkeypatch.setattr("dimos.robot.raw_manipulation_bridge.time.monotonic", lambda: clock[0])
    send(module, kind="delta", xyz=[0, 0, 0.05], timeout_s=0.2)
    module._tick()
    clock[0] = 10.1
    module._tick()
    assert module._deadline == pytest.approx(10.2)
    clock[0] = 10.3
    module._tick()
    assert module._target is None and module._deadline is None


@pytest.mark.parametrize("failure", ["stale", "task_stopped"])
def test_target_is_not_replayed_after_stale_feedback_or_task_stop(bridge, failure):
    module, feedback, targets = bridge
    send(module, kind="delta", xyz=[0, 0, 0.05])
    module._tick()
    if failure == "stale":
        feedback["t"] -= 10
    else:
        feedback["tracking"] = False
    module._tick()
    assert module._target is None
    feedback["t"] = time.time()
    module._tick()
    assert len(targets) == 1


def test_native_feedback_and_only_camera_tfs_are_exported(bridge):
    module, feedback, _ = bridge
    ts = feedback["t"]
    module._on_tf(
        TFMessage(
            Transform(
                translation=Vector3(0.4, 0.2, 0.7),
                frame_id="world",
                child_frame_id="wrist_camera_color_optical_frame",
                ts=ts,
            ),
            Transform(translation=Vector3(9, 9, 9), frame_id="world", child_frame_id="cup", ts=ts),
        )
    )
    module._tick()
    messages = {c.args[0]: json.loads(c.args[1]) for c in module._topics.put.call_args_list}
    assert messages["arm/state/json"]["gripper_position"] == 0.85
    assert messages["arm/info/json"]["gripper"]["position_limits"] == [0, 0.85]
    np.testing.assert_allclose(messages["camera_pose/json"]["xyz"], [0.3, 0, 0.4])
    assert "cup" not in json.dumps(messages)


def test_depth_round_trip_preserves_metric_values_and_invalid_pixels():
    source = np.array([[0.123456, 0.5, 1.25], [np.nan, np.inf, 0]], dtype=">f4")[:, ::-1]
    encoded = depth_f32(Image(data=source, format=ImageFormat.DEPTH))
    np.testing.assert_allclose(
        np.frombuffer(encoded, dtype="<f4").reshape(2, 3), source, equal_nan=True
    )


def test_rgb_topics_and_depth_metadata_keep_capture_timestamps(bridge):
    module, _, _ = bridge
    module._on_image(Image(data=np.zeros((4, 6, 3), dtype=np.uint8), format=ImageFormat.RGB, ts=10))
    module._on_overview_image(
        Image(data=np.zeros((8, 12, 3), dtype=np.uint8), format=ImageFormat.RGB, ts=20)
    )
    module._on_depth(Image(data=np.ones((2, 3), dtype=np.float32), format=ImageFormat.DEPTH, ts=30))
    first, second, metadata, depth = [call.args for call in module._topics.put.call_args_list]
    assert first[0] == "camera/jpeg" and first[2] == 10
    assert second[0] == "overview/jpeg" and second[2] == 20
    assert PILImage.open(io.BytesIO(first[1])).size == (6, 4)
    assert PILImage.open(io.BytesIO(second[1])).size == (12, 8)
    assert json.loads(metadata[1])["dtype"] == "<f4"
    assert depth[0] == "camera/depth_f32" and depth[2] == metadata[2] == 30
