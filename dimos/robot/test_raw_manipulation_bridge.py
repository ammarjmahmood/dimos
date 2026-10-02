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
from unittest.mock import MagicMock

import numpy as np
from PIL import Image as PILImage
import pytest

from dimos.control.manipulation_types import DeltaTarget, ManipulationFeedback, StopCommand
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.robot.raw_manipulation_bridge import RawManipulationBridge, depth_f32


@pytest.fixture
def bridge():
    module = RawManipulationBridge()
    module._topics = MagicMock()
    commands = []
    module.manipulation_command.subscribe(commands.append)
    yield module, commands
    module.stop()


@pytest.mark.parametrize(
    "payload",
    [
        {"id": "a", "kind": "delta", "xyz": [0, 0, float("nan")]},
        {"id": "a", "kind": "delta", "rpy": [0, float("inf"), 0]},
        {"id": "a", "kind": "delta", "xyz": [0, 1]},
        {"id": "a", "kind": "delta", "frame": "tool"},
        {"id": "a", "kind": "joints", "positions": ["0.1"]},
        {"id": "a", "kind": "joints", "positions": [True]},
        {"id": "a", "kind": "gripper", "opening": 1.5},
        {"id": "a", "kind": "delta", "timeout_s": -1},
        {"id": "a", "kind": "delta", "timeout_s": 31},
        {"id": "a", "kind": "stop", "xyz": [0, 0, 1]},
    ],
)
def test_invalid_commands_are_rejected_without_forwarding(bridge, payload):
    module, commands = bridge
    module._on_command(json.dumps(payload).encode(), None)
    assert not commands
    topic, payload, _ = module._topics.put.call_args.args
    assert topic == "arm/status/json"
    assert json.loads(payload)["status"] == "rejected"


def test_json_commands_forward_without_a_robot_model(bridge):
    module, commands = bridge
    module._on_command(b'{"id":"up","kind":"delta","xyz":[0,0,0.05],"rpy":[0,0,0.1]}', None)
    assert commands == [DeltaTarget(id="up", kind="delta", xyz=(0, 0, 0.05), rpy=(0, 0, 0.1))]
    module._on_feedback(ManipulationFeedback("status", {"id": "up", "status": "succeeded"}, 42.0))
    topic, payload, ts = module._topics.put.call_args.args
    assert topic == "arm/status/json" and ts == 42.0
    assert json.loads(payload) == {"id": "up", "status": "succeeded"}


def test_shutdown_requests_stop_before_closing_transport(bridge):
    module, commands = bridge
    topics = module._topics
    module.stop()
    assert isinstance(commands[-1], StopCommand)
    topics.close.assert_called_once()


def test_depth_round_trip_preserves_metric_values_shape_and_invalid_pixels():
    original = np.array([[0.123456, 0.5, 1.25], [np.nan, np.inf, 0.0]], dtype=">f4")
    source = original[:, ::-1]
    encoded = depth_f32(Image(data=source, format=ImageFormat.DEPTH, ts=7.0))
    assert len(encoded) == 2 * 3 * 4
    decoded = np.frombuffer(encoded, dtype="<f4").reshape(2, 3)
    np.testing.assert_allclose(decoded, source, equal_nan=True)
    assert decoded[0, 0] == 1.25


@pytest.mark.parametrize(
    "format,dtype,shape",
    [
        (ImageFormat.DEPTH16, np.uint16, (2, 3)),
        (ImageFormat.RGB, np.uint8, (2, 3, 3)),
    ],
)
def test_non_metric_depth_is_not_silently_misinterpreted(format, dtype, shape):
    with pytest.raises(ValueError, match="floating-point DEPTH"):
        depth_f32(Image(data=np.ones(shape, dtype=dtype), format=format))


def test_depth_metadata_matches_binary_frame(bridge):
    module, _ = bridge
    image = Image(
        data=np.array([[0.2, 0.3, 0.4], [1, 2, 3]], dtype=np.float32),
        format=ImageFormat.DEPTH,
        ts=12.5,
        frame_id="wrist_camera_color_optical_frame",
    )
    module._on_depth(image)
    metadata, frame = [call.args for call in module._topics.put.call_args_list]
    assert metadata[0] == "camera/depth_info/json"
    assert json.loads(metadata[1]) == {
        "t": 12.5,
        "width": 3,
        "height": 2,
        "dtype": "<f4",
        "unit": "metres",
        "frame_id": image.frame_id,
    }
    assert frame[0] == "camera/depth_f32" and frame[2] == 12.5
    np.testing.assert_allclose(np.frombuffer(frame[1], dtype="<f4").reshape(2, 3), image.data)


def test_overview_and_wrist_use_distinct_rgb_topics(bridge):
    module, _ = bridge
    module._on_image(
        Image(data=np.zeros((4, 6, 3), dtype=np.uint8), format=ImageFormat.RGB, ts=10.0)
    )
    module._on_overview_image(
        Image(data=np.full((8, 12, 3), 200, dtype=np.uint8), format=ImageFormat.RGB, ts=20.0)
    )
    first, second = [call.args for call in module._topics.put.call_args_list]
    assert first[0] == "camera/jpeg" and first[2] == 10.0
    assert second[0] == "overview/jpeg" and second[2] == 20.0
    assert PILImage.open(io.BytesIO(first[1])).size == (6, 4)
    assert PILImage.open(io.BytesIO(second[1])).size == (12, 8)


def test_overview_calibration_has_its_own_topic(bridge):
    module, _ = bridge
    info = CameraInfo.from_fov(fov_deg=45, width=640, height=480).with_ts(12.0)
    module._on_overview_camera_info(info)
    topic, payload, ts = module._topics.put.call_args.args
    assert topic == "overview/camera_info/json" and ts == 12.0
    assert json.loads(payload)["K"][0] == pytest.approx(579.41125497)
