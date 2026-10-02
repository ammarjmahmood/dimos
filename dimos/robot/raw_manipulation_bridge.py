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

"""Plain JSON/JPEG/depth transport for the robot's low-level manipulation control."""

from __future__ import annotations

import json
from typing import Any
from uuid import uuid4

import numpy as np

from dimos.control.manipulation_types import COMMAND, ArmCommand, ManipulationFeedback, StopCommand
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.evals.constants import RAW_ENDPOINT, RAW_JPEG_QUALITY, RAW_TOPIC_PREFIX
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.robot.raw_robot_bridge import RawTopics, jpeg_bytes
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_FEEDBACK_TOPICS = {
    "state": "arm/state/json",
    "info": "arm/info/json",
    "status": "arm/status/json",
    "camera_pose": "camera_pose/json",
    "overview_camera_pose": "overview/camera_pose/json",
}


def depth_f32(image: Image) -> bytes:
    """Encode metric optical-axis depth without quantization or image compression."""
    depth = image.as_numpy()
    if image.format != ImageFormat.DEPTH or depth.ndim != 2 or depth.dtype.kind != "f":
        raise ValueError("Raw depth requires a 2D floating-point DEPTH image in metres")
    return np.ascontiguousarray(depth, dtype="<f4").tobytes()


class RawManipulationBridgeConfig(ModuleConfig):
    endpoint: str = RAW_ENDPOINT
    prefix: str = RAW_TOPIC_PREFIX
    jpeg_quality: int = RAW_JPEG_QUALITY


class RawManipulationBridge(Module):
    """Translate wire messages and typed robot streams; no robot model or motion execution."""

    config: RawManipulationBridgeConfig
    manipulation_command: Out[ArmCommand]
    manipulation_feedback: In[ManipulationFeedback]
    color_image: In[Image]
    depth_image: In[Image]
    camera_info: In[CameraInfo]
    overview_image: In[Image]
    overview_camera_info: In[CameraInfo]

    _topics: RawTopics | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self._topics = RawTopics(self.config.endpoint, self.config.prefix, listen=True)
        self._subscriber = self._topics.subscribe("arm/command/json", self._on_command)
        self.manipulation_feedback.subscribe(self._on_feedback)
        self.color_image.subscribe(self._on_image)
        self.depth_image.subscribe(self._on_depth)
        self.camera_info.subscribe(self._on_camera_info)
        self.overview_image.subscribe(self._on_overview_image)
        self.overview_camera_info.subscribe(self._on_overview_camera_info)

    @rpc
    def stop(self) -> None:
        if self._topics is not None:
            self.manipulation_command.publish(StopCommand(id=f"bridge-stop-{uuid4()}", kind="stop"))
            self._topics.close()
            self._topics = None
        super().stop()

    def _put(self, key: str, value: Any, ts: float | None = None) -> None:
        if self._topics is not None:
            self._topics.put(key, json.dumps(value, allow_nan=False), ts)

    def _on_command(self, payload: bytes, _ts: float | None) -> None:
        try:
            command = COMMAND.validate_json(payload)
        except ValueError as exc:
            self._put("arm/status/json", {"id": None, "status": "rejected", "reason": str(exc)})
            return
        self.manipulation_command.publish(command)

    def _on_feedback(self, feedback: ManipulationFeedback) -> None:
        self._put(_FEEDBACK_TOPICS[feedback.kind], feedback.data, feedback.ts)

    def _on_image(self, image: Image) -> None:
        if self._topics is not None:
            self._topics.put("camera/jpeg", jpeg_bytes(image, self.config.jpeg_quality), image.ts)

    def _on_depth(self, image: Image) -> None:
        if self._topics is None:
            return
        try:
            payload = depth_f32(image)
        except ValueError as exc:
            logger.warning("Raw manipulation depth frame rejected", error=str(exc))
            return
        self._put(
            "camera/depth_info/json",
            {
                "t": image.ts,
                "width": image.width,
                "height": image.height,
                "dtype": "<f4",
                "unit": "metres",
                "frame_id": image.frame_id,
            },
            image.ts,
        )
        self._topics.put("camera/depth_f32", payload, image.ts)

    def _on_camera_info(self, info: CameraInfo) -> None:
        self._put(
            "camera_info/json", {"width": info.width, "height": info.height, "K": info.K}, info.ts
        )

    def _on_overview_image(self, image: Image) -> None:
        if self._topics is not None:
            self._topics.put("overview/jpeg", jpeg_bytes(image, self.config.jpeg_quality), image.ts)

    def _on_overview_camera_info(self, info: CameraInfo) -> None:
        self._put(
            "overview/camera_info/json",
            {"width": info.width, "height": info.height, "K": info.K},
            info.ts,
        )
