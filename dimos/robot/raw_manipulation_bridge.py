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

"""Plain agent inputs to existing control tasks, with continuous robot observations.

No command replies, IDs, completion state machine, robot model or IK solver.
The adapter renews a fixed pose target until its lease expires; the task's own
watchdog stops tracking if this adapter stops renewing it.
"""

from __future__ import annotations

import json
import math
import threading
import time
from typing import Annotated, Any, Literal, Protocol

import numpy as np
from numpy.typing import NDArray
from pydantic import BaseModel, ConfigDict, Field, TypeAdapter
from scipy.spatial.transform import Rotation

from dimos.control.tasks.trajectory_task.trajectory_task import (
    TrajectoryCancellationResult,
    TrajectoryExecutionResult,
    TrajectoryExecutionStatus,
)
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.evals.constants import RAW_ENDPOINT, RAW_JPEG_QUALITY, RAW_TOPIC_PREFIX
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.robot.manipulators.common.topics import CARTESIAN_IK_TASK_NAME
from dimos.robot.raw_robot_bridge import RawTopics, jpeg_bytes
from dimos.spec.utils import Spec
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import matrix_to_pose, pose_to_matrix

logger = setup_logger()


class _Command(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False, strict=True)


class DeltaCommand(_Command):
    kind: Literal["delta"]
    xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)
    frame: Literal["base"] = "base"
    timeout_s: float = Field(default=10, gt=0, le=30)


class JointsCommand(_Command):
    kind: Literal["joints"]
    positions: list[float] = Field(min_length=1)
    timeout_s: float = Field(default=10, gt=0, le=30)


class GripperCommand(_Command):
    kind: Literal["gripper"]
    position: float


class StopCommand(_Command):
    kind: Literal["stop"]


COMMAND: TypeAdapter[DeltaCommand | JointsCommand | GripperCommand | StopCommand] = TypeAdapter(
    Annotated[
        DeltaCommand | JointsCommand | GripperCommand | StopCommand, Field(discriminator="kind")
    ]
)


class ArmCoordinator(Spec, Protocol):
    def task_invoke(
        self, task_name: str, method: str, kwargs: dict[str, Any] | None = None
    ) -> Any: ...
    def execute_trajectory(self, trajectory: JointTrajectory) -> TrajectoryExecutionResult: ...
    def cancel_trajectory(self) -> TrajectoryCancellationResult: ...


def pose_json(matrix: NDArray[np.float64]) -> dict[str, Any]:
    return {
        "xyz": matrix[:3, 3].tolist(),
        "quaternion_xyzw": Rotation.from_matrix(matrix[:3, :3]).as_quat().tolist(),
    }


def depth_f32(image: Image) -> bytes:
    """Metric optical-axis depth without quantization or image compression."""
    depth = image.as_numpy()
    if image.format != ImageFormat.DEPTH or depth.ndim != 2 or depth.dtype.kind != "f":
        raise ValueError("Raw depth requires a 2D floating-point DEPTH image in metres")
    return np.ascontiguousarray(depth, dtype="<f4").tobytes()


class RawManipulationBridgeConfig(ModuleConfig):
    endpoint: str = RAW_ENDPOINT
    prefix: str = RAW_TOPIC_PREFIX
    cartesian_task: str = CARTESIAN_IK_TASK_NAME
    gripper_task: str = "arm_gripper"
    gripper_joint: str = "arm/gripper"
    gripper_unit: str = "radians"
    camera_optical_frame: str = "wrist_camera_color_optical_frame"
    overview_optical_frame: str = "env_camera_color_optical_frame"
    rate_hz: float = Field(default=20, gt=0, le=100, allow_inf_nan=False)
    stale_s: float = Field(default=1, gt=0, allow_inf_nan=False)
    max_delta_m: float = Field(default=0.2, gt=0, allow_inf_nan=False)
    max_delta_rad: float = Field(default=math.pi / 2, gt=0, allow_inf_nan=False)
    jpeg_quality: int = RAW_JPEG_QUALITY


class RawManipulationBridge(Module):
    """Another input source for Cartesian, trajectory and gripper task cards."""

    config: RawManipulationBridgeConfig
    coordinator: ArmCoordinator
    color_image: In[Image]
    depth_image: In[Image]
    camera_info: In[CameraInfo]
    overview_image: In[Image]
    overview_camera_info: In[CameraInfo]
    tf: In[TFMessage]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._topics: RawTopics | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._pending_arm: DeltaCommand | JointsCommand | None = None
        self._pending_gripper: GripperCommand | None = None
        self._stop_pending = False
        self._target: PoseStamped | None = None
        self._deadline: float | None = None
        self._info: dict[str, Any] | None = None
        self._base = np.eye(4)
        self._base_inv = np.eye(4)
        self._cameras: dict[str, tuple[str, NDArray[np.float64], float]] = {}
        self._last_info = float("-inf")
        self._next_discovery = 0.0

    @rpc
    def start(self) -> None:
        super().start()
        self._topics = RawTopics(self.config.endpoint, self.config.prefix, listen=True)
        self._subscriber = self._topics.subscribe("arm/command/json", self._on_command)
        self.color_image.subscribe(self._on_image)
        self.depth_image.subscribe(self._on_depth)
        self.camera_info.subscribe(self._on_camera_info)
        self.overview_image.subscribe(self._on_overview_image)
        self.overview_camera_info.subscribe(self._on_overview_camera_info)
        self.tf.subscribe(self._on_tf)
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run, name="raw-manipulation-input", daemon=True
        )
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=5)
        if self._topics is not None:
            try:
                self._stop_arm()
            finally:
                self._topics.close()
                self._topics = None
        super().stop()

    def _invoke(self, method: str, **kwargs: Any) -> Any:
        return self.coordinator.task_invoke(self.config.cartesian_task, method, kwargs)

    def _stop_arm(self) -> None:
        self._target, self._deadline = None, None
        try:
            if self._invoke("cancel") is not True:
                raise RuntimeError("Cartesian task did not acknowledge cancellation")
        finally:
            result = self.coordinator.cancel_trajectory()
            if result is None or not result.safe:
                raise RuntimeError("Trajectory cancellation was not acknowledged")

    def _on_command(self, payload: bytes, _ts: float | None) -> None:
        try:
            command = COMMAND.validate_json(payload)
            if isinstance(command, DeltaCommand) and (
                np.linalg.norm(command.xyz) > self.config.max_delta_m
                or np.linalg.norm(command.rpy) > self.config.max_delta_rad
            ):
                raise ValueError("delta exceeds configured limits")
            if self._info is not None:
                if isinstance(command, JointsCommand):
                    limits = self._info["joint_limits"]
                    if len(command.positions) != len(limits) or any(
                        not lo <= q <= hi
                        for q, (lo, hi) in zip(command.positions, limits, strict=True)
                    ):
                        raise ValueError("joint target must match robot joint order and limits")
                if isinstance(command, GripperCommand):
                    lo, hi = self._info["gripper"]["position_limits"]
                    if not lo <= command.position <= hi:
                        raise ValueError("gripper position outside native limits")
        except ValueError as exc:
            logger.warning("Raw command rejected", error=str(exc))
            return
        with self._lock:
            if isinstance(command, StopCommand):
                self._stop_pending = True
                self._pending_arm = None
                self._pending_gripper = None
            elif not self._stop_pending:
                if isinstance(command, GripperCommand):
                    self._pending_gripper = command
                else:
                    self._pending_arm = command

    def _load_info(self) -> bool:
        info = self._invoke("get_control_info")
        limits = self.coordinator.task_invoke(self.config.gripper_task, "get_limits")
        if info is None or limits is None:
            return False
        info = dict(info)
        self._base = np.asarray(pose_to_matrix(info.pop("base_pose")), dtype=np.float64)
        self._base_inv = np.asarray(np.linalg.inv(self._base), dtype=np.float64)
        info["gripper"] = {
            "joint": self.config.gripper_joint,
            "unit": self.config.gripper_unit,
            "position_limits": limits[self.config.gripper_joint],
        }
        info.update(
            max_delta_m=self.config.max_delta_m,
            max_delta_rad=self.config.max_delta_rad,
            commands=["delta", "joints", "gripper", "stop"],
        )
        self._info = info
        return True

    def _apply_arm(
        self, command: DeltaCommand | JointsCommand, measured: PoseStamped, now: float
    ) -> None:
        assert self._info is not None
        if isinstance(command, JointsCommand):
            limits = self._info["joint_limits"]
            if len(command.positions) != len(limits) or any(
                not lo <= q <= hi for q, (lo, hi) in zip(command.positions, limits, strict=True)
            ):
                raise ValueError("joint target must match robot joint order and limits")
        self._stop_arm()
        if isinstance(command, DeltaCommand):
            target = self._base_inv @ pose_to_matrix(measured)
            target[:3, 3] += command.xyz
            target[:3, :3] = Rotation.from_euler("xyz", command.rpy).as_matrix() @ target[:3, :3]
            pose = matrix_to_pose(self._base @ target)
            self._target = PoseStamped(
                position=pose.position, orientation=pose.orientation, frame_id="world"
            )
            self._send_target()
        else:
            result = self.coordinator.execute_trajectory(
                JointTrajectory(
                    joint_names=list(self._info["joint_names"]),
                    points=[TrajectoryPoint(positions=command.positions)],
                )
            )
            if result is None or result.status is not TrajectoryExecutionStatus.ACCEPTED:
                raise RuntimeError(f"Joint task rejected input: {result}")
        self._deadline = now + command.timeout_s

    def _send_target(self) -> None:
        if self._invoke("on_cartesian_command", pose=self._target, t_now=None) is not True:
            raise RuntimeError("Cartesian task rejected input")

    def _tick(self) -> None:
        with self._lock:
            stopping, self._stop_pending = self._stop_pending, False
            arm, self._pending_arm = self._pending_arm, None
            gripper, self._pending_gripper = self._pending_gripper, None
            cameras = self._cameras.copy()
        if stopping:
            self._stop_arm()
            return
        if self._info is None:
            now = time.monotonic()
            if now < self._next_discovery:
                return
            self._next_discovery = now + 0.5
            if not self._load_info():
                return
        assert self._info is not None
        feedback = self._invoke("get_feedback", state=None)
        if (
            feedback is None
            or abs(time.time() - feedback["t"]) > self.config.stale_s
            or feedback["ee_pose"] is None
        ):
            if self._deadline is not None:
                self._stop_arm()
            return
        positions = feedback["positions"]
        names = self._info["joint_names"]
        if not all(
            n in positions and math.isfinite(positions[n])
            for n in (*names, self.config.gripper_joint)
        ):
            raise ValueError("Robot feedback is missing or non-finite")
        measured = feedback["ee_pose"]
        self._put(
            "arm/state/json",
            {
                "t": feedback["t"],
                "frame": "base",
                "joint_names": names,
                "positions": [positions[n] for n in names],
                "velocities": [feedback["velocities"].get(n, 0.0) for n in names],
                "ee_pose": pose_json(np.asarray(self._base_inv @ pose_to_matrix(measured))),
                "gripper_position": positions[self.config.gripper_joint],
            },
            feedback["t"],
        )
        for frame, topic in (
            (self.config.camera_optical_frame, "camera_pose/json"),
            (self.config.overview_optical_frame, "overview/camera_pose/json"),
        ):
            camera = cameras.get(frame)
            if camera is None or abs(time.time() - camera[2]) > self.config.stale_s:
                continue
            parent, matrix, ts = camera
            if parent == "world":
                matrix = self._base_inv @ matrix
            elif parent != self._info["base_frame"]:
                continue
            self._put(topic, {"t": ts, "frame": "base", **pose_json(matrix)}, ts)
        now = time.monotonic()
        if now - self._last_info >= 1:
            self._put("arm/info/json", self._info)
            self._last_info = now
        with self._lock:
            if self._stop_pending:
                return
        if self._deadline is not None and now >= self._deadline:
            self._stop_arm()
        if arm is not None:
            self._apply_arm(arm, measured, now)
        elif self._target is not None:
            if feedback["tracking"]:
                self._send_target()  # Same absolute target, never another delta.
            else:
                self._target, self._deadline = None, None  # Do not undo a task stop/preemption.
        if gripper is not None:
            lo, hi = self._info["gripper"]["position_limits"]
            if not lo <= gripper.position <= hi:
                raise ValueError("gripper position outside native limits")
            if (
                self.coordinator.task_invoke(
                    self.config.gripper_task, "set_position", {"values": [gripper.position]}
                )
                is not True
            ):
                raise RuntimeError("Gripper task rejected input")

    def _run(self) -> None:
        while not self._stop.wait(1 / self.config.rate_hz):
            try:
                self._tick()
            except Exception:
                logger.exception("Raw manipulation input failed")
                try:
                    self._stop_arm()
                except Exception:
                    logger.exception("Task cancellation failed; Cartesian watchdog remains active")

    def _put(self, key: str, value: Any, ts: float | None = None) -> None:
        if self._topics is not None:
            self._topics.put(key, json.dumps(value, allow_nan=False), ts)

    def _on_tf(self, message: TFMessage) -> None:
        for transform in message.transforms:
            if transform.child_frame_id not in (
                self.config.camera_optical_frame,
                self.config.overview_optical_frame,
            ):
                continue
            matrix = np.eye(4)
            matrix[:3, 3] = transform.translation.to_numpy()
            matrix[:3, :3] = Rotation.from_quat(transform.rotation.to_numpy()).as_matrix()
            with self._lock:
                self._cameras[transform.child_frame_id] = (transform.frame_id, matrix, transform.ts)

    def _on_image(self, image: Image) -> None:
        if self._topics is not None:
            self._topics.put("camera/jpeg", jpeg_bytes(image, self.config.jpeg_quality), image.ts)

    def _on_depth(self, image: Image) -> None:
        if self._topics is None:
            return
        try:
            payload = depth_f32(image)
        except ValueError as exc:
            logger.warning("Raw depth frame rejected", error=str(exc))
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
