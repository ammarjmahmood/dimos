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

"""Feedback-bounded manipulation commands over the existing solver and coordinator."""

from __future__ import annotations

from dataclasses import dataclass
import math
from queue import Empty, Full, Queue, SimpleQueue
import threading
import time
from typing import Any, Protocol

import numpy as np
from numpy.typing import NDArray
from pydantic import Field, model_validator
from scipy.spatial.transform import Rotation

from dimos.control.manipulation_types import (
    ArmCommand,
    Command,
    DeltaTarget,
    FeedbackKind,
    GripperTarget,
    JointTarget,
    ManipulationFeedback,
    StopCommand,
)
from dimos.control.tasks.pose_target_ik import PinkPoseTargetSolver, PoseTargetIKTaskConfig
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.spec.utils import Spec
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import matrix_to_pose, pose_to_matrix

logger = setup_logger()


class ArmCoordinator(Spec, Protocol):
    def task_invoke(
        self, task_name: str, method: str, kwargs: dict[str, Any] | None = None
    ) -> Any: ...


def delta_pose(
    current: NDArray[np.float64], xyz: tuple[float, float, float], rpy: tuple[float, float, float]
) -> NDArray[np.float64]:
    """Translate along base axes; left-compose extrinsic XYZ Euler rotation."""
    target = current.copy()
    target[:3, 3] += xyz
    target[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix() @ current[:3, :3]
    return target


def pose_json(matrix: NDArray[np.float64]) -> dict[str, Any]:
    rotation = Rotation.from_matrix(matrix[:3, :3])
    return {"xyz": matrix[:3, 3].tolist(), "quaternion_xyzw": rotation.as_quat().tolist()}


class ManipulationControlConfig(ModuleConfig):
    model: RobotModelConfig | None = None
    ee_frame: str = "link_tcp"
    camera_mount_frame: str = "link7"
    camera_optical_frame: str = "wrist_camera_color_optical_frame"
    overview_optical_frame: str = "env_camera_color_optical_frame"
    trajectory_task: str = "joint_trajectory"
    gripper_task: str = "arm_gripper"
    gripper_joint: str = "arm/gripper"
    gripper_range: tuple[float, float] = (0.0, 0.85)
    rate_hz: float = Field(default=20.0, gt=0, le=100, allow_inf_nan=False)
    stale_s: float = Field(default=1.0, gt=0, allow_inf_nan=False)
    max_joint_velocity: float = Field(default=0.5, gt=0, allow_inf_nan=False)
    max_delta_m: float = Field(default=0.20, gt=0, allow_inf_nan=False)
    max_delta_rad: float = Field(default=math.pi / 2, gt=0, allow_inf_nan=False)
    position_tolerance_m: float = Field(default=0.005, gt=0, allow_inf_nan=False)
    rotation_tolerance_rad: float = Field(default=0.03, gt=0, allow_inf_nan=False)
    joint_tolerance_rad: float = Field(default=0.02, gt=0, allow_inf_nan=False)
    settled_s: float = Field(default=0.2, ge=0, allow_inf_nan=False)

    @model_validator(mode="after")
    def check_gripper_range(self) -> ManipulationControlConfig:
        low, high = self.gripper_range
        if not (math.isfinite(low) and math.isfinite(high) and low < high):
            raise ValueError("gripper_range must be finite and ordered")
        return self


@dataclass
class ActiveCommand:
    command: JointTarget | DeltaTarget | GripperTarget
    deadline: float
    target: NDArray[np.float64] | None = None
    settled_since: float | None = None
    commanded_joints: JointState | None = None


class ManipulationControl(Module):
    """Robot-specific FK, bounded IK and command lifecycle; independent of transport."""

    config: ManipulationControlConfig
    coordinator: ArmCoordinator
    manipulation_command: In[ArmCommand]
    manipulation_feedback: Out[ManipulationFeedback]
    coordinator_joint_state: In[JointState]
    tf: In[TFMessage]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._started = False
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._state: JointState | None = None
        self._received = 0.0
        self._camera_transforms: dict[str, tuple[str, NDArray[np.float64], float]] = {}
        self._queue: Queue[Command] = Queue(maxsize=32)
        self._stop_commands: SimpleQueue[StopCommand] = SimpleQueue()
        self._active: ActiveCommand | None = None
        self._results: dict[str, dict[str, Any]] = {}
        self._requests: dict[str, str] = {}
        self._last_result: dict[str, Any] | None = None
        self._last_info_publish = float("-inf")
        self._last_status_publish = float("-inf")

    @rpc
    def start(self) -> None:
        model = self.config.model
        if model is None:
            raise ValueError("ManipulationControl requires a robot model from its blueprint")
        super().start()
        self._joint_names = tuple(model.joint_names)
        loaded = model.model.load()
        limits = [loaded.get_joint(name) for name in self._joint_names]
        self._limits = [
            (
                j.lower if j.lower is not None else -2 * math.pi,
                j.upper if j.upper is not None else 2 * math.pi,
            )
            for j in limits
            if j is not None
        ]
        if len(self._limits) != len(self._joint_names):
            raise ValueError("Robot model is missing controlled joints")
        self._base = np.asarray(pose_to_matrix(model.base_pose), dtype=np.float64)
        self._base_inv = np.linalg.inv(self._base)
        self._solver = PinkPoseTargetSolver(
            PoseTargetIKTaskConfig(
                joint_names=self._joint_names,
                robot_model=model,
                target_frames=(self.config.ee_frame,),
                max_joint_velocity_rad_s=self.config.max_joint_velocity,
            )
        )
        self.manipulation_command.subscribe(self._on_command)
        self.coordinator_joint_state.subscribe(self._on_state)
        self.tf.subscribe(self._on_tf)
        self._stop.clear()
        self._started = True
        self._thread = threading.Thread(target=self._run, name="manipulation-control", daemon=True)
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=5.0)
        if self._started:
            try:
                self._hold()
            finally:
                self._started = False
        super().stop()

    def _emit(self, kind: FeedbackKind, value: dict[str, Any], ts: float | None = None) -> None:
        self.manipulation_feedback.publish(ManipulationFeedback(kind, value, ts))

    def _on_state(self, state: JointState) -> None:
        with self._lock:
            self._state, self._received = state, time.monotonic()

    def _on_tf(self, message: TFMessage) -> None:
        # Never forward simulator object ground truth from the same tf stream.
        for transform in message.transforms:
            if transform.child_frame_id in (
                self.config.camera_optical_frame,
                self.config.overview_optical_frame,
            ):
                matrix = np.eye(4)
                matrix[:3, 3] = transform.translation.to_numpy()
                matrix[:3, :3] = Rotation.from_quat(transform.rotation.to_numpy()).as_matrix()
                with self._lock:
                    self._camera_transforms[transform.child_frame_id] = (
                        transform.frame_id,
                        matrix,
                        transform.ts,
                    )

    def _on_command(self, command: ArmCommand) -> None:
        if isinstance(command, StopCommand):
            self._stop_commands.put(command)
            return
        if not isinstance(command, (JointTarget, DeltaTarget, GripperTarget)):
            self._emit(
                "status", {"id": command.id, "status": "rejected", "reason": "unknown command"}
            )
            return
        try:
            self._queue.put_nowait(command)
        except Full:
            self._emit(
                "status",
                {
                    "id": command.id,
                    "status": "rejected",
                    "reason": "command queue full; retry later",
                },
            )

    def _status(self, command: Command, status: str, reason: str = "") -> None:
        self._requests.setdefault(command.id, command.model_dump_json())
        result = {"id": command.id, "status": status, "reason": reason, "t": time.time()}
        self._results[command.id] = self._last_result = result
        self._emit("status", result)
        self._last_status_publish = time.monotonic()

    def _invoke(self, task: str, method: str, **kwargs: Any) -> Any:
        return self.coordinator.task_invoke(task, method, kwargs)

    def _send_joints(self, positions: list[float]) -> None:
        result = self._invoke(
            self.config.trajectory_task,
            "execute",
            trajectory=JointTrajectory(
                joint_names=list(self._joint_names), points=[TrajectoryPoint(positions=positions)]
            ),
            current_positions={},
        )
        if result is None or result.status.name != "ACCEPTED":
            raise RuntimeError(f"Joint controller rejected command: {result}")

    def _hold(self) -> None:
        result = self._invoke(self.config.trajectory_task, "cancel")
        if result is None:
            raise RuntimeError("Joint controller did not acknowledge stop")
        self._solver.reset()
        # Cancellation keeps the last bounded arm setpoint. Gripper holds its
        # last target so stopping arm motion does not release an object.

    def _accept(
        self, command: Command, state: JointState, ee: NDArray[np.float64], now: float
    ) -> None:
        request = command.model_dump_json()
        if command.id in self._requests:
            if self._requests[command.id] != request:
                self._emit(
                    "status",
                    {"id": command.id, "status": "rejected", "reason": "id_conflict"},
                )
            else:
                self._emit("status", self._results[command.id])
            return
        if len(self._requests) >= 10000 and not isinstance(command, StopCommand):
            self._emit(
                "status",
                {"id": command.id, "status": "rejected", "reason": "command history full"},
            )
            return
        self._requests[command.id] = request
        if isinstance(command, StopCommand):
            self._hold()
            if self._active:
                self._status(self._active.command, "cancelled")
            self._active = None
            self._discard_pending("cancelled by stop")
            self._status(command, "succeeded")
            return
        if self._active is not None:
            self._status(command, "rejected", "busy; wait for completion or send stop")
            return
        active = ActiveCommand(command, now + command.timeout_s)
        if isinstance(command, JointTarget):
            if len(command.positions) != len(self._joint_names):
                raise ValueError("positions must follow the joint_names order in arm/info/json")
            if any(
                not low <= q <= high
                for q, (low, high) in zip(command.positions, self._limits, strict=True)
            ):
                raise ValueError("joint target outside model limits")
            active.target = np.asarray(command.positions)
            self._send_joints(command.positions)
        elif isinstance(command, DeltaTarget):
            if np.linalg.norm(command.xyz) > self.config.max_delta_m:
                raise ValueError("translation exceeds max_delta_m")
            if np.linalg.norm(command.rpy) > self.config.max_delta_rad:
                raise ValueError("rotation exceeds max_delta_rad")
            active.target = self._base @ delta_pose(self._base_inv @ ee, command.xyz, command.rpy)
            self._solver.reset()
        else:
            accepted = self._invoke(
                self.config.gripper_task, "set_normalized", values=[command.opening]
            )
            if not accepted:
                raise RuntimeError("Gripper controller rejected command")
        self._active = active
        self._status(command, "running")

    def _discard_pending(self, reason: str) -> None:
        for _ in range(32):
            try:
                pending = self._queue.get_nowait()
            except Empty:
                break
            if pending.id not in self._requests:
                self._status(pending, "rejected", reason)

    def _advance(self, state: JointState, ee: NDArray[np.float64], now: float) -> None:
        active = self._active
        if active is None:
            return
        command = active.command
        positions = dict(zip(state.name, state.position, strict=True))
        if now >= active.deadline:
            self._hold()
            self._status(command, "timed_out", "target not reached; arm stopped")
            self._active = None
            return
        if isinstance(command, JointTarget):
            reached = (
                max(
                    abs(positions[n] - q)
                    for n, q in zip(self._joint_names, command.positions, strict=True)
                )
                < self.config.joint_tolerance_rad
            )
        elif isinstance(command, DeltaTarget):
            assert active.target is not None
            distance = np.linalg.norm(ee[:3, 3] - active.target[:3, 3])
            angle = Rotation.from_matrix(active.target[:3, :3] @ ee[:3, :3].T).magnitude()
            reached = bool(
                distance < self.config.position_tolerance_m
                and angle < self.config.rotation_tolerance_rad
            )
            if not reached:
                servo_target = active.target.copy()
                if active.commanded_joints is not None:
                    commanded_pose = self._solver.frame_poses(
                        active.commanded_joints, (self.config.ee_frame,)
                    )[self.config.ee_frame]
                    commanded = np.asarray(pose_to_matrix(commanded_pose))
                    # Close the Cartesian loop on measured feedback, compensating
                    # servo tracking error (e.g. gravity sag) rather than declaring
                    # success when only the kinematic command reaches the target.
                    servo_target[:3, 3] += commanded[:3, 3] - ee[:3, 3]
                    servo_target[:3, :3] = active.target[:3, :3] @ ee[:3, :3].T @ commanded[:3, :3]
                pose = matrix_to_pose(servo_target)
                joints = self._solver.step(
                    {
                        self.config.ee_frame: PoseStamped(
                            position=pose.position, orientation=pose.orientation, frame_id="world"
                        )
                    },
                    state,
                    1.0 / self.config.rate_hz,
                )
                if joints is None:
                    raise RuntimeError("Cartesian controller could not produce a joint target")
                selected = dict(zip(joints.name, joints.position, strict=True))
                self._send_joints([selected[n] for n in self._joint_names])
                active.commanded_joints = joints
        else:
            low, high = self.config.gripper_range
            opening = (positions[self.config.gripper_joint] - low) / (high - low)
            reached = abs(opening - command.opening) < 0.08
        if reached:
            if active.settled_since is None:
                active.settled_since = now
            if now - active.settled_since >= self.config.settled_s:
                self._hold()
                self._status(command, "succeeded")
                self._active = None
        else:
            active.settled_since = None

    def _tick(self) -> None:
        now = time.monotonic()
        stopped = False
        for _ in range(32):
            try:
                stop = self._stop_commands.get_nowait()
            except Empty:
                break
            try:
                self._accept(stop, JointState(), np.eye(4), now)
            except Exception as exc:
                self._status(stop, "error", str(exc))
                raise
            stopped = True
        if stopped:
            return
        with self._lock:
            state, received = self._state, self._received
            cameras = self._camera_transforms.copy()
        if (
            state is None
            or now - received > self.config.stale_s
            or abs(time.time() - state.ts) > self.config.stale_s
        ):
            if self._active:
                self._hold()
                self._status(self._active.command, "error", "joint feedback stale")
                self._active = None
            # A retry of an already executed command must remain idempotent even
            # while feedback is unavailable.
            for _ in range(32):
                try:
                    command = self._queue.get_nowait()
                except Empty:
                    break
                if command.id in self._requests:
                    if self._requests[command.id] == command.model_dump_json():
                        self._emit("status", self._results[command.id])
                    else:
                        self._emit(
                            "status",
                            {"id": command.id, "status": "rejected", "reason": "id_conflict"},
                        )
                elif isinstance(command, StopCommand):
                    self._hold()
                    self._status(command, "succeeded")
                else:
                    self._status(command, "rejected", "joint feedback unavailable")
            return
        positions = dict(zip(state.name, state.position, strict=True))
        if not all(
            n in positions and math.isfinite(positions[n])
            for n in (*self._joint_names, self.config.gripper_joint)
        ):
            raise ValueError("Missing or non-finite robot joint feedback")
        frames = self._solver.frame_poses(
            state, (self.config.ee_frame, self.config.camera_mount_frame)
        )
        ee = np.asarray(pose_to_matrix(frames[self.config.ee_frame]), dtype=np.float64)
        low, high = self.config.gripper_range
        velocity = dict(zip(state.name, state.velocity, strict=False))
        self._emit(
            "state",
            {
                "t": state.ts,
                "frame": "base",
                "joint_names": self._joint_names,
                "positions": [positions[n] for n in self._joint_names],
                "velocities": [velocity.get(n, 0.0) for n in self._joint_names],
                "ee_pose": pose_json(self._base_inv @ ee),
                "gripper_opening": (positions[self.config.gripper_joint] - low) / (high - low),
            },
            state.ts,
        )
        camera_topics: tuple[tuple[str, FeedbackKind], ...] = (
            (self.config.camera_optical_frame, "camera_pose"),
            (self.config.overview_optical_frame, "overview_camera_pose"),
        )
        for camera_frame, topic in camera_topics:
            camera = cameras.get(camera_frame)
            if camera is None or abs(time.time() - camera[2]) >= self.config.stale_s:
                continue
            parent, matrix, ts = camera
            world_camera: NDArray[np.float64] | None = matrix
            if parent == self.config.camera_mount_frame:
                world_camera = np.asarray(pose_to_matrix(frames[parent])) @ matrix
            elif parent != "world":
                world_camera = None
            if world_camera is not None:
                self._emit(
                    topic,
                    {
                        "t": ts,
                        "frame": "base",
                        **pose_json(np.asarray(self._base_inv @ world_camera, dtype=np.float64)),
                    },
                    ts,
                )
        # Static metadata is discoverable for late subscribers without repeating
        # it at the control-loop rate. Large URDFs are supplied as eval files.
        if now - self._last_info_publish >= 1.0:
            self._emit(
                "info",
                {
                    "joint_names": self._joint_names,
                    "joint_limits": self._limits,
                    "base_frame": self.config.model.base_link if self.config.model else "",
                    "ee_frame": self.config.ee_frame,
                    "primary_commands": ["delta", "gripper"],
                    "optional_commands": ["joints"],
                    "control_commands": ["stop"],
                    "gripper_opening": {"unit": "normalized", "closed": 0.0, "open": 1.0},
                    "max_delta_m": self.config.max_delta_m,
                    "max_delta_rad": self.config.max_delta_rad,
                    "max_joint_velocity_rad_s": self.config.max_joint_velocity,
                },
            )
            self._last_info_publish = now
        for _ in range(32):
            if not self._stop_commands.empty():
                return
            try:
                command = self._queue.get_nowait()
            except Empty:
                break
            try:
                self._accept(command, state, ee, now)
            except ValueError as exc:
                self._status(command, "rejected", str(exc))
            except Exception as exc:
                # A failed RPC may have delivered the command before its reply
                # was lost. Cancel rather than assuming no motion was started.
                try:
                    self._hold()
                except Exception:
                    logger.exception("Raw manipulation stop was not acknowledged")
                if self._active is not None:
                    self._status(self._active.command, "error", str(exc))
                    self._active = None
                self._status(command, "error", str(exc))
        self._advance(state, ee, now)
        if self._last_result and now - self._last_status_publish >= 1.0:
            self._emit("status", self._last_result)
            self._last_status_publish = now

    def _run(self) -> None:
        while not self._stop.wait(1.0 / self.config.rate_hz):
            try:
                self._tick()
            except Exception as exc:
                logger.exception("Raw manipulation update failed")
                if self._active:
                    try:
                        self._hold()
                    except Exception:
                        logger.exception("Raw manipulation stop was not acknowledged")
                    self._status(self._active.command, "error", str(exc))
                    self._active = None
                self._discard_pending("robot feedback or controller unavailable")
