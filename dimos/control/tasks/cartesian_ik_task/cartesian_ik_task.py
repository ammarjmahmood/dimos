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

"""Absolute Cartesian pose leaf over the shared bounded Pink IK task."""

from __future__ import annotations

from collections.abc import Mapping
import threading
from typing import TYPE_CHECKING, Any

import attrs
import numpy as np

from dimos.control.task import CoordinatorState, JointCommandOutput
from dimos.control.tasks.pose_target_ik import (
    FrameTargetSnapshot,
    PinkPoseTargetSolver,
    PoseTargetIKTask,
    PoseTargetIKTaskConfig,
    PoseTargetIKTaskParams,
    string_tuple_converter,
)
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.transform_utils import matrix_to_pose, pose_to_matrix

if TYPE_CHECKING:
    from dimos.control.coordinator import TaskConfig
    from dimos.control.hardware_interface import ConnectedHardware, ConnectedWholeBody


@attrs.frozen(slots=False)
class CartesianIKTaskConfig(PoseTargetIKTaskConfig):
    """Configuration for one absolute Cartesian target frame."""

    target_frames: tuple[str, ...] = attrs.field(
        default=(),
        converter=string_tuple_converter,
        validator=[attrs.validators.min_len(1), attrs.validators.max_len(1)],
    )
    feedback_correction: bool = False


class CartesianIKTask(PoseTargetIKTask):
    """Track one stream of absolute poses with the shared Pink control core."""

    def __init__(
        self,
        name: str,
        config: CartesianIKTaskConfig,
        *,
        solver: PinkPoseTargetSolver | None = None,
    ) -> None:
        self._lock = threading.Lock()
        self._target_pose: PoseStamped | None = None
        self._last_update_time = 0.0
        self._active = False
        self._feedback_correction = config.feedback_correction
        self._commanded: JointState | None = None
        super().__init__(name, config, solver=solver)

    def is_active(self) -> bool:
        with self._lock:
            return self._active and self._target_pose is not None

    def on_cartesian_command(self, pose: Pose | PoseStamped, t_now: float) -> bool:
        """Accept an absolute target pose and activate tracking."""
        target = PoseStamped(
            ts=pose.ts if isinstance(pose, PoseStamped) else 0.0,
            frame_id=pose.frame_id if isinstance(pose, PoseStamped) else "",
            position=pose.position,
            orientation=pose.orientation,
        )
        with self._lock:
            self._target_pose = target
            self._last_update_time = t_now
            self._active = True
        return True

    def start(self) -> None:
        with self._lock:
            self._active = True

    def stop(self) -> None:
        with self._lock:
            self._active = False
            self._commanded = None
        self._reset_command_state()

    def clear(self) -> None:
        with self._lock:
            self._target_pose = None
            self._active = False
            self._commanded = None
        self._reset_command_state()

    def is_tracking(self) -> bool:
        return self.is_active()

    def cancel(self) -> bool:
        """Clear the target without requiring joint feedback or an IK update."""
        self.clear()
        return True

    def get_control_info(self) -> dict[str, Any]:
        """Describe the configured robot without exposing the model to input sources."""
        model = self._config.robot_model
        loaded = model.model.load()
        limits = []
        for name in self._joint_names:
            joint = loaded.get_joint(name)
            if joint is None or joint.lower is None or joint.upper is None:
                raise ValueError(f"Controlled joint {name!r} needs position limits")
            limits.append((joint.lower, joint.upper))
        return {
            "joint_names": self._joint_names,
            "joint_limits": limits,
            "base_frame": model.base_link,
            "base_pose": model.base_pose,
            "ee_frame": self._config.target_frames[0],
            "max_joint_velocity_rad_s": self._config.max_joint_velocity_rad_s,
        }

    def get_feedback(self, state: CoordinatorState) -> dict[str, Any]:
        """Measured state and FK from this task's existing solver, even while idle."""
        poses = self.current_frame_poses(state, self._config.target_frames)
        return {
            "t": state.joints.timestamp,
            "positions": state.joints.joint_positions,
            "velocities": state.joints.joint_velocities,
            "ee_pose": None if poses is None else poses[self._config.target_frames[0]],
            "tracking": self.is_tracking(),
        }

    def _frame_target_snapshot(self, state: CoordinatorState) -> FrameTargetSnapshot | None:
        with self._lock:
            if not self._active or self._target_pose is None:
                return None
            target, commanded, updated = self._target_pose, self._commanded, self._last_update_time
        frame = self._config.target_frames[0]
        if self._feedback_correction and commanded is not None:
            measured = self.current_frame_poses(state, self._config.target_frames)
            if measured is not None:
                measured_matrix = pose_to_matrix(measured[frame])
                commanded_matrix = pose_to_matrix(
                    self._solver.frame_poses(commanded, self._config.target_frames)[frame]
                )
                corrected = np.array(pose_to_matrix(target), copy=True)
                corrected[:3, 3] += commanded_matrix[:3, 3] - measured_matrix[:3, 3]
                corrected[:3, :3] = (
                    corrected[:3, :3] @ measured_matrix[:3, :3].T @ commanded_matrix[:3, :3]
                )
                pose = matrix_to_pose(corrected)
                target = PoseStamped(
                    position=pose.position, orientation=pose.orientation, frame_id=target.frame_id
                )
        return FrameTargetSnapshot(targets={frame: target}, last_update_time=updated)

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        output = super().compute(state)
        if not self._feedback_correction:
            return output
        with self._lock:
            self._commanded = (
                JointState(name=output.joint_names, position=output.positions)
                if output is not None and output.positions is not None
                else None
            )
        return output

    def _on_target_timeout(self) -> None:
        self.clear()

    def _on_pose_target_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        self.clear()


class CartesianIKTaskParams(PoseTargetIKTaskParams):
    """Task-owned parameters carried inside the generic task envelope."""

    target_frame: str
    feedback_correction: bool = False


def create_task(
    cfg: TaskConfig,
    hardware: Mapping[str, ConnectedHardware | ConnectedWholeBody],
) -> CartesianIKTask:
    """Create an absolute Cartesian Pink task from a registry configuration."""
    params = CartesianIKTaskParams.model_validate(cfg.params)
    return CartesianIKTask(
        cfg.name,
        CartesianIKTaskConfig(
            joint_names=tuple(cfg.joint_names),
            robot_model=params.robot_model,
            target_frames=(params.target_frame,),
            feedback_correction=params.feedback_correction,
            pink=params.pink,
            priority=cfg.priority,
            timeout=params.timeout,
            max_joint_velocity_rad_s=params.max_joint_velocity_rad_s,
            joint_velocity_limits_rad_s=params.joint_velocity_limits_rad_s,
            joint_command_filter_cutoff_hz=params.joint_command_filter_cutoff_hz,
            max_command_tracking_error_deg=params.max_command_tracking_error_deg,
            feedback_limit_tolerance=params.feedback_limit_tolerance,
            command_limit_margin=params.command_limit_margin,
        ),
    )
