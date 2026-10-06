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

"""Capture alignment and native robot-surface exclusion before mapping."""

from __future__ import annotations

from collections.abc import Callable
from threading import RLock
from typing import TYPE_CHECKING

import numpy as np

from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.protocol.tf.tf import MultiTBuffer
from dimos.types.timestamped import TimestampedBufferCollection
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.manipulation.planning.spec.protocols import WorldSpec

logger = setup_logger()

# Capture policies are internal; delayed data never authorizes latest-state fallback.
_MATCH_TOLERANCE_S = 0.1
_TF_WAIT_TIMEOUT_S = 0.1
_STATE_HISTORY_S = 5.0


class RobotPointCloudFilter:
    """WorldMonitor's capture history and robot-surface exclusion.

    This component owns no robot geometry. The world classifies points using a
    consumer context; this lock preserves capture and publication order.
    """

    def __init__(self, world: WorldSpec) -> None:
        self._world = world
        self._lock = RLock()
        self._states = TimestampedBufferCollection[JointState](_STATE_HISTORY_S)
        self._last_stamp: float | None = None

    def record_joint_state(self, state: JointState) -> None:
        """Store an already validated canonical state at its original timestamp."""
        with self._lock:
            self._states.remove_by_timestamp(state.ts)
            self._states.add(JointState(state))
            latest = self._states.last()
            if latest is not None:
                self._states.prune_old(latest.ts - _STATE_HISTORY_S)

    def publish(
        self,
        cloud: PointCloud2,
        tfbuffer: MultiTBuffer,
        output: Callable[[PointCloud2], None],
        *,
        world_frame: str = "world",
    ) -> None:
        """Publish aligned captures in order while preserving point fields."""
        with self._lock:
            filtered = self.filter(cloud, tfbuffer, world_frame=world_frame)
            if filtered is not None:
                output(filtered)

    def filter(
        self, cloud: PointCloud2, tfbuffer: MultiTBuffer, *, world_frame: str = "world"
    ) -> PointCloud2 | None:
        """Drop unaligned captures; never substitute the world's latest state."""
        with self._lock:
            if not np.isfinite(cloud.ts) or (
                self._last_stamp is not None and cloud.ts < self._last_stamp
            ):
                logger.warning("Dropping point cloud: invalid or out-of-order timestamp")
                return None
            tolerance = _MATCH_TOLERANCE_S
            state = self._states.find_closest(cloud.ts, tolerance)
            if state is None:
                logger.warning("Dropping point cloud: capture-time joint state unavailable")
                return None
            world_from_sensor = tfbuffer.get(
                world_frame,
                cloud.frame_id,
                time_point=cloud.ts,
                time_tolerance=tolerance,
                forward_tolerance=_TF_WAIT_TIMEOUT_S,
            )
            if world_from_sensor is None:
                logger.warning("Dropping point cloud: capture-time sensor TF unavailable")
                return None
            points = cloud.points_f32()
            if not np.isfinite(points).all():
                logger.warning("Dropping point cloud: non-finite points")
                return None
            transform = world_from_sensor.to_matrix()
            world_points = np.asarray(points @ transform[:3, :3].T + transform[:3, 3])
            with self._world.scratch_context() as ctx:
                self._world.set_joint_state(ctx, state)
                keep = ~self._world.robot_body_mask(ctx, world_points)
            intensities = cloud.intensities_f32()
            filtered = PointCloud2.from_numpy(
                points[keep],
                frame_id=cloud.frame_id,
                timestamp=cloud.ts,
                intensities=intensities[keep] if intensities is not None else None,
            )
            for name, values in cloud.pointcloud_tensor.point.items():
                if name not in ("positions", "intensities"):
                    filtered.pointcloud_tensor.point[name] = values[keep]
            self._last_stamp = cloud.ts
            return filtered
