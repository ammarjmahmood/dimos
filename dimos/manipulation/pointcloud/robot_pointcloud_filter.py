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
from functools import partial
from threading import RLock

import numpy as np
import roboplan.core as roboplan_core

from dimos.manipulation.planning.groups.registry import PlanningGroupRegistry
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.joint_space import CoordinateTopology
from dimos.manipulation.planning.spec.measured_joint_state import canonicalize_measured_joint_state
from dimos.manipulation.planning.spec.validation import prepare_robot_model
from dimos.manipulation.planning.world.roboplan_model import build_roboplan_model
from dimos.manipulation.planning.world.roboplan_scene import create_roboplan_scene
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.protocol.tf.tf import MultiTBuffer
from dimos.types.timestamped import TimestampedBufferCollection
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Capture policies are internal; delayed data never authorizes latest-state fallback.
_MATCH_TOLERANCE_S = 0.1
_TF_WAIT_TIMEOUT_S = 0.1
_STATE_HISTORY_S = 5.0


class RobotPointCloudFilter:
    """Own a robot-only native scene and capture history, independent of planning.

    Geometry is loaded once from the canonical prepared model. All classification
    uses upstream RobotBodyFilter; no planning world or RPC is involved.
    """

    def __init__(self, config: RobotModelConfig) -> None:
        self._prepared = prepare_robot_model(config)
        self._model = build_roboplan_model(
            self._prepared,
            PlanningGroupRegistry(config.planning_groups),
            partial(create_roboplan_scene, roboplan_core),
        )
        self._context = roboplan_core.SceneContext(self._model.scene)
        self._body_filter = roboplan_core.RobotBodyFilter(
            self._model.scene,
            roboplan_core.RobotBodyFilterOptions(
                padding=0.01,
                method=roboplan_core.RobotBodyFilterMethod.Narrowphase,
                num_threads=1,
            ),
        )
        self._circle_names = {
            coordinate.name
            for coordinate in self._prepared.joint_space.coordinates
            if coordinate.topology is CoordinateTopology.CIRCLE
        }
        self._closed = False
        self._lock = RLock()
        self._states = TimestampedBufferCollection[JointState](_STATE_HISTORY_S)
        self._last_stamp: float | None = None

    def record_joint_state(self, state: JointState) -> None:
        """Capture complete canonical positions from the coordinator stream."""
        try:
            state = canonicalize_measured_joint_state(state, self._prepared.config)
        except ValueError as error:
            logger.warning("Dropping invalid capture joint state", error=str(error))
            return
        with self._lock:
            if self._closed:
                return
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
            if self._closed:
                return
            filtered = self.filter(cloud, tfbuffer, world_frame=world_frame)
            if filtered is not None:
                output(filtered)

    def filter(
        self, cloud: PointCloud2, tfbuffer: MultiTBuffer, *, world_frame: str = "world"
    ) -> PointCloud2 | None:
        """Drop unaligned captures; never substitute the world's latest state."""
        with self._lock:
            if self._closed:
                return None
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
            positions = dict(zip(state.name, state.position, strict=True))
            group = self._model.all_group
            native_positions: list[float] = []
            for name in group.public_names:
                value = positions[name]
                if name in self._circle_names:
                    native_positions.extend((float(np.cos(value)), float(np.sin(value))))
                else:
                    native_positions.append(value)
            q = self._context.toFullJointPositions(group.name, np.asarray(native_positions))
            keep = ~np.asarray(self._body_filter.computeMask(q, world_points), dtype=bool)
            filtered = PointCloud2(frame_id=cloud.frame_id, ts=cloud.ts, seq=cloud.seq)
            for name, values in cloud.pointcloud_tensor.point.items():
                filtered.pointcloud_tensor.point[name] = values[keep]
            self._last_stamp = cloud.ts
            return filtered

    def close(self) -> None:
        """Wait for any publication and prevent further output during shutdown."""
        with self._lock:
            self._closed = True
