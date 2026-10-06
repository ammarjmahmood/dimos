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

"""Capture-time adaptation for upstream robot surface filtering."""

from __future__ import annotations

import asyncio
import importlib
from threading import RLock
import xml.etree.ElementTree as ET

import numpy as np
from numpy.typing import NDArray
from pydantic import Field

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.assets.model import RobotModel
from dimos.types.timestamped import TimestampedBufferCollection
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import matrix_to_pose

logger = setup_logger()


class PointCloudSelfFilterConfig(ModuleConfig):
    model: RobotModel
    padding_m: float = Field(default=0.01, ge=0.0)
    tf_tolerance_s: float = Field(default=0.02, ge=0.0)
    tf_forward_tolerance_s: float = Field(default=0.05, ge=0.0)
    state_tolerance_s: float = Field(default=0.02, ge=0.0)
    state_history_s: float = Field(default=5.0, gt=0.0)


class PointCloudSelfFilter(Module):
    """Remove robot surface returns before voxel fusion using upstream geometry."""

    config: PointCloudSelfFilterConfig

    pointcloud: In[PointCloud2]
    tf: In[TFMessage]
    coordinator_joint_state: In[JointState]
    filtered_pointcloud: Out[PointCloud2]

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._filter_lock = RLock()
        self._states = TimestampedBufferCollection[JointState](self.config.state_history_s)
        self._load_scene()
        self._last_capture: float | None = None

    @rpc
    def start(self) -> None:
        self.tfbuffer  # noqa: B018 - Initialize TF before binding input handlers.
        super().start()

    async def handle_pointcloud(self, cloud: PointCloud2) -> None:
        """Filter the latest capture without starving TF transport callbacks."""
        await asyncio.to_thread(self._on_pointcloud, cloud)

    async def handle_coordinator_joint_state(self, state: JointState) -> None:
        await asyncio.to_thread(self.add_joint_state, state)

    def add_joint_state(self, state: JointState) -> None:
        """Buffer full model state; out-of-order arrivals remain timestamped."""
        with self._filter_lock:
            if np.isfinite(state.ts):
                self._states.remove_by_timestamp(state.ts)
                self._states.add(JointState(state))
                latest = self._states.last()
                if latest is not None:
                    self._states.prune_old(latest.ts - self.config.state_history_s)

    def filter_cloud(self, cloud: PointCloud2) -> PointCloud2 | None:
        """Filter one aligned capture without changing its frame or point fields."""
        with self._filter_lock:
            if not np.isfinite(cloud.ts) or (
                self._last_capture is not None and cloud.ts < self._last_capture
            ):
                logger.warning("Dropping cloud: invalid or out-of-order capture timestamp")
                return None
            base_from_sensor = self.tfbuffer.get(
                self._base_link,
                cloud.frame_id,
                time_point=cloud.ts,
                time_tolerance=self.config.tf_tolerance_s,
                forward_tolerance=self.config.tf_forward_tolerance_s,
            )
            q = self._capture_configuration(cloud.ts)
            if base_from_sensor is None or q is None:
                logger.warning("Dropping cloud: capture-time robot state or TF unavailable")
                return None
            points = cloud.points_f32()
            if not np.isfinite(points).all():
                return None
            # This Scene is rooted at the URDF base, so its world is the base frame.
            transform = base_from_sensor.to_matrix()
            base_points = np.asarray(
                points @ transform[:3, :3].T + transform[:3, 3], dtype=np.float64
            )
            keep = ~np.asarray(self._body_filter.computeMask(q, base_points), dtype=bool)
            filtered = PointCloud2(frame_id=cloud.frame_id, ts=cloud.ts, seq=cloud.seq)
            for name, values in cloud.pointcloud_tensor.point.items():
                filtered.pointcloud_tensor.point[name] = values[keep]
            self._last_capture = cloud.ts
            return filtered

    def _on_pointcloud(self, cloud: PointCloud2) -> None:
        with self._filter_lock:
            filtered = self.filter_cloud(cloud)
            if filtered is not None:
                self.filtered_pointcloud.publish(filtered)

    def _capture_configuration(self, stamp: float) -> NDArray[np.float64] | None:
        state = self._states.find_closest(stamp, self.config.state_tolerance_s)
        positions: dict[str, float] = {}
        if state is not None:
            if len(state.name) != len(state.position) or len(set(state.name)) != len(state.name):
                return None
            positions = dict(zip(state.name, state.position, strict=True))
            if not np.isfinite(list(positions.values())).all():
                return None
        # Start from the model's neutral configuration, never its latest state.
        q = self._neutral_q.copy()
        for name in self._scene.getJointNames():
            info = self._scene.getJointInfo(name)
            if info.num_velocity_dofs == 1:
                if name not in positions:
                    return None
                value = positions[name]
                values = [np.cos(value), np.sin(value)] if info.num_position_dofs == 2 else [value]
            else:
                # Multi-DOF joints have no scalar JointState representation.
                # Recover their configuration from capture-time relative TF.
                joint = self._joints[name]
                parent_from_child = self.tfbuffer.get(
                    joint.parent_link,
                    joint.child_link,
                    time_point=stamp,
                    time_tolerance=self.config.tf_tolerance_s,
                    forward_tolerance=self.config.tf_forward_tolerance_s,
                )
                if parent_from_child is None:
                    return None
                origin = np.asarray(
                    self._context.forwardKinematics(
                        self._neutral_q, joint.child_link, joint.parent_link
                    )
                )
                motion = np.linalg.inv(origin) @ parent_from_child.to_matrix()
                pose = matrix_to_pose(motion)
                if info.num_position_dofs == 4:
                    if not np.allclose(motion[2], [0, 0, 1, 0], atol=1e-6):
                        return None
                    angle = np.arctan2(motion[1, 0], motion[0, 0])
                    values = [motion[0, 3], motion[1, 3], np.cos(angle), np.sin(angle)]
                elif info.num_position_dofs == 7:
                    values = [
                        *motion[:3, 3],
                        pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z,
                        pose.orientation.w,
                    ]
                else:
                    raise ValueError(f"Unsupported robot joint layout: {name}")
            q[self._scene.getJointPositionIndices([name])] = values
        return q

    def _load_scene(self) -> None:
        # RoboPlan is optional for stacks that do not use this module.
        native = importlib.import_module("roboplan.core")

        description = self.config.model.load()
        root = ET.fromstring(description.xml)
        root.set("version", "1.0")
        self._scene = native.Scene(
            "dimos_self_filter",
            native.loadUrdfSceneDescriptionFromXml(
                ET.tostring(root, encoding="unicode"),
                [str(path) for path in description.package_paths.values()],
            ),
        )
        self._neutral_q = np.asarray(self._scene.getCurrentJointPositions()).copy()
        self._context = native.SceneContext(self._scene)
        self._body_filter = native.RobotBodyFilter(
            self._scene,
            native.RobotBodyFilterOptions(
                padding=self.config.padding_m,
                method=native.RobotBodyFilterMethod.Narrowphase,
                num_threads=1,
            ),
        )
        # Reuse the asset loader's topology for multi-DOF TF lookup, not geometry parsing.
        self._base_link = description.root_link
        self._joints = {joint.name: joint for joint in description.joints}
