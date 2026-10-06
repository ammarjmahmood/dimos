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

"""Independent capture-time robot filter upstream of voxel-map fusion."""

from __future__ import annotations

import asyncio
from typing import Any

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.pointcloud.robot_pointcloud_filter import RobotPointCloudFilter
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage


class RobotPointCloudFilterModuleConfig(ModuleConfig):
    model: RobotModelConfig
    world_frame: str = "world"


class RobotPointCloudFilterModule(Module):
    """Own one native robot scene; emit only captures aligned to state and TF."""

    config: RobotPointCloudFilterModuleConfig
    coordinator_joint_state: In[JointState]
    pointcloud: In[PointCloud2]
    filtered_pointcloud: Out[PointCloud2]
    tf: In[TFMessage]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        # Build only inside the deployed worker, never in the blueprint process.
        self._filter: RobotPointCloudFilter | None = None
        self._started = False

    def _initialize_filter(self) -> None:
        if self._filter is None:
            self._filter = RobotPointCloudFilter(self.config.model)

    @rpc
    def start(self) -> None:
        if self._started:
            return
        try:
            self._initialize_filter()
            # Subscribe to TF before the first camera capture arrives.
            _ = self.tfbuffer
            super().start()
            self.register_disposable(
                Disposable(self.coordinator_joint_state.subscribe(self._on_joint_state))
            )
            self.process_observable(self.pointcloud.pure_observable(), self._handle_pointcloud)
            self._started = True
        except BaseException:
            self.stop()
            raise

    @rpc
    def stop(self) -> None:
        if self._filter is not None:
            self._filter.close()
        super().stop()
        self._filter = None
        self._started = False

    def _on_joint_state(self, state: JointState) -> None:
        if self._filter is not None:
            self._filter.record_joint_state(state)

    async def _handle_pointcloud(self, cloud: PointCloud2) -> None:
        if self._filter is not None:
            await asyncio.to_thread(
                self._filter.publish,
                cloud,
                self.tfbuffer,
                self.filtered_pointcloud.publish,
                world_frame=self.config.world_frame,
            )
