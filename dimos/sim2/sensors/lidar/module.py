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

"""Ideal scans in a declared frame, with a timestamp-matched sensor transform."""

import numpy as np
from pydantic import InstanceOf
from scipy.spatial.transform import Rotation

from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.sim2.sensors.lidar.raycast import Raycaster
from dimos.sim2.sensors.module import SensorModule, SensorModuleConfig
from dimos.sim2.sensors.spec import Lidar


class LidarModuleConfig(SensorModuleConfig):
    sensor: InstanceOf[Lidar]
    root_body: str


class LidarModule(SensorModule):
    config: LidarModuleConfig
    pointcloud: Out[PointCloud2]
    tf: Out[TFMessage]

    def open(self) -> None:
        robot = self.config.robot_id
        self._site = self.reader.model.site(f"{robot}/{self.config.sensor.model_name}").id
        self._frame = f"{robot}/{self.config.sensor.name}"
        self._rays = self.config.sensor.model.directions()
        self._raycaster = Raycaster(
            self.reader.model,
            self.reader.model.body(f"{robot}/{self.config.root_body}").id,
        )

    def capture(self) -> None:
        data = self.reader.data
        origin = data.site_xpos[self._site]
        rotation = data.site_xmat[self._site].reshape(3, 3)
        rays = self._rays @ rotation.T
        elevation_limit = self.config.sensor.maximum_world_elevation
        if elevation_limit is not None:
            rays = rays[rays[:, 2] <= np.sin(np.deg2rad(elevation_limit)) + 1e-12]
        points = self._raycaster.cast(
            data,
            origin,
            rays,
            self.config.sensor.model.min_range,
            self.config.sensor.model.max_range,
        )
        frame = "world"
        if self.config.sensor.output_frame == "sensor":
            points = (points - origin) @ rotation
            frame = self._frame
            self.tf.publish(
                TFMessage(
                    Transform(
                        translation=Vector3(*origin),
                        rotation=Quaternion(*Rotation.from_matrix(rotation).as_quat()),
                        frame_id="world",
                        child_frame_id=frame,
                        ts=self.reader.timestamp,
                    )
                )
            )
        self.pointcloud.publish(
            PointCloud2.from_numpy(
                points.astype(np.float32),
                frame_id=frame,
                timestamp=self.reader.timestamp,
            )
        )
