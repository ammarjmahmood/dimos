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

"""Typed RGB and RGB-D connections, with optical-frame transforms."""

import math

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.sim2.sensors.camera.renderers.mujoco import MujocoCamera
from dimos.sim2.sensors.module import SensorModule, SensorModuleConfig
from dimos.sim2.sensors.spec import Camera


class CameraModuleConfig(SensorModuleConfig):
    sensor: Camera


class SimCameraModule(SensorModule):
    config: CameraModuleConfig
    color_image: Out[Image]
    camera_info: Out[CameraInfo]
    tf: Out[TFMessage]

    def open(self) -> None:
        sensor = self.config.sensor
        self._renderer = MujocoCamera(self.reader.model, sensor.width, sensor.height)
        self._camera = self.reader.model.camera(f"{self.config.robot_id}/{sensor.model_name}").id
        self._frame = f"{self.config.robot_id}/{sensor.name}_optical"
        fovy = float(self.reader.model.cam_fovy[self._camera])
        self._focal = sensor.height / (2 * math.tan(math.radians(fovy) / 2))

    def capture(self) -> None:
        sensor = self.config.sensor
        rgb, depth = self._renderer.capture(self.reader.data, self._camera, sensor.depth)
        ts = self.reader.timestamp
        position = self.reader.data.cam_xpos[self._camera]
        # MuJoCo looks along -Z; optical frames look along +Z with Y down.
        rotation = self.reader.data.cam_xmat[self._camera].reshape(3, 3) @ np.diag([1, -1, -1])
        quat = Rotation.from_matrix(rotation).as_quat()
        self.tf.publish(
            TFMessage(
                Transform(
                    translation=Vector3(*position),
                    rotation=Quaternion(*quat),
                    frame_id="world",
                    child_frame_id=self._frame,
                    ts=ts,
                )
            )
        )
        info = CameraInfo.from_intrinsics(
            self._focal,
            self._focal,
            (sensor.width - 1) / 2,
            (sensor.height - 1) / 2,
            sensor.width,
            sensor.height,
            frame_id=self._frame,
        )
        info.ts = ts
        self.camera_info.publish(info)
        self.color_image.publish(
            Image(data=rgb, format=ImageFormat.RGB, frame_id=self._frame, ts=ts)
        )
        self.publish_depth(depth, info, ts)

    def publish_depth(self, depth: NDArray[np.float32] | None, info: CameraInfo, ts: float) -> None:
        pass

    def close_sensor(self) -> None:
        renderer = getattr(self, "_renderer", None)
        if renderer is not None:
            renderer.close()


class SimRGBDCameraModule(SimCameraModule):
    depth_image: Out[Image]
    depth_camera_info: Out[CameraInfo]

    def publish_depth(self, depth: NDArray[np.float32] | None, info: CameraInfo, ts: float) -> None:
        assert depth is not None
        self.depth_image.publish(
            Image(data=depth, format=ImageFormat.DEPTH, frame_id=self._frame, ts=ts)
        )
        self.depth_camera_info.publish(info)


class SimRGBDPointCloudCameraModule(SimRGBDCameraModule):
    """An RGB-D camera that also publishes each depth frame as a point cloud.

    The cloud is in the camera's optical frame and carries the depth frame's
    timestamp, so a ``world -> optical`` lookup at the cloud's stamp places it.
    """

    pointcloud: Out[PointCloud2]

    def publish_depth(self, depth: NDArray[np.float32] | None, info: CameraInfo, ts: float) -> None:
        super().publish_depth(depth, info, ts)
        assert depth is not None
        sensor = self.config.sensor
        points = depth_to_points(
            depth,
            info,
            decimation=sensor.pointcloud_decimation,
            max_range=sensor.pointcloud_max_range,
        )
        self.pointcloud.publish(PointCloud2.from_numpy(points, frame_id=self._frame, timestamp=ts))


def depth_to_points(
    depth: NDArray[np.float32],
    info: CameraInfo,
    *,
    decimation: int = 1,
    max_range: float = math.inf,
) -> NDArray[np.float32]:
    """Turn a depth image into 3D points in the camera's optical frame.

    Args:
        depth: One value per pixel, shape (height, width): the distance from the
            camera plane along the optical axis, in metres. A value of zero, NaN or
            infinity means the pixel saw nothing and is left out.
        info: The camera's pinhole intrinsics (focal lengths and principal point in
            pixels). Its width and height must match ``depth``.
        decimation: Keep every n-th row and column. 1 keeps every pixel.
        max_range: Leave out points farther than this along the optical axis, in
            metres. Pixels that see nothing come back at the renderer's far plane, so
            an infinite value here lets that background into the cloud.

    Returns:
        An (N, 3) float32 array of x, y, z in the optical frame: x right, y down,
        z forward, in metres.
    """
    k = info.get_K_matrix()
    fx, fy, cx, cy = k[0, 0], k[1, 1], k[0, 2], k[1, 2]
    z = np.asarray(depth, dtype=np.float32)[::decimation, ::decimation]
    rows, cols = np.indices(z.shape)
    keep = np.isfinite(z) & (z > 0) & (z <= max_range)
    z = z[keep]
    # Pinhole model: a pixel (u, v) at depth z sits at ((u - cx) / fx * z, (v - cy) / fy * z, z).
    x = (cols[keep] * decimation - cx) / fx * z
    y = (rows[keep] * decimation - cy) / fy * z
    return np.stack([x, y, z], axis=1).astype(np.float32)
