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

from collections.abc import Iterator
from typing import Any
from uuid import uuid4

import numpy as np
from numpy.typing import NDArray
import pytest

from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.sim2.sensors.camera import module as camera_module
from dimos.sim2.sensors.camera.module import SimRGBDPointCloudCameraModule, depth_to_points
from dimos.sim2.sensors.reader import WorldReader
from dimos.sim2.sensors.spec import Camera

WIDTH, HEIGHT = 64, 48
FX = FY = 40.0
CX, CY = (WIDTH - 1) / 2, (HEIGHT - 1) / 2


def _info() -> CameraInfo:
    return CameraInfo.from_intrinsics(FX, FY, CX, CY, WIDTH, HEIGHT, frame_id="cam")


def test_flat_plane_one_metre_away_back_projects_to_z_one() -> None:
    depth = np.full((HEIGHT, WIDTH), 1.0, dtype=np.float32)

    points = depth_to_points(depth, _info())

    assert points.shape == (WIDTH * HEIGHT, 3)
    assert points.dtype == np.float32
    assert points[:, 2] == pytest.approx(np.ones(len(points)))
    # Pixel (u, v) lands at ((u - cx) / fx, (v - cy) / fy) one metre out, so the
    # top-left pixel sits up and to the left of the optical axis.
    assert points[0] == pytest.approx([-CX / FX, -CY / FY, 1.0])
    # The pixel nearest the principal point is on the optical axis.
    on_axis = np.argmin(np.abs(points[:, 0]) + np.abs(points[:, 1]))
    assert points[on_axis, :2] == pytest.approx([-0.5 / FX, -0.5 / FY])


def test_background_and_missing_depth_are_left_out() -> None:
    depth = np.full((HEIGHT, WIDTH), 0.8, dtype=np.float32)
    depth[0, :] = 0.0  # no return
    depth[1, :] = np.nan
    depth[2, :] = np.inf
    depth[3, :] = 50.0  # the renderer's far plane

    points = depth_to_points(depth, _info(), max_range=5.0)

    assert len(points) == (HEIGHT - 4) * WIDTH
    assert points[:, 2] == pytest.approx(np.full(len(points), 0.8))


def test_decimation_keeps_every_nth_pixel_at_its_own_position() -> None:
    depth = np.full((HEIGHT, WIDTH), 2.0, dtype=np.float32)

    full = depth_to_points(depth, _info())
    every_fourth = depth_to_points(depth, _info(), decimation=4)

    assert len(every_fourth) == (HEIGHT // 4) * (WIDTH // 4)
    kept = full.reshape(HEIGHT, WIDTH, 3)[::4, ::4].reshape(-1, 3)
    np.testing.assert_allclose(every_fourth, kept)


class _FlatRenderer:
    """Stands in for the MuJoCo renderer: a plane one metre ahead of the camera."""

    def __init__(self, model: Any, width: int, height: int) -> None:
        self.width, self.height = width, height

    def capture(
        self, data: Any, camera: int, depth: bool
    ) -> tuple[NDArray[np.uint8], NDArray[np.float32]]:
        rgb = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        return rgb, np.full((self.height, self.width), 1.0, dtype=np.float32)

    def close(self) -> None:
        pass


@pytest.fixture
def module() -> Iterator[SimRGBDPointCloudCameraModule]:
    sensor = Camera(
        "wrist_camera", camera="wrist_camera", width=WIDTH, height=HEIGHT, pointcloud=True
    )
    instance = SimRGBDPointCloudCameraModule(
        robot_id="arm", sensor=sensor, instance_name=f"test-camera-{uuid4().hex}"
    )
    try:
        yield instance
    finally:
        instance.stop()


def test_cloud_is_stamped_and_framed_like_the_depth_image(
    module: SimRGBDPointCloudCameraModule, mocker: Any
) -> None:
    mocker.patch.object(camera_module, "MujocoCamera", _FlatRenderer)
    model = mocker.Mock()
    model.camera.return_value.id = 0
    model.cam_fovy = np.array([60.0])
    data = mocker.Mock(cam_xpos=np.zeros((1, 3)), cam_xmat=np.eye(3).reshape(1, 9))
    module.reader = mocker.Mock(spec=WorldReader, model=model, data=data, timestamp=123.0)
    publish_depth = mocker.patch.object(module.depth_image, "publish")
    publish_cloud = mocker.patch.object(module.pointcloud, "publish")
    for port in ("color_image", "camera_info", "depth_camera_info", "tf"):
        mocker.patch.object(getattr(module, port), "publish")
    module.open()

    module.capture()

    depth_image = publish_depth.call_args.args[0]
    cloud = publish_cloud.call_args.args[0]
    assert cloud.frame_id == depth_image.frame_id == "arm/wrist_camera_optical"
    assert cloud.ts == depth_image.ts == 123.0
    points = cloud.points().numpy()
    assert len(points) == (HEIGHT // 2) * (WIDTH // 2)
    assert points[:, 2] == pytest.approx(np.ones(len(points)))
