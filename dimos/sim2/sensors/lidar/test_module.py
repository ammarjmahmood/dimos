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

from dataclasses import replace
from uuid import uuid4

import mujoco
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dimos.robot.unitree.g1.sim2 import G1_GROOT
from dimos.sim2.sensors.lidar.module import LidarModule
from dimos.sim2.sensors.reader import WorldReader
from dimos.sim2.sensors.spec import Lidar

pytestmark = pytest.mark.mujoco


@pytest.fixture
def module():
    sensor = next(s for s in G1_GROOT.sensors if isinstance(s, Lidar))
    instance = LidarModule(
        robot_id="g1", root_body="pelvis", sensor=sensor, instance_name=f"test-lidar-{uuid4().hex}"
    )
    try:
        yield instance
    finally:
        instance.stop()


@pytest.mark.parametrize("pitch", [0.0, -0.35, 0.35])
def test_g1_scan_excludes_ceiling_after_mount_rotation(pitch, module, mocker):
    sensor = replace(module.config.sensor, output_frame="world")
    module.config.sensor = sensor
    model = mujoco.MjModel.from_xml_string(f"""
        <mujoco>
          <compiler angle="radian"/>
          <worldbody>
            <geom type="plane" size="10 10 0.1"/>
            <geom type="box" pos="0 0 3" size="10 10 0.1"/>
            <body name="g1/pelvis" pos="0 0 1" euler="0 {pitch} 0">
              <geom type="sphere" size="0.1"/>
              <site name="g1/mid360_link" pos="0.0002835 0.00003 0.41618"
                    quat="0 0.9997985784932998 0 -0.020069938783589012"/>
            </body>
          </worldbody>
        </mujoco>
    """)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    module.reader = mocker.Mock(spec=WorldReader, model=model, data=data, timestamp=123.0)
    publish = mocker.patch.object(module.pointcloud, "publish")
    module.open()

    module.capture()

    publish.assert_called_once()
    cloud = publish.call_args.args[0]
    points = cloud.points().numpy()
    assert len(points) > 1000
    assert points[:, 2] == pytest.approx(np.zeros(len(points)), abs=1e-6)
    assert cloud.frame_id == "world"
    assert cloud.ts == 123.0

    module.config.sensor = replace(sensor, maximum_world_elevation=None)
    module.capture()
    unfiltered = publish.call_args.args[0].points().numpy()
    assert np.count_nonzero(unfiltered[:, 2] > 2.8) > 100


def test_sensor_frame_preserves_ray_origin_and_cloud_timestamp(module, mocker):
    model = mujoco.MjModel.from_xml_string("""
        <mujoco><compiler angle="radian"/><worldbody>
          <geom type="plane" size="10 10 0.1"/>
          <body name="g1/pelvis" pos="2 3 1" euler="0 0.3 0.6">
            <geom type="sphere" size="0.1"/>
            <site name="g1/mid360_link" pos="0.2 0 0.1" euler="3.14159265359 0 0"/>
          </body>
        </worldbody></mujoco>
    """)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    module.reader = mocker.Mock(spec=WorldReader, model=model, data=data, timestamp=456.0)
    publish = mocker.patch.object(module.pointcloud, "publish")
    publish_tf = mocker.patch.object(module.tf, "publish")
    module.open()
    module.capture()
    cloud = publish.call_args.args[0]
    transform = publish_tf.call_args.args[0].transforms[0]
    assert transform.frame_id == "world"
    assert cloud.frame_id == transform.child_frame_id == "g1/lidar"
    assert cloud.ts == transform.ts == 456.0
    origin = np.array(transform.translation.to_tuple())
    assert origin == pytest.approx(data.site_xpos[0])
    rotation = Rotation.from_quat(transform.rotation.to_tuple())
    world_points = rotation.apply(cloud.points().numpy()) + origin
    assert len(world_points) > 1000
    assert world_points[:, 2] == pytest.approx(np.zeros(len(world_points)), abs=1e-5)
