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

from types import SimpleNamespace

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
    PoseWithCovariance,
    Quaternion,
)
from dimos_generated.nav_msgs.msg import Odometry
from dimos_generated.std_msgs.msg import Header

from dimos.robot.unitree.b1.connection import B1ConnectionModule


def test_b1_odometry_forwarding_preserves_generated_pose_and_exact_header():
    source = Odometry(
        header=Header(frame_id="map", stamp=Time(sec=1700000000, nanosec=123456789)),
        pose=PoseWithCovariance(
            pose=Pose(position=Point(x=1, y=2, z=3), orientation=Quaternion(w=1))
        ),
    )
    received = []
    proxy = SimpleNamespace(odom_pose=SimpleNamespace(publish=received.append))
    B1ConnectionModule._publish_odom_pose(proxy, source)
    result = PoseStamped.decode(received[0].encode())
    assert result.header == source.header
    assert result.pose == source.pose.pose
