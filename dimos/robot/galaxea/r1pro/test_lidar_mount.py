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

import xml.etree.ElementTree as ElementTree

import pytest

from dimos.robot.galaxea.r1pro.config import R1PRO_MODEL
from dimos.robot.galaxea.r1pro.connection import LIDAR_MOUNT_XYZ


def test_the_lidar_mount_matches_the_urdf_joint() -> None:
    """The hardcoded base_link -> lidar edge must agree with the URDF, or two tf paths to the lidar disagree."""
    joint = next(
        joint
        for joint in ElementTree.fromstring(R1PRO_MODEL.load().xml).iter("joint")
        if joint.attrib["name"] == "lidar_chassis_left_joint"
    )
    origin = joint.find("origin")
    assert origin is not None
    assert LIDAR_MOUNT_XYZ == pytest.approx(tuple(float(v) for v in origin.attrib["xyz"].split()))
