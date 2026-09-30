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

import math
import sys
from types import SimpleNamespace
from unittest.mock import MagicMock

from dimos_generated.std_msgs.msg import Header
import numpy as np

from dimos.mapping.loop_closure.pgo import Keyframe, PGOConfig, PoseGraph, _KeyPose, _PGOState
from dimos.memory.type.observation import Observation
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.pointcloud import pointcloud_from_xyz, pointcloud_xyz
from dimos.msgs.time import time_from_seconds


def test_generated_observation_pose_correction_applies_rotation_after_translation():
    local = Transform(ts=1.0)
    optimized = Transform(
        translation=Vector3(5.0, 0.0, 0.0),
        rotation=Quaternion(0, 0, math.sqrt(0.5), math.sqrt(0.5)),
        ts=1.0,
    )
    graph = PoseGraph(keyframes=(Keyframe(ts=1.0, local=local, optimized=optimized),))
    source = Observation(id=1, ts=1.0, _data="payload", pose=(1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0))
    corrected = next(graph(iter([source])))
    assert corrected.data == "payload" and corrected.ts == source.ts
    np.testing.assert_allclose(
        corrected.pose_tuple, (3.0, 1.0, 0.0, 0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)), atol=1e-9
    )
    assert source.pose_tuple == (1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0)


def test_pose_graph_retains_poseless_observations_without_constructing_correction():
    source = Observation(id=1, ts=1.0, _data="payload")
    assert list(PoseGraph()(iter([source]))) == [source]


def test_pgo_submap_places_generated_body_clouds_before_merging(monkeypatch):
    class MatrixPose:
        def __init__(self, x):
            self.value = np.eye(4)
            self.value[0, 3] = x

        def matrix(self):
            return self.value

    solver = SimpleNamespace(
        **{
            name: MagicMock(name=name)
            for name in (
                "Pose3",
                "ISAM2Params",
                "ISAM2",
                "NonlinearFactorGraph",
                "Values",
            )
        }
    )
    monkeypatch.setitem(sys.modules, "gtsam", solver)
    state = _PGOState(PGOConfig(submap_resolution=0.2))
    solver.ISAM2Params.return_value.setRelinearizeThreshold.assert_called_once_with(0.01)
    assert solver.ISAM2Params.return_value.relinearizeSkip == 1
    clouds = [
        pointcloud_from_xyz(
            np.array([[1.0, 0.0, 0.0]]), header=Header(frame_id="body", stamp=time_from_seconds(ts))
        )
        for ts in (1.0, 2.0)
    ]
    state._key_poses = [
        _KeyPose(local=MatrixPose(0), optimized=MatrixPose(offset), timestamp=ts, body_cloud=cloud)
        for cloud, offset, ts in zip(clouds, (2.0, 4.0), (1.0, 2.0), strict=True)
    ]
    before = [cloud.encode() for cloud in clouds]
    result = state._get_submap(0, 1)
    np.testing.assert_allclose(pointcloud_xyz(result), [[3.0, 0.0, 0.0], [5.0, 0.0, 0.0]])
    assert result.header.frame_id == "world_corrected"
    assert result.header.stamp == time_from_seconds(2.0)
    assert [cloud.encode() for cloud in clouds] == before
