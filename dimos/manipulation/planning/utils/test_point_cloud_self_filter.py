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

from __future__ import annotations

from collections.abc import Callable, Iterator
from pathlib import Path
import platform
import sys
from typing import Any, cast

import numpy as np
from open3d.core import Tensor
import pytest
import trimesh

# The tests extra excludes yourdfpy on Linux ARM because embreex has no wheel.
if sys.platform == "linux" and platform.machine() == "aarch64":
    pytest.importorskip(
        "yourdfpy", reason="yourdfpy is unavailable in the Linux ARM test environment"
    )

from dimos.manipulation.planning.utils.point_cloud_self_filter import PointCloudSelfFilter
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.protocol.tf.tf import MultiTBuffer
from dimos.robot.assets.model import RobotModel

# A 20cm cube on `arm`, so the filter has real volume to exclude and the clear
# mask has real cells to name, with no external assets.
_URDF = """<?xml version="1.0"?>
<robot name="box_robot">
  <link name="base"/>
  <link name="arm">
    <collision><geometry><box size="0.2 0.2 0.2"/></geometry></collision>
  </link>
  <joint name="shoulder" type="fixed">
    <parent link="base"/><child link="arm"/>
  </joint>
</robot>
"""


@pytest.fixture
def make_filter(tmp_path: Path) -> Iterator[Callable[..., PointCloudSelfFilter]]:
    urdf = tmp_path / "robot.urdf"
    urdf.write_text(_URDF)
    modules: list[PointCloudSelfFilter] = []

    def make(**overrides: Any) -> PointCloudSelfFilter:
        urdf.write_text(overrides.pop("urdf_xml", _URDF))
        settings: dict[str, Any] = {
            "model": RobotModel.from_file(urdf),
            "padding_m": 0.01,
            "voxel_size": 0.05,
            "tf_tolerance_s": 0.001,
            "tf_forward_tolerance_s": 0.0,
        }
        settings.update(overrides)
        module = PointCloudSelfFilter(**settings)
        cast("dict[str, Any]", module.__dict__)["_tf"] = MultiTBuffer()
        modules.append(module)
        return module

    yield make
    for module in modules:
        cast("dict[str, Any]", module.__dict__)["_tf"] = None
        module.dispose()


def _place_arm(module: PointCloudSelfFilter, at: tuple[float, float, float], ts: float) -> None:
    """Move the rigid robot base at capture time in both frames."""
    for parent in ("camera", "world"):
        module.tfbuffer.receive_transform(
            Transform(
                translation=Vector3(*at),
                frame_id=parent,
                child_frame_id="base",
                ts=ts,
            )
        )


def _cloud(points: list[list[float]], ts: float = 1.0) -> PointCloud2:
    return PointCloud2.from_numpy(
        np.asarray(points, dtype=np.float32).reshape((-1, 3)),
        frame_id="camera",
        timestamp=ts,
    )


def _keys(cloud: PointCloud2, voxel_size: float) -> set[tuple[int, int, int]]:
    points = cloud.points_f32()
    if not len(points):
        return set()
    return {tuple(k) for k in np.floor(points / voxel_size).astype(int).tolist()}


def test_points_on_the_robot_are_dropped_and_the_rest_survive(
    make_filter: Callable[..., PointCloudSelfFilter],
) -> None:
    module = make_filter()
    _place_arm(module, (0.0, 0.0, 0.0), 1.0)

    result = module.filter_cloud(_cloud([[0.0, 0.0, 0.0], [0.05, 0.0, 0.0], [2.0, 0.0, 0.0]]))

    assert result is not None
    filtered, _ = result
    np.testing.assert_allclose(filtered.points_f32(), [[2.0, 0.0, 0.0]], atol=1e-6)


def test_the_mask_also_covers_where_the_robot_just_was(
    make_filter: Callable[..., PointCloudSelfFilter],
) -> None:
    # This is the whole point of the mask. Ray tracing cannot clear the volume
    # a link has vacated, because the link occluded it while it was there, so
    # the ghost stays until the filter names those cells itself.
    module = make_filter()

    _place_arm(module, (1.0, 0.0, 0.0), 1.0)
    first = module.filter_cloud(_cloud([], ts=1.0))
    assert first is not None

    _place_arm(module, (2.0, 0.0, 0.0), 2.0)
    second = module.filter_cloud(_cloud([], ts=2.0))
    assert second is not None

    keys = _keys(second[1], 0.05)
    assert (20, 0, 0) in keys, "the cell the arm vacated must still be cleared"
    assert (40, 0, 0) in keys, "and the cell it moved into"

    _place_arm(module, (2.0, 0.0, 0.0), 3.0)
    third = module.filter_cloud(_cloud([], ts=3.0))
    assert third is not None
    assert (20, 0, 0) not in _keys(third[1], 0.05), "a stale cell is cleared once, not forever"


def test_the_mask_quantizes_the_way_the_mapper_does(
    make_filter: Callable[..., PointCloudSelfFilter],
) -> None:
    # The crate floors world coordinates by voxel_size and emits cell centers.
    # A mask built any other way names cells the map does not hold.
    module = make_filter()
    _place_arm(module, (1.0, 0.0, 0.0), 1.0)

    result = module.filter_cloud(_cloud([]))

    assert result is not None
    points = result[1].points_f32()
    offsets = points / 0.05 - np.floor(points / 0.05)
    np.testing.assert_allclose(offsets, 0.5, atol=1e-5)


def test_a_cloud_without_capture_time_tf_is_dropped_whole(
    make_filter: Callable[..., PointCloudSelfFilter],
) -> None:
    # Half-filtering would let the arm into the map. Better to lose the frame.
    module = make_filter()

    assert module.filter_cloud(_cloud([[2.0, 0.0, 0.0]])) is None


def _joint_robot(kind: str) -> str:
    return _URDF.replace('name="shoulder" type="fixed"', f'name="shoulder" type="{kind}"').replace(
        '<parent link="base"/><child link="arm"/>',
        '<parent link="base"/><child link="arm"/><axis xyz="1 0 0"/>'
        '<limit lower="-3" upper="3" effort="1" velocity="1"/>',
    )


def test_capture_state_is_matched_by_timestamp_instead_of_latest(make_filter):
    module = make_filter(urdf_xml=_joint_robot("prismatic"), state_tolerance_s=0.001)
    _place_arm(module, (0, 0, 0), 1.0)
    module.add_joint_state(JointState(ts=2.0, name=["shoulder"], position=[2.0]))
    module.add_joint_state(JointState(ts=1.0, name=["shoulder"], position=[0.5]))

    result = module.filter_cloud(_cloud([[0.5, 0, 0], [2, 0, 0]], ts=1.0))

    assert result is not None
    np.testing.assert_allclose(result[0].points_f32(), [[2, 0, 0]])
    assert (10, 0, 0) in _keys(result[1], 0.05)


@pytest.mark.parametrize(
    "state",
    [
        None,
        JointState(ts=2.0, name=["shoulder"], position=[0.5]),
        JointState(ts=1.0, name=["wrong"], position=[0.5]),
        JointState(ts=1.0, name=["shoulder"], position=[float("nan")]),
        JointState(ts=1.0, name=["shoulder", "shoulder"], position=[0.5, 0.5]),
    ],
)
def test_missing_late_or_invalid_state_drops_the_capture(make_filter, state):
    module = make_filter(urdf_xml=_joint_robot("prismatic"), state_tolerance_s=0.001)
    _place_arm(module, (0, 0, 0), 1.0)
    if state is not None:
        module.add_joint_state(state)

    assert module.filter_cloud(_cloud([[0.5, 0, 0]])) is None


def test_failed_capture_does_not_replace_previous_clear_volume(make_filter):
    module = make_filter()
    _place_arm(module, (1, 0, 0), 1.0)
    assert module.filter_cloud(_cloud([], ts=1.0)) is not None
    assert module.filter_cloud(_cloud([], ts=2.0)) is None
    _place_arm(module, (2, 0, 0), 3.0)

    result = module.filter_cloud(_cloud([], ts=3.0))

    assert result is not None
    assert {(20, 0, 0), (40, 0, 0)} <= _keys(result[1], 0.05)
    assert module.filter_cloud(_cloud([], ts=1.0)) is None


def test_continuous_joint_uses_cos_sin_configuration(make_filter):
    urdf = _joint_robot("continuous").replace('<axis xyz="1 0 0"/>', '<axis xyz="0 0 1"/>')
    urdf = urdf.replace("<collision><geometry>", '<collision><origin xyz="0.5 0 0"/><geometry>')
    module = make_filter(urdf_xml=urdf)
    _place_arm(module, (0, 0, 0), 1.0)
    module.add_joint_state(JointState(ts=1.0, name=["shoulder"], position=[np.pi / 2]))

    result = module.filter_cloud(_cloud([[0, 0.5, 0], [0.5, 0, 0]]))

    assert result is not None
    np.testing.assert_allclose(result[0].points_f32(), [[0.5, 0, 0]])


def test_mimic_joint_is_derived_from_its_source(make_filter):
    urdf = _joint_robot("prismatic").replace(
        "</robot>",
        '<link name="replica"><collision><geometry><sphere radius="0.05"/>'
        "</geometry></collision></link>"
        '<joint name="follower" type="prismatic"><parent link="arm"/><child link="replica"/>'
        '<axis xyz="1 0 0"/><limit lower="-3" upper="3" effort="1" velocity="1"/>'
        '<mimic joint="shoulder" multiplier="2" offset="0.1"/></joint></robot>',
    )
    module = make_filter(urdf_xml=urdf)
    _place_arm(module, (0, 0, 0), 1.0)
    module.add_joint_state(JointState(ts=1.0, name=["shoulder"], position=[0.2]))

    result = module.filter_cloud(_cloud([[0.2, 0, 0], [0.7, 0, 0], [1, 0, 0]]))

    assert result is not None
    np.testing.assert_allclose(result[0].points_f32(), [[1, 0, 0]])


@pytest.mark.parametrize("kind", ["planar", "floating"])
def test_multidof_joints_use_capture_time_relative_tf(make_filter, kind):
    module = make_filter(urdf_xml=_joint_robot(kind))
    _place_arm(module, (0, 0, 0), 1.0)
    assert module.filter_cloud(_cloud([[0.5, 0, 0]])) is None
    module.tfbuffer.receive_transform(
        Transform(translation=Vector3(0.5, 0, 0), frame_id="base", child_frame_id="arm", ts=1.0)
    )

    result = module.filter_cloud(_cloud([[0.5, 0, 0], [0, 0, 0]]))

    assert result is not None
    np.testing.assert_allclose(result[0].points_f32(), [[0, 0, 0]])


def test_narrowphase_retains_sphere_corner_obstacles_and_ancillary_fields(make_filter):
    urdf = _URDF.replace('<box size="0.2 0.2 0.2"/>', '<sphere radius="0.1"/>')
    module = make_filter(urdf_xml=urdf, padding_m=0.05)
    _place_arm(module, (0, 0, 0), 1.0)
    cloud = PointCloud2.from_numpy(
        np.array([[0.14, 0, 0], [0.12, 0.12, 0], [0.2, 0, 0]], dtype=np.float32),
        frame_id="camera",
        timestamp=1.0,
        intensities=np.array([1, 2, 3], dtype=np.float32),
    )
    cloud.pointcloud_tensor.point["labels"] = Tensor(np.array([[10], [20], [30]], dtype=np.int32))

    result = module.filter_cloud(cloud)

    assert result is not None
    filtered = result[0]
    np.testing.assert_allclose(filtered.points_f32(), [[0.12, 0.12, 0], [0.2, 0, 0]])
    np.testing.assert_array_equal(filtered.intensities_f32(), [2, 3])
    np.testing.assert_array_equal(filtered.pointcloud_tensor.point["labels"].numpy(), [[20], [30]])
    assert (filtered.frame_id, filtered.ts) == ("camera", 1.0)


def test_rotated_base_aligns_camera_points_and_world_clear_cells(make_filter):
    module = make_filter()
    module.tfbuffer.receive_transform(Transform(frame_id="world", child_frame_id="camera", ts=1.0))
    module.tfbuffer.receive_transform(
        Transform(
            translation=Vector3(-1, 0, 0),
            rotation=Quaternion(0, 0, np.sin(np.pi / 4), np.cos(np.pi / 4)),
            frame_id="world",
            child_frame_id="base",
            ts=1.0,
        )
    )

    result = module.filter_cloud(_cloud([[-1, 0.05, 0], [-1, 1, 0]]))

    assert result is not None
    np.testing.assert_allclose(result[0].points_f32(), [[-1, 1, 0]])
    assert (-20, 0, 0) in _keys(result[1], 0.05)
    offsets = result[1].points_f32() / 0.05 - np.floor(result[1].points_f32() / 0.05)
    np.testing.assert_allclose(offsets, 0.5, atol=1e-5)


def test_real_mesh_surface_is_removed_and_its_interior_is_cleared(make_filter, tmp_path):
    mesh_path = tmp_path / "cube.stl"
    trimesh.creation.box(extents=[0.2, 0.2, 0.2]).export(mesh_path)
    urdf = _URDF.replace('<box size="0.2 0.2 0.2"/>', f'<mesh filename="{mesh_path}"/>')
    module = make_filter(urdf_xml=urdf)
    _place_arm(module, (0, 0, 0), 1.0)

    result = module.filter_cloud(_cloud([[0.1, 0, 0], [0.105, 0, 0], [0.2, 0, 0]]))

    assert result is not None
    np.testing.assert_allclose(result[0].points_f32(), [[0.2, 0, 0]])
    assert (0, 0, 0) in _keys(result[1], 0.05)
