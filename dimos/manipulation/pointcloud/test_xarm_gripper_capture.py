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

"""CPU-only cached xArm geometry regression; no simulator or live transport."""

import xml.etree.ElementTree as ET

import numpy as np
import pytest
import trimesh

pin = pytest.importorskip("pinocchio")
pytest.importorskip("roboplan.core")

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.manipulation.planning.planners.roboplan_config import RoboPlanPlannerConfig
from dimos.manipulation.planning.planners.roboplan_planner import RoboPlanPlanner
from dimos.manipulation.planning.spec.enums import PlanningStatus
from dimos.manipulation.planning.spec.measured_joint_state import canonicalize_measured_joint_state
from dimos.manipulation.planning.spec.validation import prepare_robot_model
from dimos.manipulation.planning.trajectory_generator.config import (
    RoboPlanTOPPRAParametrizationConfig,
)
from dimos.manipulation.planning.trajectory_generator.roboplan_toppra_parametrizer import (
    RoboPlanTOPPRAParametrizer,
)
from dimos.manipulation.planning.utils.mesh_utils import prepare_urdf_for_drake
from dimos.manipulation.planning.world.roboplan_model import _prepare_model
from dimos.manipulation.planning.world.roboplan_world import RoboPlanWorld
from dimos.manipulation.pointcloud.robot_pointcloud_filter import RobotPointCloudFilter
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.protocol.tf.tf import MultiTBuffer
from dimos.robot.assets.git_cache import DEFAULT_ROBOT_ASSET_CACHE_ROOT, GitAssetCache
from dimos.robot.manipulators.xarm.config import (
    XARM_ROS2_REF,
    XARM_ROS2_REPO,
    make_xarm7_sim_robot_config,
)

pytestmark = pytest.mark.self_hosted


@pytest.fixture
def cached_filter(monkeypatch):
    key = GitAssetCache._source_key(XARM_ROS2_REPO, XARM_ROS2_REF)
    cached = DEFAULT_ROBOT_ASSET_CACHE_ROOT / "sources" / key / "xarm_ros2"
    if not cached.is_dir():
        pytest.skip("Pinned xArm source cache is required; this test never fetches assets")
    monkeypatch.setattr(GitAssetCache, "resolve", lambda *_: cached)
    config = make_xarm7_sim_robot_config(
        base_pose=PoseStamped(frame_id="world"), tf_extra_links=[], include_gripper_state=True
    )
    filtering = RobotPointCloudFilter(config)
    try:
        yield filtering, config
    finally:
        filtering.close()


def test_actual_gripper_surfaces_follow_capture_measurements_and_mimics(cached_filter):
    filtering, config = cached_filter
    prepared = filtering._prepared
    converted = prepare_urdf_for_drake(prepared.description, convert_meshes=True)
    xml = _prepare_model(config, converted).xml
    urdf = ET.fromstring(xml)
    drive = urdf.find("./joint[@name='drive_joint']")
    assert drive is not None
    assert drive.find("limit").get("lower") == "0"
    assert float(drive.find("limit").get("upper")) == 0.85
    assert len(urdf.findall("./joint/mimic[@joint='drive_joint']")) == 5
    mjcf = ET.parse(DIMOS_PROJECT_ROOT / "data/xarm_grasp_sim/hand.xml")
    assert mjcf.find(".//default[@class='xarm7_hand']/joint").get("range") == "0 0.85"
    assert mjcf.find(".//joint[@name='left_driver_joint']").get("class") == "driver"

    model = pin.buildModelFromXML(xml, mimic=True)
    geoms = pin.buildGeomFromUrdfString(
        model,
        xml,
        pin.GeometryType.COLLISION,
        package_dirs=[str(p) for p in converted.package_paths.values()],
    )
    assert model.nq == 8
    assert model.joints[model.getJointId("drive_joint")].idx_q == 7
    arm = [0.0, -0.04609, 0.0, 1.83940, 0.0, 1.87106, 0.0]
    controls = np.array([[2, 2, 2], [-2, -2, -2]], dtype=np.float32)
    captures = []
    for stamp, drive_position in ((1.0, 0.0), (2.0, 0.85)):
        measured = JointState(
            ts=stamp,
            name=["arm/gripper", *config.joint_names[:7]],
            position=[0.85 - drive_position, *arm],
        )
        canonical = canonicalize_measured_joint_state(measured, config)
        assert canonical.position == [*arm, drive_position]
        filtering.record_joint_state(measured)
        data, gdata = model.createData(), pin.GeometryData(geoms)
        pin.updateGeometryPlacements(model, data, geoms, gdata, np.asarray([*arm, drive_position]))
        surfaces = []
        for i, geom in enumerate(geoms.geometryObjects):
            if any(name in geom.name for name in ("gripper", "finger", "knuckle")):
                local, _ = trimesh.sample.sample_surface(
                    trimesh.load_mesh(geom.meshPath), 100, seed=42 + i
                )
                local *= np.asarray(geom.meshScale)
                surfaces.append(local @ gdata.oMg[i].rotation.T + gdata.oMg[i].translation)
        assert len(surfaces) == 7
        captures.append(
            PointCloud2.from_numpy(
                np.concatenate([*surfaces, controls]).astype(np.float32),
                frame_id="world",
                timestamp=stamp,
            )
        )

    # Both messages arrive after the newer state; each must use its own gripper pose.
    for cloud in captures:
        filtered = filtering.filter(cloud, MultiTBuffer())
        assert filtered is not None
        assert filtered.ts == cloud.ts and filtered.frame_id == "world"
        np.testing.assert_allclose(filtered.points_f32(), controls)


def test_measured_gripper_remains_outside_native_arm_trajectory(cached_filter):
    filtering, config = cached_filter
    world = RoboPlanWorld()
    world.load_model(prepare_robot_model(config))
    world.finalize()
    assert world._scene is not filtering._model.scene
    arm = [0.0, -0.04609, 0.0, 1.83940, 0.0, 1.87106, 0.0]
    measured = JointState(
        ts=12.5, name=[*config.joint_names[:7], "arm/gripper"], position=[*arm, 0.0]
    )
    world.sync_from_joint_state(canonicalize_measured_joint_state(measured, config))
    selection = world._planning_groups.select(("manipulator",))
    start = JointState(name=list(selection.joint_names), position=arm)
    goal = JointState(name=start.name, position=[arm[0] + 0.005, *arm[1:]])
    result = RoboPlanPlanner(world, RoboPlanPlannerConfig()).plan_selected_joint_path(
        world, selection, start, goal, timeout=1.0
    )
    assert result.status == PlanningStatus.SUCCESS, result.message
    plan = RoboPlanTOPPRAParametrizer(RoboPlanTOPPRAParametrizationConfig()).materialize_plan(
        world, selection, result
    )
    assert plan.trajectory.joint_names == config.joint_names[:7]
    np.testing.assert_allclose(plan.trajectory.points[-1].positions, goal.position)
    assert world.get_joint_state(world.get_live_context()).position[-1] == 0.85
