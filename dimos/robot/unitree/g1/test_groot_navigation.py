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

"""G1 navigation wiring through the actual blueprint and worker config boundary."""

import importlib.util
from pathlib import Path
import pickle
import sys

import pytest

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.global_config import global_config
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap, RayTracingVoxelMapConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Path import Path as NavPath
from dimos.navigation.global_planner.mls_planner.mls_planner_native import (
    MLSPlannerNative,
    MLSPlannerNativeConfig,
)
from dimos.navigation.local_planner.native import LocalPlannerNative, LocalPlannerNativeConfig
from dimos.navigation.trajectory_follower.fancy.laws.hinted import HintedController
from dimos.navigation.trajectory_follower.fancy.native import (
    TrajectoryFollowerNative,
    TrajectoryFollowerNativeConfig,
)
from dimos.robot.unitree.g1.g1_tf_publisher import G1TfPublisher
from dimos.robot.unitree.g1.navigation import G1_GROOT_NAVIGATION
from dimos.sim2.sensors.spec import Lidar


@pytest.fixture(params=["", "mujoco"], ids=["hardware", "simulation"])
def groot(request, monkeypatch, tmp_path):
    scene = tmp_path / "scene.xml"
    scene.write_text("<mujoco><worldbody/></mujoco>")
    monkeypatch.setattr(global_config, "simulation", request.param)
    monkeypatch.setattr(global_config, "viewer", "none")
    monkeypatch.setattr(global_config, "scene_package", str(scene) if request.param else None)
    # Evaluate the import-time backend selection without replacing the real module
    # that other G1 blueprints import during test collection.
    spec = importlib.util.spec_from_file_location(
        "_test_groot_navigation", Path(__file__).parent / "blueprints/basic/unitree_g1_groot_wbc.py"
    )
    module = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, spec.name, module)
    spec.loader.exec_module(module)
    return module


def test_full_blueprint_reconstructs_g1_settings_for_native_workers(groot):
    blueprint = groot.unitree_g1_groot_wbc
    parsed = BlueprintConfigParser(blueprint).parse(environ={})
    local = LocalPlannerNativeConfig(
        **pickle.loads(pickle.dumps(parsed.module_kwargs(LocalPlannerNative.name)))
    )
    follower = TrajectoryFollowerNativeConfig(
        **pickle.loads(pickle.dumps(parsed.module_kwargs(TrajectoryFollowerNative.name)))
    )
    planner = MLSPlannerNativeConfig(**parsed.module_kwargs(MLSPlannerNative.name))
    mapper = RayTracingVoxelMapConfig(**parsed.module_kwargs(RayTracingVoxelMap.name))

    assert local.embodiment == follower.embodiment == G1_GROOT_NAVIGATION
    assert local.to_config_dict()["embodiment"] == follower.to_config_dict()["embodiment"]
    assert local.world_frame == planner.world_frame == mapper.world_frame == "world"
    assert local.base_frame == follower.base_frame == planner.base_frame == groot._nav_base_frame
    assert planner.robot_height == G1_GROOT_NAVIGATION.height
    assert planner.start_z_offset_m == G1_GROOT_NAVIGATION.base_height
    assert mapper.emit_every > 0
    assert mapper.voxel_size == planner.voxel_size


def test_global_and_local_paths_are_separate_and_control_still_uses_cmd_vel(groot):
    blueprint = groot.unitree_g1_groot_wbc
    assert blueprint.remapping_map[(MLSPlannerNative.name, "path")] == "planner_path"
    assert blueprint.remapping_map[(MLSPlannerNative.name, "global_map")] == "global_map_unused"
    assert blueprint.remapping_map[("ControlCoordinator", "twist_command")] == "cmd_vel"
    names = {atom.module.__name__ for atom in blueprint.active_blueprints}
    assert {"LocalPlannerNative", "TrajectoryFollowerNative", "MovementManager"} <= names
    assert names.isdisjoint({"CostMapper", "ReplanningAStarPlanner", "VoxelGridMapper"})


def test_navigation_does_not_leak_into_the_teleop_control_core(groot):
    core = groot._unitree_g1_groot_wbc_core
    assert {atom.module for atom in core.active_blueprints}.isdisjoint(
        {
            PointLio,
            G1TfPublisher,
            RayTracingVoxelMap,
            MLSPlannerNative,
            LocalPlannerNative,
            TrajectoryFollowerNative,
        }
    )
    task = next(
        task
        for task in groot._coordinator.blueprints[0].kwargs["tasks"]
        if task.type == "g1_groot_wbc"
    )
    assert task.params["auto_arm"] is bool(global_config.simulation)
    assert task.params["auto_dry_run"] is not bool(global_config.simulation)


@pytest.mark.parametrize("groot", ["mujoco"], indirect=True)
def test_simulated_lidar_uses_its_sensor_origin_and_the_actual_pelvis_frame(groot):
    blueprint = groot.unitree_g1_groot_wbc
    lidar = next(sensor for sensor in groot.G1_GROOT.sensors if isinstance(sensor, Lidar))
    assert blueprint.remapping_map[(RayTracingVoxelMap.name, "lidar")] == "pointcloud"
    assert lidar.output_frame == "sensor"
    assert groot._nav_base_frame == f"g1/{groot.G1_GROOT.root_body}"


@pytest.mark.parametrize("groot", [""], indirect=True)
def test_hardware_lio_and_existing_mount_publisher_supply_the_base_frame(groot):
    blueprint = groot.unitree_g1_groot_wbc
    pointlio = next(atom for atom in blueprint.active_blueprints if atom.module is PointLio)
    assert blueprint.remapping_map[(RayTracingVoxelMap.name, "lidar")] == "lidar"
    assert pointlio.kwargs["frame_id"] == "world"
    assert groot._nav_base_frame == "base_link"
    assert G1TfPublisher in {atom.module for atom in blueprint.active_blueprints}


def test_groot_follower_bounds_velocity_without_go2_slip_compensation():
    controller = HintedController(G1_GROOT_NAVIGATION)
    path = NavPath(frame_id="world", poses=[PoseStamped(), PoseStamped(2.0, 0.0, 0.0)])
    command = controller.update(PoseStamped(), path, t=1.0)
    assert command.linear.x == pytest.approx(0.3)
    assert command.linear.y == 0.0
    assert command.angular.z == 0.0
    assert controller.update(PoseStamped(), NavPath(), t=2.0).linear.x == 0.0
