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

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import Blueprint, BlueprintAtom
from dimos.manipulation.manipulation_module import ManipulationModule, ManipulationModuleConfig
from dimos.manipulation.planning.utils.point_cloud_self_filter import (
    PointCloudSelfFilter,
    PointCloudSelfFilterConfig,
)
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap, RayTracingVoxelMapConfig
from dimos.robot.manipulators.xarm.blueprints.sim2 import (
    XARM7_SIM2_VOXEL_SIZE,
    xarm_perception_sim2,
)
from dimos.robot.manipulators.xarm.config import XARM7_COLLISION_LINKS
from dimos.robot.manipulators.xarm.sim2 import xarm7_simulation
from dimos.sim2.sensors.camera.module import SimRGBDCameraModule, SimRGBDPointCloudCameraModule


def _topic(blueprint: Blueprint, atom: BlueprintAtom, stream: str) -> str:
    """The topic a module's port ends up on, the way the coordinator resolves it."""
    topic = blueprint.remapping_map.get((atom.name, stream), stream)
    assert isinstance(topic, str)
    return topic


def test_the_wrist_camera_cloud_reaches_the_planner_through_the_filter_and_mapper() -> None:
    blueprint = xarm_perception_sim2
    atoms = {atom.name: atom for atom in blueprint.active_blueprints}
    camera = atoms["arm_wrist_camera"]
    self_filter = atoms[PointCloudSelfFilter.name]
    mapper = atoms[RayTracingVoxelMap.name]
    planner = atoms[ManipulationModule.name]

    assert camera.module is SimRGBDPointCloudCameraModule
    assert "pointcloud" in {stream.name for stream in camera.streams}
    assert _topic(blueprint, camera, "pointcloud") == _topic(blueprint, self_filter, "pointcloud")
    assert _topic(blueprint, self_filter, "filtered_pointcloud") == _topic(
        blueprint, mapper, "lidar"
    )
    assert _topic(blueprint, self_filter, "voxel_clear_mask") == _topic(
        blueprint, mapper, "voxel_clear_mask"
    )
    assert _topic(blueprint, mapper, "global_map") == _topic(blueprint, planner, "voxel_map")

    # Scene registration publishes its own "pointcloud" of detected objects; the
    # mapping chain must see only what the camera saw.
    registration = atoms["objectsceneregistrationmodule"]
    assert _topic(blueprint, registration, "pointcloud") != _topic(
        blueprint, self_filter, "pointcloud"
    )


def test_the_chain_shares_one_resolution_and_one_planning_frame() -> None:
    blueprint = xarm_perception_sim2
    atoms = {atom.name: atom for atom in blueprint.active_blueprints}
    self_filter = atoms[PointCloudSelfFilter.name]
    mapper = atoms[RayTracingVoxelMap.name]
    planner = atoms[ManipulationModule.name]

    parsed = BlueprintConfigParser(blueprint).parse(environ={})
    filter_config = PointCloudSelfFilterConfig.model_validate(
        parsed.module_kwargs(self_filter.name)
    )
    mapper_config = RayTracingVoxelMapConfig.model_validate(parsed.module_kwargs(mapper.name))
    planner_config = ManipulationModuleConfig.model_validate(parsed.module_kwargs(planner.name))

    assert (
        filter_config.voxel_size
        == mapper_config.voxel_size
        == planner_config.voxel_map_resolution
        == XARM7_SIM2_VOXEL_SIZE
    )
    assert filter_config.world_frame == mapper_config.world_frame == "world"
    assert planner_config.world_frame == planner_config.model.base_pose.frame_id == "world"
    # The self filter needs a capture-time transform for every collision link,
    # and the planner publishes them all against "world", the camera's parent.
    assert set(XARM7_COLLISION_LINKS) <= set(planner_config.model.tf_extra_links)
    # Robot TF arrives at 10 Hz; a tolerance under one period drops most clouds.
    assert filter_config.tf_tolerance_s >= 0.1
    assert filter_config.tf_forward_tolerance_s >= 0.1


def test_the_plain_simulation_keeps_the_planner_coordinator_camera_and_tf() -> None:
    plain = xarm7_simulation(None)
    camera = next(
        atom for atom in plain.devices.active_blueprints if atom.name == "arm_wrist_camera"
    )
    assert camera.module is SimRGBDCameraModule
    assert "pointcloud" not in {stream.name for stream in camera.streams}
    assert plain.model.tf_extra_links == ["link7"]
