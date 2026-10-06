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

"""Wiring contracts for robot filtering before xArm voxel fusion."""

import importlib

import pytest

from dimos.control.tasks.trajectory_task.trajectory_task import JOINT_TRAJECTORY_TASK_NAME
from dimos.core.global_config import global_config
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.pointcloud.robot_pointcloud_filter_module import RobotPointCloudFilterModule
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.robot.manipulators.xarm.blueprints import grasp


@pytest.mark.parametrize("simulation", ["", "mujoco"])
def test_grasp_filters_captures_before_fusion_with_the_planners_model(monkeypatch, simulation):
    try:
        with monkeypatch.context() as patch:
            patch.setattr(global_config, "simulation", simulation)
            blueprint = importlib.reload(grasp).xarm_grasp
            atoms = {atom.module: atom for atom in blueprint.blueprints}
            filtering = atoms[RobotPointCloudFilterModule]
            planning = atoms[ManipulationModule]
            mapping = atoms[RayTracingVoxelMap]

            assert filtering.kwargs["model"] is planning.kwargs["model"]
            model = filtering.kwargs["model"]
            assert ("drive_joint" in model.joint_names) == bool(simulation)
            assert bool(model.joint_state_transforms) == bool(simulation)
            assert planning.kwargs["trajectory_tasks"][JOINT_TRAJECTORY_TASK_NAME] == list(
                model.planning_groups[0].joint_names
            )
            assert "drive_joint" not in model.planning_groups[0].joint_names
            assert blueprint.remapping_map[(mapping.name, "lidar")] == "filtered_pointcloud"
            assert blueprint.remapping_map[(planning.name, "voxel_map")] == "global_map"
            assert {stream.name for stream in planning.streams}.isdisjoint(
                {"pointcloud", "filtered_pointcloud"}
            )
            assert {(stream.name, stream.direction) for stream in filtering.streams} >= {
                ("pointcloud", "in"),
                ("filtered_pointcloud", "out"),
                ("coordinator_joint_state", "in"),
                ("tf", "in"),
            }
    finally:
        importlib.reload(grasp)
