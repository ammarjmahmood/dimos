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

"""Existing GR00T stack with PimSim's rolling Mid360 in place of the ideal scanner."""

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_groot_wbc import unitree_g1_groot_wbc
from dimos.robot.unitree.g1.sim2 import G1_GROOT_MID360
from dimos.sim2.blueprint import simulation
from dimos.sim2.scene import scene_path, scene_robot

if global_config.simulation != "mujoco":
    raise ValueError("unitree-g1-groot-mid360 requires --simulation mujoco")

_scene = scene_path(global_config.scene_package, "logistics.xml")
_devices = simulation(
    scene=_scene,
    robots={"g1": scene_robot(_scene, G1_GROOT_MID360, default=(0, 0, 0))},
    sim_id="g1-groot",
)
unitree_g1_groot_mid360 = autoconnect(unitree_g1_groot_wbc, _devices.blueprint)
