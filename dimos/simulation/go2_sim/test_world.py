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

import numpy as np
import pytest

from dimos.msgs.sim_msgs.Contacts import Contact
from dimos.navigation.sim_eval.scenes import office
from dimos.simulation.go2_sim.world import FRAME_DT, SimWorld, scene_edges

pytestmark = pytest.mark.mujoco

STILL = np.zeros(3)


@pytest.fixture(scope="module")
def world() -> SimWorld:
    scene = office(1)
    world = SimWorld(scene, seed=1)
    world.reset(*scene.start, 0.0)
    return world


def test_frames_come_out_at_10hz_deskewed_to_the_frame_end_pose(world: SimWorld) -> None:
    frames = [f for _ in range(10) if (f := world.tick(STILL)) is not None]
    assert [f.t for f in frames] == pytest.approx([FRAME_DT, 2 * FRAME_DT])
    frame = frames[-1]
    assert 12_000 < len(frame.points) < 20_000
    world_points = frame.position + frame.points.astype(np.float64) @ frame.rotation.T
    floor = world.scene.params["z0"]
    low = np.quantile(world_points[:, 2], 0.1)
    assert abs(low - floor) < 0.03
    assert world_points[:, 2].min() > floor - 0.05


def test_standing_robot_touches_the_floor_only_with_its_feet(world: SimWorld) -> None:
    for _ in range(25):
        world.tick(STILL)
    contacts = world.contacts()
    assert Contact("foot", "floor") in contacts
    assert {c.kind for c in contacts} == {"floor"}
    assert "trunk" not in {c.part for c in contacts}


def test_sensor_sits_on_the_mount_above_the_base(world: SimWorld) -> None:
    base, _ = world.base_pose()
    sensor, rotation = world.sensor_pose()
    assert 0.1 < sensor[2] - base[2] < 0.2
    forward = rotation @ np.array([1.0, 0.0, 0.0])
    assert forward[2] < -0.8


def test_scene_edges_cover_every_box(world: SimWorld) -> None:
    edges = scene_edges(world.scene)
    assert edges.shape == (12 * len(world.scene.boxes), 2, 3)
    lo, hi = world.scene.bounds()
    assert np.allclose(edges.reshape(-1, 3).min(0), lo)
    assert np.allclose(edges.reshape(-1, 3).max(0), hi)
