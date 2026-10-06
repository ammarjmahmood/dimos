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

import mujoco
import numpy as np
from numpy.typing import NDArray
import pytest

from dimos.simulation.go2_legged.policy import OnnxGo2Policy
from dimos.simulation.go2_legged.robot import LeggedGo2, apply_fitted_physics, go2_spec

pytestmark = pytest.mark.mujoco


def _robot() -> LeggedGo2:
    spec = go2_spec()
    ground = spec.worldbody.add_geom()
    ground.type = mujoco.mjtGeom.mjGEOM_BOX
    ground.pos = (5.0, 0.0, -0.1)
    ground.size = (10.0, 5.0, 0.1)
    model = spec.compile()
    apply_fitted_physics(model)
    robot = LeggedGo2(model, mujoco.MjData(model), OnnxGo2Policy.load())
    robot.reset(0.0, 0.0, 0.0, 0.0)
    return robot


def _walk(seconds: float, command: tuple[float, float, float]) -> NDArray[np.float64]:
    robot = _robot()
    for _ in range(round(seconds / 0.02)):
        robot.tick(np.asarray(command, dtype=np.float64))
    assert robot.upright() > 0.9
    return np.append(robot.base_pose()[0], robot.yaw())


def test_spawns_standing_and_stays_put_without_a_command() -> None:
    robot = _robot()
    assert robot.standing
    for _ in range(150):
        robot.tick(np.zeros(3))
    end = robot.base_pose()[0]
    assert np.hypot(end[0], end[1]) < 0.02
    assert abs(robot.yaw()) < 0.02
    assert 0.25 < end[2] < 0.4


def test_walks_forward_at_about_the_commanded_speed() -> None:
    end = _walk(6.0, (0.8, 0.0, 0.0))
    assert 3.0 < end[0] < 5.0
    assert abs(end[1]) < 1.0


def test_turns_at_about_the_commanded_rate() -> None:
    end = _walk(4.0, (0.0, 0.0, 0.5))
    assert 1.2 < end[3] < 2.4


def test_stands_still_after_the_command_drops_to_zero() -> None:
    robot = _robot()
    for _ in range(150):
        robot.tick(np.array([0.5, 0.0, 0.5]))
    assert not robot.standing
    for _ in range(100):
        robot.tick(np.zeros(3))
    assert robot.standing
    start, yaw = robot.base_pose()[0].copy(), robot.yaw()
    for _ in range(150):
        robot.tick(np.zeros(3))
    end = robot.base_pose()[0]
    assert np.hypot(*(end[:2] - start[:2])) < 0.02
    assert abs(robot.yaw() - yaw) < 0.02
    assert end[2] > 0.25
    assert robot.upright() > 0.95


def test_rollouts_are_deterministic() -> None:
    assert np.array_equal(_walk(2.0, (0.5, 0.0, 0.3)), _walk(2.0, (0.5, 0.0, 0.3)))
