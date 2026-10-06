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

import math

import pytest

from dimos.simulation.scenes.procedural import (
    CEILING_HEIGHT,
    GOAL_MIN_DISTANCE,
    SLAB_THICKNESS,
    Scene,
    generate,
    office,
)


def test_office_is_deterministic_and_pinned() -> None:
    assert office(3).digest() == office(3).digest() == "9e198719f08231d1"
    assert office(3).digest() != office(4).digest()


def test_office_geometry_is_enclosed_and_on_one_floor() -> None:
    scene = generate("office", 1)
    z0 = scene.params["z0"]
    assert {box.kind for box in scene.boxes} == {"floor", "wall", "ceiling", "clutter"}
    lo, hi = scene.bounds()
    assert lo[2] == pytest.approx(z0 - SLAB_THICKNESS)
    assert hi[2] > z0 + CEILING_HEIGHT
    assert scene.start[2] == z0
    assert all(goal.position[2] == z0 for goal in scene.goals)


def test_office_goals_are_far_from_the_start_and_inside_the_walls() -> None:
    scene = office(7)
    lo, hi = scene.bounds()
    for goal in scene.goals:
        distance = math.hypot(goal.position[0] - scene.start[0], goal.position[1] - scene.start[1])
        assert distance >= GOAL_MIN_DISTANCE
        assert lo[0] < goal.position[0] < hi[0]
        assert lo[1] < goal.position[1] < hi[1]


def test_office_records_its_parameters() -> None:
    params = office(2).params
    assert {"width", "length", "z0", "rooms", "clutter", "tables"} <= params.keys()
    assert params["rooms"] in (4, 6)


def test_degenerate_boxes_are_dropped() -> None:
    scene = Scene("empty")
    scene.add((0.0, 0.0, 0.0), (1.0, 0.0, 1.0), "wall")
    assert scene.boxes == []


def test_unknown_family_is_an_error() -> None:
    with pytest.raises(KeyError):
        generate("warehouse", 1)
