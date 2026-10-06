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

import numpy as np

from dimos.navigation.sim_eval.scenes import SLAB, generate, office


def test_office_is_deterministic_per_seed() -> None:
    assert office(3).digest() == office(3).digest()
    assert office(3).digest() != office(4).digest()


def test_office_geometry_is_enclosed_and_on_one_floor() -> None:
    s = generate("office", 1)
    z0 = s.params["z0"]
    kinds = {b.kind for b in s.boxes}
    assert kinds == {"floor", "wall", "ceiling", "clutter"}
    lo, hi = s.bounds()
    assert lo[2] == np.float64(z0 - SLAB)
    assert hi[2] > z0 + 2.6
    assert s.start[2] == z0
    assert all(g.position[2] == z0 for g in s.goals)


def test_office_goals_are_far_from_the_start_and_inside_the_walls() -> None:
    s = office(7)
    lo, hi = s.bounds()
    for g in s.goals:
        assert math.hypot(g.position[0] - s.start[0], g.position[1] - s.start[1]) >= 5.0
        assert lo[0] < g.position[0] < hi[0]
        assert lo[1] < g.position[1] < hi[1]


def test_office_records_its_parameters() -> None:
    p = office(2).params
    assert {"width", "length", "z0", "rooms", "clutter", "tables"} <= p.keys()
    assert p["rooms"] in (4, 6)
