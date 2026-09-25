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

from typing import Any

import pytest

from dimos.evals.suites.lib.r1pro_tasks import (
    Layout,
    full_task,
    grade_samples,
    roles,
    tray_task,
)
from dimos.evals.suites.r1pro_open_space import SUITE, layouts


@pytest.fixture
def layout() -> Layout:
    return next(layout for layout in layouts() if layout.seed == 5000)


class World:
    """A hand-driven stand-in for get_scene samples over time."""

    def __init__(self, layout: Layout) -> None:
        self.layout = layout
        self.on = {o.id: o.platform for o in layout.objects}
        self.inside: set[str] = set()
        self.held: dict[str, str | None] = {"left": None, "right": None}
        self.tray_station: str | None = layout.tray_platform
        self.tray_held = False
        self.t = 0.0
        self.samples: list[dict[str, Any]] = []

    def pick(self, obj_id: str, arm: str) -> None:
        self.held[arm] = obj_id
        self.on[obj_id] = None  # type: ignore[assignment]
        self.inside.discard(obj_id)
        self.snap()

    def place(self, arm: str, where: str) -> None:
        obj_id = self.held[arm]
        assert obj_id is not None
        self.held[arm] = None
        if where == "tray":
            self.inside.add(obj_id)
            self.on[obj_id] = "tray"
        else:
            self.on[obj_id] = where
        self.snap()

    def lift_tray(self) -> None:
        self.tray_held, self.tray_station = True, None
        self.snap()

    def set_tray(self, platform: str) -> None:
        self.tray_held, self.tray_station = False, platform
        self.snap()

    def snap(self) -> None:
        self.t += 1.0
        self.samples.append(
            {
                "t": self.t,
                "objects": [
                    {
                        "id": o.id,
                        "on": self.on[o.id],
                        "upright": True,
                        "inside": o.id in self.inside,
                    }
                    for o in self.layout.objects
                ],
                "held_objects": dict(self.held),
                "tray": {"station": self.tray_station, "held": self.tray_held},
                "base_pose": [0.0, 0.0, 0.0],
            }
        )


def test_every_seed_has_the_full_curriculum() -> None:
    assert len(SUITE) == 6 * len(layouts())
    assert len({case.id for case in SUITE}) == len(SUITE)
    for layout in layouts():
        r = roles(layout)
        assert len({r["Y"], r["Z"], r["W"]}) == 3 and layout.tray_platform not in r.values()
        assert r["D"] in ("low_bench", "display_table")


def test_full_task_done_as_asked_scores_one(layout: Layout) -> None:
    task = full_task(layout)
    r = roles(layout)
    a, b, c = layout.on(r["Y"]), layout.on(r["Z"]), layout.on(r["W"])
    world = World(layout)
    world.snap()
    world.pick(a.id, "left")
    world.pick(b.id, "right")
    world.place("left", r["W"])
    world.place("right", r["W"])
    world.pick(c.id, "right")
    world.pick(b.id, "left")
    world.place("right", "tray")
    world.place("left", "tray")
    world.lift_tray()
    world.set_tray(r["D"])
    grade = grade_samples(task, world.samples)
    assert grade.score == 1.0, grade.details
    assert grade.details["missed"] == [] and grade.details["dropped"] == []


def test_stopping_after_the_forced_swap_gets_partial_credit(layout: Layout) -> None:
    task = full_task(layout)
    r = roles(layout)
    a, b, c = layout.on(r["Y"]), layout.on(r["Z"]), layout.on(r["W"])
    world = World(layout)
    world.snap()
    world.pick(a.id, "left")
    world.pick(b.id, "right")
    world.place("left", r["W"])
    world.place("right", r["W"])
    world.pick(c.id, "right")
    grade = grade_samples(task, world.samples)
    assert grade.details["milestones_reached"] == 5
    assert grade.details["missed"][0].endswith("in left hand")
    assert 0.0 < grade.score < 1.0


def test_milestones_count_only_in_order(layout: Layout) -> None:
    """Loading the tray before the picks it depends on earns nothing past the first gap."""
    task = tray_task(layout, "low_bench")
    obj = layout.on(layout.tray_platform)
    world = World(layout)
    world.snap()
    world.lift_tray()  # tray first: "held" milestone not reached yet, so this cannot count
    world.set_tray(layout.tray_platform)
    world.pick(obj.id, "right")
    grade = grade_samples(task, world.samples)
    assert grade.details["milestones_reached"] == 1


def test_a_malformed_sample_proves_nothing(layout: Layout) -> None:
    task = full_task(layout)
    grade = grade_samples(task, [{"t": 0.0, "objects": [], "held_objects": {}, "tray": {}}])
    assert grade.score == 0.0


def test_an_object_on_no_support_is_reported_dropped(layout: Layout) -> None:
    world = World(layout)
    world.on[layout.objects[0].id] = None  # type: ignore[assignment]
    world.snap()
    grade = grade_samples(full_task(layout), world.samples)
    assert grade.details["dropped"] == [layout.objects[0].id]
