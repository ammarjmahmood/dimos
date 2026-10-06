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

"""Seeded procedural scenes built from boxes."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field
import hashlib
import json
import math

import numpy as np
from numpy.typing import ArrayLike, NDArray

from dimos.msgs.sim_msgs.Contacts import Kind

Vec3 = tuple[float, float, float]

SLAB_THICKNESS = 0.15
WALL_THICKNESS = 0.1
CEILING_HEIGHT = 2.6
DOOR_HEIGHT = 2.0
DOOR_WIDTH = (0.8, 1.2)
TABLE_TOP_THICKNESS = 0.04
TABLE_LEG = 0.04
GOAL_MIN_DISTANCE = 5.0


@dataclass(frozen=True)
class Box:
    center: Vec3
    half: Vec3
    kind: Kind


@dataclass(frozen=True)
class Goal:
    name: str
    position: Vec3


@dataclass
class Scene:
    name: str
    params: dict[str, float] = field(default_factory=dict)
    boxes: list[Box] = field(default_factory=list)
    start: Vec3 = (0.0, 0.0, 0.0)
    goals: list[Goal] = field(default_factory=list)

    def add(self, lo: ArrayLike, hi: ArrayLike, kind: Kind) -> None:
        low, high = np.asarray(lo, float), np.asarray(hi, float)
        if np.any(high - low <= 1e-6):
            return
        center, half = (low + high) / 2, (high - low) / 2
        self.boxes.append(
            Box(
                (float(center[0]), float(center[1]), float(center[2])),
                (float(half[0]), float(half[1]), float(half[2])),
                kind,
            )
        )

    def bounds(self) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        centers = np.array([box.center for box in self.boxes])
        halves = np.array([box.half for box in self.boxes])
        return (centers - halves).min(0), (centers + halves).max(0)

    def digest(self) -> str:
        """A hash of the geometry, start and goals, for detecting generator drift."""
        record = {
            "boxes": [(box.center, box.half, box.kind) for box in self.boxes],
            "start": self.start,
            "goals": [(goal.name, goal.position) for goal in self.goals],
        }
        return hashlib.sha256(json.dumps(record, sort_keys=True).encode()).hexdigest()[:16]


def _walls(scene: Scene, x0: float, y0: float, x1: float, y1: float, z: float, top: float) -> None:
    t = WALL_THICKNESS
    scene.add((x0 - t, y0 - t, z), (x0, y1 + t, top), "wall")
    scene.add((x1, y0 - t, z), (x1 + t, y1 + t, top), "wall")
    scene.add((x0, y0 - t, z), (x1, y0, top), "wall")
    scene.add((x0, y1, z), (x1, y1 + t, top), "wall")


def _ceiling(scene: Scene, x0: float, y0: float, x1: float, y1: float, z: float) -> None:
    t = WALL_THICKNESS
    scene.add((x0 - t, y0 - t, z), (x1 + t, y1 + t, z + SLAB_THICKNESS), "ceiling")


def _wall_with_doors(
    scene: Scene,
    axis: int,
    at: float,
    lo: float,
    hi: float,
    z: float,
    top: float,
    doors: list[tuple[float, float]],
) -> None:
    """A wall across the given axis with door gaps and lintels."""
    t = WALL_THICKNESS
    edges = [lo]
    for door_start, door_width in sorted(doors):
        edges += [door_start, door_start + door_width]
    edges.append(hi)
    for i in range(0, len(edges), 2):
        a, b = edges[i], edges[i + 1]
        if axis == 0:
            scene.add((at, a, z), (at + t, b, top), "wall")
        else:
            scene.add((a, at, z), (b, at + t, top), "wall")
    for door_start, door_width in doors:
        lintel = z + DOOR_HEIGHT
        if axis == 0:
            scene.add((at, door_start, lintel), (at + t, door_start + door_width, top), "wall")
        else:
            scene.add((door_start, at, lintel), (door_start + door_width, at + t, top), "wall")


def _clutter(
    scene: Scene,
    rng: np.random.Generator,
    x0: float,
    y0: float,
    x1: float,
    y1: float,
    z: float,
    n: int,
) -> None:
    for _ in range(n):
        hx, hy = rng.uniform(0.15, 0.5), rng.uniform(0.15, 0.5)
        cx, cy = rng.uniform(x0 + hx, x1 - hx), rng.uniform(y0 + hy, y1 - hy)
        scene.add((cx - hx, cy - hy, z), (cx + hx, cy + hy, z + rng.uniform(0.2, 1.0)), "clutter")


def _table(
    scene: Scene, rng: np.random.Generator, x0: float, y0: float, x1: float, y1: float, z: float
) -> None:
    """A table on four legs, with its top anywhere from below to well above the robot's height."""
    lx, ly = rng.uniform(1.0, 1.8), rng.uniform(0.6, 0.9)
    tx, ty = rng.uniform(x0, x1 - lx), rng.uniform(y0, y1 - ly)
    top = z + rng.uniform(0.45, 0.8)
    scene.add((tx, ty, top - TABLE_TOP_THICKNESS), (tx + lx, ty + ly, top), "clutter")
    for px in (tx, tx + lx - TABLE_LEG):
        for py in (ty, ty + ly - TABLE_LEG):
            scene.add(
                (px, py, z), (px + TABLE_LEG, py + TABLE_LEG, top - TABLE_TOP_THICKNESS), "clutter"
            )


def _far_point(
    rng: np.random.Generator,
    x0: float,
    y0: float,
    x1: float,
    y1: float,
    z: float,
    start: Vec3,
    min_dist: float,
) -> Vec3:
    for _ in range(200):
        p = (float(rng.uniform(x0, x1)), float(rng.uniform(y0, y1)), z)
        if math.hypot(p[0] - start[0], p[1] - start[1]) >= min_dist:
            return p
    return p


def office(seed: int) -> Scene:
    """One floor of rooms joined by doorways, with clutter and tables placed anywhere."""
    rng = np.random.default_rng(seed)
    width, length = float(rng.uniform(12, 18)), float(rng.uniform(9, 13))
    z0 = float(rng.uniform(0.0, 0.08))
    scene = Scene(f"office_{seed}", {"width": width, "length": length, "z0": z0})
    top = z0 + CEILING_HEIGHT
    scene.add((0.0, 0.0, z0 - SLAB_THICKNESS), (width, length, z0), "floor")
    _walls(scene, 0, 0, width, length, z0, top)
    _ceiling(scene, 0, 0, width, length, top)
    wy = rng.uniform(0.4, 0.6) * length
    xs = sorted(rng.uniform(0.25, 0.75, size=int(rng.integers(1, 3))) * width)
    doors_mid = []
    for a, b in zip([0.0, *xs], [*xs, width], strict=True):
        door_width = rng.uniform(*DOOR_WIDTH)
        if b - a > door_width + 1.0:
            doors_mid.append((rng.uniform(a + 0.4, b - door_width - 0.4), door_width))
    _wall_with_doors(scene, 1, wy, 0, width, z0, top, doors_mid)
    for x in xs:
        for a, b in ((0.0, wy), (wy + WALL_THICKNESS, length)):
            door_width = rng.uniform(*DOOR_WIDTH)
            door = (rng.uniform(a + 0.3, b - door_width - 0.3), door_width)
            _wall_with_doors(scene, 0, x, a, b, z0, top, [door])
    clutter = int(rng.integers(8, 16))
    _clutter(scene, rng, 0.2, 0.2, width - 0.2, length - 0.2, z0, clutter)
    tables = int(rng.integers(1, 4))
    for _ in range(tables):
        _table(scene, rng, 0.3, 0.3, width - 0.3, length - 0.3, z0)
    scene.params.update({"rooms": 2 * (len(xs) + 1), "clutter": clutter, "tables": tables})
    scene.start = (1.0, 1.0, z0)
    for i in range(3):
        goal = _far_point(
            rng, 0.8, 0.8, width - 0.8, length - 0.8, z0, scene.start, GOAL_MIN_DISTANCE
        )
        scene.goals.append(Goal(f"room_{i}", goal))
    return scene


FAMILIES: dict[str, Callable[[int], Scene]] = {"office": office}


def generate(family: str, seed: int) -> Scene:
    return FAMILIES[family](seed)
