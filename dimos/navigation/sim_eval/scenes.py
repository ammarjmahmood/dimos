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

"""Seeded procedural scenes built from boxes.

Difficulty comes only from seeded parameters. Generators never keep clutter away from
goals or doorways. Cases a Go2 cannot physically do are rejected by validation, not
designed away.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field
import hashlib
import json
import math
from typing import Literal

import numpy as np
from numpy.typing import ArrayLike, NDArray

Vec3 = tuple[float, float, float]
Kind = Literal["floor", "wall", "ceiling", "clutter"]

SLAB = 0.15
WALL = 0.1


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
    family: str
    seed: int
    params: dict[str, float] = field(default_factory=dict)
    boxes: list[Box] = field(default_factory=list)
    start: Vec3 = (0.0, 0.0, 0.0)
    goals: list[Goal] = field(default_factory=list)

    def add(self, lo: ArrayLike, hi: ArrayLike, kind: Kind) -> None:
        a, b = np.asarray(lo, float), np.asarray(hi, float)
        if np.any(b - a <= 1e-6):
            return
        c, h = (a + b) / 2, (b - a) / 2
        self.boxes.append(
            Box(
                (float(c[0]), float(c[1]), float(c[2])),
                (float(h[0]), float(h[1]), float(h[2])),
                kind,
            )
        )

    def bounds(self) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        c = np.array([b.center for b in self.boxes])
        h = np.array([b.half for b in self.boxes])
        return (c - h).min(0), (c + h).max(0)

    def digest(self) -> str:
        """A hash of the geometry, start and goals, for detecting generator drift."""
        record = {
            "boxes": [(b.center, b.half, b.kind) for b in self.boxes],
            "start": self.start,
            "goals": [(g.name, g.position) for g in self.goals],
        }
        return hashlib.sha256(json.dumps(record, sort_keys=True).encode()).hexdigest()[:16]


def _walls(s: Scene, x0: float, y0: float, x1: float, y1: float, z: float, top: float) -> None:
    s.add((x0 - WALL, y0 - WALL, z), (x0, y1 + WALL, top), "wall")
    s.add((x1, y0 - WALL, z), (x1 + WALL, y1 + WALL, top), "wall")
    s.add((x0, y0 - WALL, z), (x1, y0, top), "wall")
    s.add((x0, y1, z), (x1, y1 + WALL, top), "wall")


def _ceiling(s: Scene, x0: float, y0: float, x1: float, y1: float, z: float) -> None:
    s.add((x0 - WALL, y0 - WALL, z), (x1 + WALL, y1 + WALL, z + SLAB), "ceiling")


def _wall_with_doors(
    s: Scene,
    axis: int,
    at: float,
    lo: float,
    hi: float,
    z: float,
    top: float,
    doors: list[tuple[float, float]],
) -> None:
    """A wall at x = at (axis 0) or y = at (axis 1) spanning lo..hi, with door gaps (start, width) and lintels."""
    edges = [lo]
    for d0, w in sorted(doors):
        edges += [d0, d0 + w]
    edges.append(hi)
    for i in range(0, len(edges), 2):
        a, b = edges[i], edges[i + 1]
        if axis == 0:
            s.add((at, a, z), (at + WALL, b, top), "wall")
        else:
            s.add((a, at, z), (b, at + WALL, top), "wall")
    for d0, w in doors:
        if axis == 0:
            s.add((at, d0, z + 2.0), (at + WALL, d0 + w, top), "wall")
        else:
            s.add((d0, at, z + 2.0), (d0 + w, at + WALL, top), "wall")


def _clutter(
    s: Scene, rng: np.random.Generator, x0: float, y0: float, x1: float, y1: float, z: float, n: int
) -> None:
    for _ in range(n):
        hx, hy = rng.uniform(0.15, 0.5), rng.uniform(0.15, 0.5)
        cx, cy = rng.uniform(x0 + hx, x1 - hx), rng.uniform(y0 + hy, y1 - hy)
        s.add((cx - hx, cy - hy, z), (cx + hx, cy + hy, z + rng.uniform(0.2, 1.0)), "clutter")


def _table(
    s: Scene, rng: np.random.Generator, x0: float, y0: float, x1: float, y1: float, z: float
) -> None:
    """A table on four legs, with its top anywhere from below to well above the robot's height."""
    lx, ly = rng.uniform(1.0, 1.8), rng.uniform(0.6, 0.9)
    tx, ty = rng.uniform(x0, x1 - lx), rng.uniform(y0, y1 - ly)
    top = z + rng.uniform(0.45, 0.8)
    s.add((tx, ty, top - 0.04), (tx + lx, ty + ly, top), "clutter")
    leg = 0.04
    for px in (tx, tx + lx - leg):
        for py in (ty, ty + ly - leg):
            s.add((px, py, z), (px + leg, py + leg, top - 0.04), "clutter")


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
    """One floor of rooms joined by doorways, with clutter and tables anywhere, including doorways and goals."""
    rng = np.random.default_rng(seed)
    width, length = float(rng.uniform(12, 18)), float(rng.uniform(9, 13))
    z0 = float(rng.uniform(0.0, 0.08))
    s = Scene(f"office_{seed}", "office", seed, {"width": width, "length": length, "z0": z0})
    top = z0 + 2.6
    s.add((0.0, 0.0, z0 - SLAB), (width, length, z0), "floor")
    _walls(s, 0, 0, width, length, z0, top)
    _ceiling(s, 0, 0, width, length, top)
    wy = rng.uniform(0.4, 0.6) * length
    xs = sorted(rng.uniform(0.25, 0.75, size=int(rng.integers(1, 3))) * width)
    doors_mid = []
    for a, b in zip([0.0, *xs], [*xs, width], strict=True):
        w = rng.uniform(0.8, 1.2)
        if b - a > w + 1.0:
            doors_mid.append((rng.uniform(a + 0.4, b - w - 0.4), w))
    _wall_with_doors(s, 1, wy, 0, width, z0, top, doors_mid)
    for x in xs:
        for a, b in ((0.0, wy), (wy + WALL, length)):
            w = rng.uniform(0.8, 1.2)
            _wall_with_doors(s, 0, x, a, b, z0, top, [(rng.uniform(a + 0.3, b - w - 0.3), w)])
    clutter = int(rng.integers(8, 16))
    _clutter(s, rng, 0.2, 0.2, width - 0.2, length - 0.2, z0, clutter)
    tables = int(rng.integers(1, 4))
    for _ in range(tables):
        _table(s, rng, 0.3, 0.3, width - 0.3, length - 0.3, z0)
    s.params.update({"rooms": 2 * (len(xs) + 1), "clutter": clutter, "tables": tables})
    s.start = (1.0, 1.0, z0)
    for i in range(3):
        s.goals.append(
            Goal(
                f"room_{i}",
                _far_point(rng, 0.8, 0.8, width - 0.8, length - 0.8, z0, s.start, 5.0),
            )
        )
    return s


FAMILIES: dict[str, Callable[[int], Scene]] = {"office": office}


def generate(family: str, seed: int) -> Scene:
    return FAMILIES[family](seed)
