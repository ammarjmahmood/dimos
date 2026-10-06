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
from numpy.typing import NDArray
import pytest

from dimos.simulation.sensors.mid360.lidar import OcclusionMap, SimMid360
from dimos.simulation.sensors.mid360.pattern import Mid360Pattern
from dimos.utils.data import get_data


@pytest.fixture(scope="module")
def pattern() -> Mid360Pattern:
    return Mid360Pattern.load(get_data("go2_sim") / "mid360_pattern.npz")


class Sphere:
    """A sphere of the given radius around the sensor, so every ray hits it head on."""

    def __init__(self, radius: float) -> None:
        self.radius = radius

    def cast(
        self, origin: NDArray[np.float64], directions: NDArray[np.float64], max_range: float
    ) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        return np.full(len(directions), self.radius), -directions


def test_pattern_covers_the_measured_elevation_band(pattern: Mid360Pattern) -> None:
    d = pattern.directions(0, 200_000)
    assert np.allclose(np.linalg.norm(d, axis=1), 1.0)
    el = np.degrees(np.arcsin(d[:, 2]))
    assert -9.0 < el.min() < -6.0
    assert 51.0 < el.max() < 54.0


def test_pattern_is_independent_of_batching(pattern: Mid360Pattern) -> None:
    whole = pattern.directions(1001, 800)
    parts = np.concatenate([pattern.directions(1001, 333), pattern.directions(1334, 467)])
    assert np.allclose(whole, parts)


def test_returns_follow_range_with_millimeter_noise(pattern: Mid360Pattern) -> None:
    lidar = SimMid360(Sphere(3.0), pattern, seed=1)
    scan = lidar.cast(np.zeros(3), np.eye(3), 20_000, 0.1)
    r = np.linalg.norm(scan.points, axis=1)
    assert len(r) == 20_000
    assert abs(float(np.median(r)) - 3.0) < 0.001
    assert 0.003 < float(np.std(r)) < 0.008


def test_point_times_end_at_the_batch_end(pattern: Mid360Pattern) -> None:
    scan = SimMid360(Sphere(3.0), pattern, seed=1).cast(np.zeros(3), np.eye(3), 400, 0.1)
    assert scan.times[-1] == pytest.approx(0.1)
    assert scan.times[0] == pytest.approx(0.1 - 399 / 200_000)


def test_blind_zone_drops_near_returns(pattern: Mid360Pattern) -> None:
    lidar = SimMid360(Sphere(0.1), pattern, seed=1)
    assert len(lidar.cast(np.zeros(3), np.eye(3), 5_000, 0.1).points) == 0


def test_occlusion_map_blocks_rays(pattern: Mid360Pattern) -> None:
    blocked = OcclusionMap(np.ones((64, 360), np.float32), -180.0, -8.0)
    lidar = SimMid360(Sphere(3.0), pattern, seed=1, occlusion=blocked)
    assert len(lidar.cast(np.zeros(3), np.eye(3), 5_000, 0.1).points) == 0


def test_same_seed_same_scan(pattern: Mid360Pattern) -> None:
    a = SimMid360(Sphere(3.0), pattern, seed=7).cast(np.zeros(3), np.eye(3), 4_000, 0.1)
    b = SimMid360(Sphere(3.0), pattern, seed=7).cast(np.zeros(3), np.eye(3), 4_000, 0.1)
    assert np.array_equal(a.points, b.points)
