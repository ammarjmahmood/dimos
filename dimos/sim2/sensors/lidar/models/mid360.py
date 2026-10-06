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

"""PimSim's official four-channel Livox sequence, with explicit acquisition times."""

from dataclasses import dataclass
from functools import lru_cache
import math

import numpy as np
from numpy.typing import NDArray

from dimos.sim2.sensors.spec import TimedRays
from dimos.utils.data import get_data


@lru_cache(maxsize=1)
def _pattern() -> NDArray[np.uint16]:
    with np.load(get_data("mid360_pattern") / "mid360_pattern.npz", allow_pickle=False) as archive:
        angles = np.asarray(archive["angles"], dtype=np.uint16)
    if angles.shape != (800_000, 2):
        raise ValueError("the official Mid360 sequence must contain 800,000 angular pairs")
    angles.setflags(write=False)
    return angles


@dataclass(frozen=True)
class Mid360:
    """One rolling device model; return coefficients from Andrew's PR #4441.

    Noise/dropout are approximations, not a calibrated reflectivity model.
    Disable them for geometry-only diagnostics without changing acquisition.
    """

    point_rate_hz: int = 200_000
    motion_sample_rate_hz: float = 200.0
    downsample: int = 1
    min_range: float = 0.16
    max_range: float = 40.0
    noise: bool = True
    dropout: bool = True
    seed: int = 0

    def __post_init__(self) -> None:
        if self.point_rate_hz <= 0 or self.downsample < 1 or self.seed < 0:
            raise ValueError("Mid360 point rate and downsample must be positive")
        if not all(
            math.isfinite(v) for v in (self.motion_sample_rate_hz, self.min_range, self.max_range)
        ):
            raise ValueError("Mid360 rates and range limits must be finite")
        if self.motion_sample_rate_hz <= 0 or not 0 <= self.min_range < self.max_range:
            raise ValueError("Mid360 requires a positive motion rate and ordered range limits")

    def scan(self, start: float, duration: float) -> TimedRays:
        count = round(duration * self.point_rate_hz)
        if duration <= 0 or not math.isclose(count, duration * self.point_rate_hz) or count % 4:
            raise ValueError("Mid360 scan duration must contain complete four-laser groups")
        offsets = (np.arange(0, count, 4 * self.downsample)[:, None] + np.arange(4)).ravel()
        indices = (round(start * self.point_rate_hz) + offsets) % len(_pattern())
        angles = _pattern()[indices].astype(np.float64) / 100.0
        azimuth = np.deg2rad(angles[:, 0])
        elevation = np.deg2rad(90.0 - angles[:, 1])
        horizontal = np.cos(elevation)
        return TimedRays(
            directions=np.column_stack(
                (horizontal * np.cos(azimuth), horizontal * np.sin(azimuth), np.sin(elevation))
            ),
            offsets=offsets.astype(np.float64) / self.point_rate_hz,
            lines=(indices % 4).astype(np.uint8),
        )

    def measure(
        self, ranges: NDArray[np.float64], cos_incidence: NDArray[np.float64], start: float
    ) -> NDArray[np.float64]:
        # Key each scan independently: skipping a late scan cannot shift later noise.
        rng = np.random.default_rng([self.seed, round(start * self.point_rate_hz)])
        cosine = np.clip(np.abs(cos_incidence), 0.0, 1.0)
        keep = (ranges >= self.min_range) & (ranges <= self.max_range)
        if self.dropout:
            probability = np.interp(
                np.rad2deg(np.arccos(cosine)),
                (0, 78, 79.5, 82.5, 85, 88, 90),
                (1, 1, 0.76, 0.5, 0.22, 0.08, 0),
            )
            keep &= rng.random(len(ranges)) < probability
        result = ranges.copy()
        if self.noise:
            sigma = np.hypot(0.0034, 0.00073 * ranges) / np.maximum(cosine, 0.05) ** 0.78
            result += rng.standard_normal(len(ranges)) * sigma
        keep &= (result >= self.min_range) & (result <= self.max_range)
        result[~keep] = -1
        return result
