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
    """Rolling, noise-free geometry. Downsampling retains complete four-laser groups."""

    point_rate_hz: int = 200_000
    motion_sample_rate_hz: float = 200.0
    downsample: int = 1
    min_range: float = 0.1
    max_range: float = 40.0

    def __post_init__(self) -> None:
        if self.point_rate_hz <= 0 or self.downsample < 1:
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
