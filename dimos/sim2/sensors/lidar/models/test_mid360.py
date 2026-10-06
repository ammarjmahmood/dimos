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

import numpy as np
import pytest

from dimos.sim2.sensors.lidar.models.mid360 import Mid360


@pytest.fixture
def pattern(mocker):
    angles = np.tile(
        np.array([[0, 9000], [9000, 9000], [18000, 9000], [27000, 9000]], dtype=np.uint16),
        (200_000, 1),
    )
    angles[20_000:40_000, 1] = 4500
    mocker.patch("dimos.sim2.sensors.lidar.models.mid360._pattern", return_value=angles)
    return Mid360()


def test_scan_preserves_official_phase_channels_and_capture_times(pattern):
    first = pattern.scan(0, 0.1)
    second = pattern.scan(0.1, 0.1)
    assert len(first.offsets) == 20_000
    assert first.offsets[[0, 1, -1]] == pytest.approx([0, 0.000005, 0.099995])
    assert first.lines[:8].tolist() == [0, 1, 2, 3, 0, 1, 2, 3]
    np.testing.assert_allclose(
        first.directions[:4], [[1, 0, 0], [0, 1, 0], [-1, 0, 0], [0, -1, 0]], atol=1e-12
    )
    assert second.directions[:, 2] == pytest.approx(np.sqrt(0.5))
    np.testing.assert_array_equal(pattern.scan(4, 0.1).directions, first.directions)


def test_downsampling_keeps_four_lasers_and_original_time_offsets(pattern):
    rays = Mid360(downsample=8).scan(0, 0.1)
    assert len(rays.offsets) == 2500
    assert rays.lines[:8].tolist() == [0, 1, 2, 3, 0, 1, 2, 3]
    assert rays.offsets[4] == 32 / 200_000
    assert rays.offsets[-1] > 0.099


@pytest.mark.parametrize("duration", [0, -0.1, 0.100001, 0.000005])
def test_scan_rejects_partial_channel_groups(pattern, duration):
    with pytest.raises(ValueError, match="four-laser"):
        pattern.scan(0, duration)


@pytest.mark.parametrize(
    "kwargs",
    [
        {"downsample": 0},
        {"motion_sample_rate_hz": 0},
        {"max_range": 0},
        {"min_range": float("nan")},
    ],
)
def test_invalid_sensor_parameters_are_rejected(kwargs):
    with pytest.raises(ValueError):
        Mid360(**kwargs)
