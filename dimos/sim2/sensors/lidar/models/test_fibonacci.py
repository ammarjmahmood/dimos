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

from dimos.sim2.sensors.lidar.models.fibonacci import Fibonacci


def test_rays_are_unit_length_and_evenly_spaced_in_solid_angle():
    rays = Fibonacci(ray_count=4, elevation_min=-90, elevation_max=90).directions()

    assert np.linalg.norm(rays, axis=1) == pytest.approx(np.ones(4))
    assert rays[:, 2] == pytest.approx([-0.75, -0.25, 0.25, 0.75])
    azimuth = np.arange(4) * np.pi * (3 - np.sqrt(5))
    assert rays[:, :2] == pytest.approx(
        np.sqrt(1 - rays[:, 2, None] ** 2) * np.column_stack((np.cos(azimuth), np.sin(azimuth)))
    )
