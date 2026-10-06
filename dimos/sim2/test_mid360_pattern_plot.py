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

from dimos.sim2.demo_mid360_pattern import angular_points


def test_angles_use_laser_axes_and_ignore_range():
    points = np.array([[3, 0, 0], [0, 7, 0], [0, -2, 0], [2, 0, 2]], dtype=float)

    angles = angular_points(points)

    np.testing.assert_allclose(angles, [[0, 0], [90, 0], [-90, 0], [0, 45]])


@pytest.mark.parametrize("points", [[[0, 0, 0]], [[np.nan, 0, 1]], [[0, 1]], []])
def test_invalid_returns_cannot_be_plotted_as_firing_directions(points):
    with pytest.raises(ValueError):
        angular_points(np.asarray(points, dtype=float))
