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

"""Grader smoke for tidy-table MuJoCo cases (no live sim)."""

from dimos.evals.suites.mujoco_restore import SUITE, TABLE_CENTER, at_xy, stayed_xy


def test_at_xy_scores_table_center_with_a_tenth_meter_band() -> None:
    center = dict(target=TABLE_CENTER, start_z=0.19, end_z=0.19, band=0.10)
    assert at_xy(TABLE_CENTER, **center) == 1.0
    assert at_xy((0.45, -0.05), **center) == 0.5
    assert at_xy((0.45, -0.10), **center) == 0.0
    assert at_xy((0.45, -0.11), **center) == 0.0
    assert at_xy(TABLE_CENTER, **(center | {"end_z": 0.06})) == 0.0
    assert at_xy((0.45, 0.22), **center) == 0.0


def test_stayed_xy_scores_one_when_unmoved_and_zero_when_shifted() -> None:
    start = TABLE_CENTER
    z = dict(start_z=0.19, end_z=0.19, band=0.10)
    assert stayed_xy(start, start, **z) == 1.0
    assert stayed_xy(start, (0.45, -0.10), **z) == 0.0
    assert stayed_xy(start, (0.45, 0.22), **z) == 0.0


def test_suite_pairs_a_messy_table_with_an_already_tidy_control() -> None:
    assert [case.id for case in SUITE] == ["xarm_restore_cup", "xarm_table_already_tidy"]
    assert SUITE[0].inputs == SUITE[1].inputs
    assert "0.10" not in SUITE[0].inputs
    assert "0.45" not in SUITE[0].inputs
    assert "middle of the table" in SUITE[0].inputs
    assert "cleaning arm" in SUITE[0].inputs
