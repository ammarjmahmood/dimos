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

"""Grader smoke for restoring a MuJoCo body to a home offset (no live sim)."""

from dimos.evals.suites.mujoco_restore import restored_offset


def test_restored_scores_one_on_the_table_and_zero_off_it() -> None:
    on_table = dict(dx=0.0, dy=0.10, start=(0.50, 0.0), end=(0.50, 0.10), start_z=0.19, end_z=0.19)
    assert restored_offset(**on_table) == 1.0
    assert restored_offset(**(on_table | {"end": (0.50, 0.0)})) == 0.0
    assert restored_offset(**(on_table | {"end_z": 0.06})) == 0.0
    assert restored_offset(**(on_table | {"end_z": 0.24})) == 0.0
