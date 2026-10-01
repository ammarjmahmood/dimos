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

"""The xArm MuJoCo suite ships each task as a raw-camera and a perception variant."""

from __future__ import annotations

from types import FunctionType
from typing import cast

from dimos.evals.environments.mujoco_sim import MujocoEnvironment
from dimos.evals.suites.mujoco_xarm import PERCEPTION_MODULES, SUITE, TRACKED
from dimos.evals.types import EvalCase

TASKS = ("xarm_pick_cylinder", "xarm_ball_on_cylinder")


def _grader(case: EvalCase) -> tuple[str, list[object]]:
    """The grader factory and the arguments it was built with."""
    fn = cast("FunctionType", case.grade)
    return fn.__qualname__, [cell.cell_contents for cell in fn.__closure__ or ()]


def test_each_task_has_a_raw_and_a_perception_variant() -> None:
    by_id = {case.id: case for case in SUITE}
    assert list(by_id) == [*TASKS, *(f"{task}_perception" for task in TASKS)]
    for task in TASKS:
        raw, perception = by_id[task], by_id[f"{task}_perception"]
        assert raw.tags >= {"mujoco", "manipulation", "pick", "raw"}
        assert perception.tags == (raw.tags - {"raw"}) | {"perception"}
        # Same task, grader, pass bar and budget; only the launch and the guidance differ.
        assert perception.inputs.startswith(raw.inputs)
        assert "scan_objects" in perception.inputs and "scan_objects" not in raw.inputs
        assert _grader(perception) == _grader(raw)
        assert (perception.timeout_s, perception.threshold) == (raw.timeout_s, raw.threshold)
    assert "place" in by_id["xarm_ball_on_cylinder_perception"].tags
    assert "place" not in by_id["xarm_pick_cylinder_perception"].tags


def test_perception_variant_keeps_the_perception_modules() -> None:
    for case in SUITE:
        assert isinstance(case.environment, MujocoEnvironment)
        assert case.environment.config.blueprint[0] == "xarm-perception-sim"
        assert case.environment.config.tracked_bodies == TRACKED
        expected = () if "perception" in case.tags else PERCEPTION_MODULES
        assert case.environment.config.disable == expected, case.id
    assert len({id(case.environment) for case in SUITE}) == len(SUITE)


def test_variant_tags_select_exactly_one_variant() -> None:
    """``--tags`` keeps a case when any selected tag is on it (``EvalRunner.run``)."""
    for tag in ("raw", "perception"):
        selected = [case.id for case in SUITE if frozenset({tag}) & case.tags]
        assert selected == [case.id for case in SUITE if tag in case.tags]
        assert len(selected) == len(TASKS), tag
        assert all((tag == "perception") == name.endswith("_perception") for name in selected)
