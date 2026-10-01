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

"""The sim2 xArm suite is the MuJoCo suite on sim2, graded from ``sim_truth``."""

from __future__ import annotations

from pathlib import Path
from types import FunctionType
from typing import cast

from dimos.evals.environments.sim2 import Sim2Environment
from dimos.evals.suites import mujoco_xarm
from dimos.evals.suites.sim2_xarm import BLUEPRINTS, SUITE, TRUTH_POSITIONS, XARM_TABLE
from dimos.evals.types import EvalCase
from dimos.memory.store.memory import MemoryStore
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.sim2.scene_types import EntityState, SceneState


def _closure(case: EvalCase) -> dict[str, object]:
    fn = cast("FunctionType", case.grade)
    cells = [c.cell_contents for c in fn.__closure__ or ()]
    return dict(zip(fn.__code__.co_freevars, cells, strict=True))


def test_both_variants_with_the_mujoco_prompts() -> None:
    mujoco = {case.id: case for case in mujoco_xarm.SUITE}
    assert [case.id for case in SUITE] == list(mujoco)
    for case in SUITE:
        variant = "perception" if case.id.endswith("_perception") else "raw"
        assert case.tags == (mujoco[case.id].tags - {"mujoco"}) | {"sim2"}
        assert variant in case.tags
        assert case.inputs == mujoco[case.id].inputs
        assert (case.timeout_s, case.threshold) == (
            mujoco[case.id].timeout_s,
            mujoco[case.id].threshold,
        )
    assert "place" in SUITE[1].tags and "place" not in SUITE[0].tags
    assert "place" in SUITE[3].tags and "place" not in SUITE[2].tags


def test_cases_launch_the_table_scene_with_truth() -> None:
    assert (XARM_TABLE / "scene.xml").is_file() and (XARM_TABLE / "scene.json").is_file()
    for case in SUITE:
        env = case.environment
        assert isinstance(env, Sim2Environment)
        variant = "perception" if "perception" in case.tags else "raw"
        assert env.config.blueprint == BLUEPRINTS[variant]
        assert ("xarm-perception-sim2" in env.config.blueprint) == (variant == "perception")
        assert Path(env.config.scene) == XARM_TABLE
        assert env.config.truth_entities == mujoco_xarm.TRACKED
        assert "sim_truth" in env.config.recorded_topics
    assert len({id(case.environment) for case in SUITE}) == len(SUITE)


def test_graders_read_sim_truth() -> None:
    for case in SUITE:
        cells = _closure(case)
        assert cells["last"] is TRUTH_POSITIONS[1]
    with MemoryStore() as store:
        row = SceneState(
            world_id="w",
            scene_id="xarm_table",
            generation=0,
            tick=0,
            sim_time=0.0,
            ts=1.0,
            entities={
                "cup": EntityState(
                    pose=Pose((0.5, 0.0, 0.19), (0.0, 0.0, 0.0, 1.0)),
                    velocity=(0.0, 0.0, 0.0),
                    angular_velocity=(0.0, 0.0, 0.0),
                    bounds_min=(0.0, 0.0, 0.0),
                    bounds_max=(0.0, 0.0, 0.0),
                )
            },
            robots={},
            joints={},
            regions={},
            contacts=(),
        )
        store.stream("sim_truth", SceneState).append(row, ts=1.0)
        first, last = TRUTH_POSITIONS
        assert first(store, "cup").z == last(store, "cup").z == 0.19
