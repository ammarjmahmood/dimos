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

import json
from pathlib import Path

import pytest
from pytest_mock import MockerFixture

from dimos.agents.mcp.mcp_adapter import McpAdapter
from dimos.e2e_tests.dimos_cli_call import DimosCliCall
from dimos.evals.environments.r1pro_scene import R1ProScene

EXPECTED = {"object_1": {"kind": "cup", "color": "gray", "on": "display_table"}}


def _scene(on: str = "display_table") -> dict[str, object]:
    return {
        "objects": [
            {"id": "object_1", "kind": "cup", "color": "gray", "on": on, "inside": False},
        ],
        "held_objects": {"left": None, "right": None},
        "tray": {"station": "worktable", "held": False, "cargo": []},
        "base_pose": [0.0, 0.0, 0.0],
        "action": {"state": "idle"},
    }


def test_launch_pins_the_seed_and_headless_on_the_r1pro_sim(mocker: MockerFixture) -> None:
    from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
    from dimos.robot.galaxea.r1pro.open_space_blueprint import r1pro_classical_open_space_sim

    mocker.patch.object(McpAdapter, "wait_for_ready", return_value=False)
    proc = DimosCliCall()
    R1ProScene(seed=5001).configure_launch(proc)
    parsed = BlueprintConfigParser(r1pro_classical_open_space_sim).parse(environ=proc.extra_env)
    sim = parsed.module_kwargs("r1proopenspacesim")
    assert sim["seed"] == 5001
    assert sim["headless"] is True  # the environment beats the blueprint's viewer


def test_refuses_to_launch_beside_a_running_dimos(mocker: MockerFixture) -> None:
    mocker.patch.object(McpAdapter, "wait_for_ready", return_value=True)
    with pytest.raises(RuntimeError, match="already serving MCP"):
        R1ProScene().configure_launch(DimosCliCall())


def test_a_drifted_layout_fails_before_the_agent_starts() -> None:
    env = R1ProScene(expected_objects=EXPECTED)
    env._check_layout(_scene())
    with pytest.raises(RuntimeError, match="regenerate the suite's layouts"):
        env._check_layout(_scene(on="low_bench"))


def test_samples_keep_only_what_graders_read(tmp_path: Path) -> None:
    env = R1ProScene()
    env._truth = tmp_path / "scene_truth.jsonl"
    env._record(_scene())
    env._record({"error": "not ready"})  # no objects: skipped
    lines = env._truth.read_text().splitlines()
    assert len(lines) == 1
    sample = json.loads(lines[0])
    assert set(sample) == {
        "t",
        "sim_time",
        "objects",
        "held_objects",
        "tray",
        "base_pose",
        "action",
        "error",
    }
    assert sample["objects"][0]["on"] == "display_table"
