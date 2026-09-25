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
from typing import Any

import pytest
from pytest_mock import MockerFixture

from dimos.evals.agents import jev_planner
from dimos.evals.agents.jev_planner import JevPlanner, next_actions, world_state
from dimos.evals.types import RunningEnvironment

SURFACES = {
    "worktable": {"center": [0.46, 0.0, 0.7]},
    "low_bench": {"center": [-3.2, 2.5, 0.6]},
    "tall_table": {"center": [3.3, 2.5, 0.9]},
}


def _scene(
    *, left: str | None = None, right: str | None = None, tray_held: bool = False
) -> dict[str, Any]:
    objects = [
        {"id": "object_1", "kind": "cup", "color": "gray", "on": "tall_table", "inside": False},
        {"id": "object_2", "kind": "bottle", "color": "blue", "on": "low_bench", "inside": False},
        {
            "id": "object_3",
            "kind": "drink_carton",
            "color": "purple",
            "on": "worktable",
            "inside": False,
        },
    ]
    for row in objects:
        if row["id"] in (left, right):
            row["on"] = None
    return {
        "objects": objects,
        "held_objects": {"left": left, "right": right},
        "tray": {"station": None if tray_held else "worktable", "held": tray_held, "cargo": []},
        "base_pose": [0.0, 0.0, 0.0],  # beside the worktable
    }


def test_a_full_hand_can_place_but_not_pick() -> None:
    options = next_actions(_scene(left="object_1"), SURFACES)
    assert "place_left_on_low_bench" in options and "place_left_in_tray" in options
    assert not any(key.startswith("pick_") and key.endswith("_with_left") for key in options)
    assert "pick_object_2_with_right" in options and "pick_object_1_with_right" not in options
    assert "pick_up_tray" not in options  # a hand is full
    assert "go_to_worktable" not in options  # already here
    assert options["finished"][0] == ""


def test_holding_the_tray_offers_only_tray_moves() -> None:
    options = next_actions(_scene(tray_held=True), SURFACES)
    # tall_table is too high for the carried tray; display_table is not in this scene.
    assert set(options) == {
        "go_to_worktable",
        "go_to_low_bench",
        "put_down_tray_on_worktable",
        "put_down_tray_on_low_bench",
        "finished",
    }


def test_world_state_is_words_without_coordinates() -> None:
    state = world_state("pick the cup", _scene(right="object_2"), SURFACES, [])
    assert state["robot"] == {
        "beside": "worktable",
        "left_hand": "empty",
        "right_hand": "blue bottle",
        "holding_tray": False,
    }
    assert {"item": "blue bottle", "where": "in the right hand"} in state["objects"]

    def numbers(value: Any) -> list[float]:
        if isinstance(value, dict):
            return [n for v in value.values() for n in numbers(v)]
        if isinstance(value, list):
            return [n for v in value for n in numbers(v)]
        return [value] if isinstance(value, float) else []

    assert numbers(state) == []


def test_preflight_needs_the_api_key(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv(jev_planner.API_KEY_ENV, raising=False)
    with pytest.raises(RuntimeError, match="TYPESAFE_API_KEY"):
        JevPlanner().preflight(object())  # type: ignore[arg-type]


def test_run_asks_runs_and_stops_at_finished(
    tmp_path: Path, mocker: MockerFixture, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv(jev_planner.API_KEY_ENV, "test-key")
    scene = _scene()
    replies = iter(["go_to_low_bench", "finished"])
    asked: list[dict[str, Any]] = []

    def call_json(mcp: Any, tool: str, arguments: Any = None) -> dict[str, Any]:
        return SURFACES if tool == "get_surfaces" else scene

    def post(self: Any, url: str, json: dict[str, Any], timeout: float) -> Any:
        asked.append(json)
        choice = next(replies)
        return mocker.Mock(
            status_code=200,
            json=lambda: {
                "model": "jev-1",
                "answers": {"next_action": {"type": "choice", "choice": choice, "confidence": 0.9}},
                "usage": {"input_tokens": 100, "output_tokens": 3},
            },
        )

    mocker.patch.object(jev_planner, "call_json", side_effect=call_json)
    run_action = mocker.patch.object(
        jev_planner, "run_action", return_value={"state": "completed", "success": True}
    )
    mocker.patch("requests.Session.post", post)
    env = RunningEnvironment(mcp_url="http://127.0.0.1:1/mcp", streams=(), artifacts={})
    trajectory = JevPlanner().run("go to the low bench", env, tmp_path, timeout_s=60.0)

    run_action.assert_called_once_with(
        mocker.ANY, "go_to", {"destination": "low_bench"}, timeout_s=mocker.ANY
    )
    assert trajectory.extra.ended_by == "answer"
    assert [s.tool_calls[0].function_name for s in trajectory.steps if s.tool_calls] == ["go_to"]
    assert trajectory.final_metrics.total_prompt_tokens == 200
    assert asked[1]["state"]["done_so_far"] == ["go_to_low_bench: completed"]
    assert "go_to_low_bench" in asked[0]["questions"]["next_action"]["criteria"]
    assert json.loads((tmp_path / "raw" / "001-request.json").read_text())["body"] == asked[0]
