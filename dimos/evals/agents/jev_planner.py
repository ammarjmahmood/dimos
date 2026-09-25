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

"""TypeSafe's Jev choosing the R1 Pro's next classical skill, one question per turn.

Jev answers multiple-choice questions about a JSON state; it does not call
tools or produce coordinates. Each turn this agent reads ``get_scene``, lists
the skill calls possible right now (the same skills every other agent calls),
asks Jev which one moves the instruction forward, runs it and repeats until Jev
picks ``finished``. The state is in words: Jev is unreliable at comparing
numbers (docs.typesafe.ai/model-jaggedness).
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import time
from typing import TYPE_CHECKING, Any

import requests

from dimos.agents.mcp.mcp_adapter import McpAdapter
from dimos.evals.agents.base import Agent, AgentConfig
from dimos.evals.agents.lib.trajectory_builder import TrajectoryBuilder
from dimos.evals.environments.lib.r1pro_actions import call_json, run_action
from dimos.evals.types import EndedBy, Metrics, RunningEnvironment, ToolCall, Trajectory

if TYPE_CHECKING:
    from dimos.evals.environments.base import Environment

API_KEY_ENV = "TYPESAFE_API_KEY"
ARMS = ("left", "right")
# The carried tray clears only platforms below about 80 cm.
TRAY_PLATFORMS = ("worktable", "low_bench", "display_table")
NEAR_PLATFORM_M = 1.4
_RETRY_STATUSES = frozenset({429, 500, 502, 503, 504, 529})

QUESTION = (
    "Which single action should the robot take next to carry out `instruction`? "
    "`robot` says where it is and what each hand holds, `objects` where every item is, "
    "`tray` where the tray is and what is in it, and `done_so_far` what already happened, "
    "including refused or failed actions and why. A hand must be empty before it can pick. "
    "Answer finished only when every part of the instruction is done."
)


class JevPlannerConfig(AgentConfig):
    model: str = "jev-latest"
    # TYPESAFE_BASE_URL, when set, wins over this.
    base_url: str = "https://api.typesafe.ai"
    # Questions asked before giving up; each is followed by at most one skill.
    max_turns: int = 40
    # Seconds one skill may move before it is stopped.
    action_timeout_s: float = 900.0
    request_timeout_s: float = 30.0


class JevPlanner(Agent):
    """Drive the R1 Pro classical skills with Jev picking each next call."""

    config: JevPlannerConfig

    def available_tools(self, environment_tools: tuple[str, ...]) -> tuple[str, ...]:
        return environment_tools

    def preflight(self, environment: Environment) -> None:
        if not os.environ.get(API_KEY_ENV):
            raise RuntimeError(f"JevPlanner needs {API_KEY_ENV}")
        if not environment.has_robot:
            raise RuntimeError(f"JevPlanner drives a robot; {type(environment).__name__} has none")

    def run(
        self, inputs: str, env: RunningEnvironment, run_dir: Path, *, timeout_s: float
    ) -> Trajectory:
        mcp = McpAdapter(env.mcp_url)
        raw = run_dir / "raw"
        raw.mkdir(parents=True, exist_ok=True)
        trajectory = TrajectoryBuilder(inputs, name=type(self).__name__, model=self.config.model)
        surfaces: dict[str, Any] = call_json(mcp, "get_surfaces")
        history: list[str] = []
        deadline = time.monotonic() + timeout_s
        ended: EndedBy = "max_steps"
        with requests.Session() as http:
            http.headers["Authorization"] = f"Bearer {os.environ[API_KEY_ENV]}"
            for turn in range(1, self.config.max_turns + 1):
                if time.monotonic() >= deadline:
                    ended = "timeout"
                    break
                scene = call_json(mcp, "get_scene")
                options = next_actions(scene, surfaces)
                body = {
                    "model": self.config.model,
                    "state": world_state(inputs, scene, surfaces, history),
                    "questions": {
                        "next_action": {
                            "type": "choice",
                            "instructions": QUESTION,
                            "criteria": {key: text for key, (_, _, text) in options.items()},
                        }
                    },
                }
                request = raw / f"{turn:03d}-request.json"
                response = raw / f"{turn:03d}-response.json"
                started = time.time()
                request.write_text(json.dumps({"body": body, "started_at": started}, indent=2))
                reply = self._ask(http, body)
                response.write_text(json.dumps(reply, indent=2))
                answer = reply["answers"]["next_action"]
                pick = str(answer["choice"])
                usage = reply.get("usage") or {}
                metrics = Metrics(
                    prompt_tokens=int(usage.get("input_tokens", 0)),
                    completion_tokens=int(usage.get("output_tokens", 0)),
                )
                note = f"{pick} (confidence {float(answer.get('confidence', 0.0)):.2f})"
                if pick == "finished" or pick not in options:
                    trajectory.step(
                        message=note if pick == "finished" else f"unknown choice {pick!r}",
                        request=request,
                        response=response,
                        metrics=metrics,
                        model_name=str(reply.get("model") or self.config.model),
                        latency_s=time.time() - started,
                        at=started,
                    )
                    ended = "answer" if pick == "finished" else "error"
                    break
                tool, arguments, _ = options[pick]
                call = ToolCall(
                    tool_call_id=f"call_{turn}", function_name=tool, arguments=arguments
                )
                trajectory.step(
                    message=note,
                    request=request,
                    response=response,
                    tool_calls=(call,),
                    metrics=metrics,
                    model_name=str(reply.get("model") or self.config.model),
                    latency_s=time.time() - started,
                    at=started,
                )
                try:
                    outcome = run_action(
                        mcp,
                        tool,
                        arguments,
                        timeout_s=min(
                            self.config.action_timeout_s, max(0.0, deadline - time.monotonic())
                        ),
                    )
                except Exception as e:
                    outcome = {"state": "error", "success": False, "error": repr(e)}
                trajectory.observe(call.tool_call_id, json.dumps(outcome))
                history.append(_summarize(pick, outcome))
        return trajectory.build(ended)

    def _ask(self, http: requests.Session, body: dict[str, Any]) -> dict[str, Any]:
        url = os.environ.get("TYPESAFE_BASE_URL", self.config.base_url) + "/v1/systemone"
        for attempt in range(3):
            resp = http.post(url, json=body, timeout=self.config.request_timeout_s)
            if resp.status_code in _RETRY_STATUSES and attempt < 2:
                time.sleep(float(resp.headers.get("retry-after", 0.5 * 2**attempt)))
                continue
            if resp.status_code >= 400:
                raise RuntimeError(f"TypeSafe {resp.status_code}: {resp.text[:300]}")
            reply: dict[str, Any] = resp.json()
            return reply
        raise AssertionError("unreachable")


def _name(row: dict[str, Any]) -> str:
    return f"{row['color']} {str(row['kind']).replace('_', ' ')}"


def _near(scene: dict[str, Any], surfaces: dict[str, Any]) -> str | None:
    """The platform the robot stands beside, if any."""
    x, y = scene["base_pose"][:2]
    best = min(
        surfaces,
        key=lambda name: math.hypot(
            surfaces[name]["center"][0] - x, surfaces[name]["center"][1] - y
        ),
        default=None,
    )
    if best is None:
        return None
    cx, cy = surfaces[best]["center"][:2]
    return best if math.hypot(cx - x, cy - y) <= NEAR_PLATFORM_M else None


def world_state(
    instruction: str, scene: dict[str, Any], surfaces: dict[str, Any], history: list[str]
) -> dict[str, Any]:
    """What Jev sees: the instruction and the scene in words, no coordinates."""
    held = scene["held_objects"]
    rows = {row["id"]: row for row in scene["objects"]}
    tray = scene["tray"]

    def where(row: dict[str, Any]) -> str:
        arm = next((a for a in ARMS if held.get(a) == row["id"]), None)
        if arm:
            return f"in the {arm} hand"
        if row.get("inside"):
            return "in the tray"
        return f"on the {row['on']}" if row.get("on") else "on the floor"

    return {
        "instruction": instruction,
        "robot": {
            "beside": _near(scene, surfaces) or "no platform",
            "left_hand": _name(rows[held["left"]]) if held.get("left") else "empty",
            "right_hand": _name(rows[held["right"]]) if held.get("right") else "empty",
            "holding_tray": bool(tray["held"]),
        },
        "objects": [{"item": _name(row), "where": where(row)} for row in scene["objects"]],
        "tray": {
            "where": "in both hands" if tray["held"] else f"on the {tray['station']}",
            "contents": [_name(rows[i]) for i in tray.get("cargo", []) if i in rows],
            "fits_on": [p for p in TRAY_PLATFORMS if p in surfaces],
        },
        "platforms": list(surfaces),
        "done_so_far": history or ["nothing yet"],
    }


def next_actions(
    scene: dict[str, Any], surfaces: dict[str, Any]
) -> dict[str, tuple[str, dict[str, Any], str]]:
    """Every skill call that can start now: key -> (tool, arguments, what it does).

    Only preconditions the skills themselves enforce are applied here, so an
    impossible choice never reaches the robot; choosing among the rest is Jev's.
    """
    held = scene["held_objects"]
    tray = scene["tray"]
    rows = {row["id"]: row for row in scene["objects"]}
    here = _near(scene, surfaces)
    say = {name: name.replace("_", " ") for name in surfaces}
    options: dict[str, tuple[str, dict[str, Any], str]] = {}
    if tray["held"]:
        for p in (p for p in TRAY_PLATFORMS if p in surfaces):
            options[f"go_to {p}"] = ("go_to", {"destination": p}, f"carry the tray to the {say[p]}")
            options[f"put_down_tray {p}"] = (
                "put_down_tray",
                {"region": p},
                f"set the tray down on the {say[p]}, freeing both hands",
            )
    else:
        for p in surfaces:
            if p != here:
                options[f"go_to {p}"] = (
                    "go_to",
                    {"destination": p},
                    f"drive to the {say[p]}, keeping whatever the hands hold",
                )
        loose = [
            row for row in scene["objects"] if row["id"] not in held.values() and row.get("on")
        ]
        for arm in ARMS:
            if held.get(arm):
                item = _name(rows[held[arm]])
                for p in surfaces:
                    options[f"place {arm} {p}"] = (
                        "place_object",
                        {"region": p, "arm": arm},
                        f"put the {item} from the {arm} hand down on the {say[p]}",
                    )
                if tray["station"] is not None:
                    options[f"place {arm} tray"] = (
                        "place_object",
                        {"region": "tray", "arm": arm},
                        f"put the {item} from the {arm} hand into the tray",
                    )
            else:
                for row in loose:
                    options[f"pick {row['id']} {arm}"] = (
                        "pick_object",
                        {"object": row["id"], "arm": arm},
                        f"pick up the {_name(row)} ({'in the tray' if row.get('inside') else 'on the ' + str(row['on'])}) "
                        f"with the {arm} hand and hold it",
                    )
        if not any(held.get(a) for a in ARMS) and tray["station"] is not None:
            options["pick_up_tray"] = (
                "pick_up_tray",
                {},
                "lift the tray and everything in it with both hands",
            )
    options["finished"] = ("", {}, "every part of the instruction is done; stop")
    return options


def _summarize(pick: str, outcome: dict[str, Any]) -> str:
    state = outcome.get("state", "unknown")
    reason = outcome.get("error") or outcome.get("reason")
    return f"{pick}: {state}" + (
        f" ({str(reason)[:160]})" if reason and state != "completed" else ""
    )
