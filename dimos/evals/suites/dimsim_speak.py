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

"""Go to the person standing by the bed and speak to them.

The mesh is Quaternius's Casual Character (CC0) at
``misc/DimSim/scenes/apartment/person.glb``. Idle_Neutral keeps them standing.
Full credit is a successful ``speak`` while the robot is already within 2 m.
The person cannot report that they heard it.

    dimos evals run dimos.evals.suites.dimsim_speak --agent dimos.evals.agents.mcp_client_adapter
"""

from __future__ import annotations

from datetime import datetime
from typing import TYPE_CHECKING

from dimos.evals.environments.dimsim import DimSimEnvironment
from dimos.evals.scorers import ramp
from dimos.evals.types import EvalCase, Outcome, Suite, recording
from dimos.msgs.geometry_msgs.Vector3 import Vector3

if TYPE_CHECKING:
    from dimos.e2e_tests.dim_sim_client import DimSimClient
    from dimos.memory.store.base import Store

# Scene Y-up (x, y, z) is published as odom (z, x). Feet on the floor.
_SCENE = (-1.332, 0.0, -3.567)
PERSON = Vector3(_SCENE[2], _SCENE[0], 0.0)
_PERSON_URL = "/scenes/apartment/person.glb"
_NEAR_M = 2.0
_BLUEPRINT = ["unitree-go2", "mcp-server", "unitree-skill-container", "speak-skill", "mcp-client"]


def _place_person(sim: DimSimClient) -> None:
    sim.client.add_npc(
        _PERSON_URL,
        name="person",
        position=_SCENE,
        scale=1.0,
        animation="Idle_Neutral",
    )


def _speak_succeeded(content: str) -> bool:
    text = content.lstrip()
    return bool(text) and not text.startswith(("Error", "Warning"))


def _speak_times(outcome: Outcome) -> list[float]:
    times: list[float] = []
    for step in outcome.trajectory.steps:
        calls = {
            call.tool_call_id for call in step.tool_calls or () if call.function_name == "speak"
        }
        if not calls or step.observation is None or not step.timestamp:
            continue
        if not any(
            result.source_call_id in calls and _speak_succeeded(result.content)
            for result in step.observation.results
        ):
            continue
        try:
            times.append(datetime.fromisoformat(step.timestamp).timestamp())
        except ValueError:
            continue
    return times


def _position_at(store: Store, when: float) -> Vector3 | None:
    latest = None
    for sample in store.streams.odom:
        if sample.ts <= when and (latest is None or sample.ts >= latest.ts):
            latest = sample
    if latest is None:
        return None
    return latest.data.position


def spoke_nearby(outcome: Outcome) -> float:
    """1.0 for a successful speak within 2 m, 0.0 if it failed or was 4 m away."""
    times = _speak_times(outcome)
    if not times:
        return 0.0
    with recording(outcome) as store:
        try:
            positions = [_position_at(store, when) for when in times]
        except (LookupError, AttributeError):
            return 0.0
    distances = [
        Vector3(position.x - PERSON.x, position.y - PERSON.y, 0.0).length()
        for position in positions
        if position is not None
    ]
    if not distances:
        return 0.0
    return ramp(max(0.0, min(distances) - _NEAR_M), band=_NEAR_M)


speak_to_person = EvalCase(
    id="dimsim_speak_to_person",
    inputs="A person is standing by the bed. Go to them and speak to them.",
    environment=DimSimEnvironment(
        blueprint=_BLUEPRINT,
        scene="apartment",
        setup=_place_person,
    ),
    grade=spoke_nearby,
    timeout_s=180.0,
    tags=frozenset({"nav", "speak"}),
)

SUITE: Suite = [speak_to_person]
