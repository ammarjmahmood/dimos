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

"""Grader smoke for speaking to a person on the bed (no live DimSim)."""

from pathlib import Path

from dimos.evals.suites.dimsim_speak import PERSON, SUITE, spoke_nearby
from dimos.evals.types import (
    AgentInfo,
    FinalMetrics,
    Outcome,
    RunExtra,
    Step,
    ToolCall,
    Trajectory,
)
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import make_vector3


def _recording(path: Path, x: float, y: float) -> Path:
    with SqliteStore(path=str(path)) as store:
        store.stream("odom", PoseStamped).append(
            PoseStamped(
                position=make_vector3(x, y, 0.0),
                orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
                frame_id="world",
            ),
            ts=1.0,
        )
    return path


def _outcome(path: Path, *, spoke: bool) -> Outcome:
    calls = (
        (ToolCall(tool_call_id="c1", function_name="speak", arguments={"text": "Hello."}),)
        if spoke
        else None
    )
    return Outcome(
        trajectory=Trajectory(
            agent=AgentInfo(name="test", version="1", model_name="test"),
            steps=(Step(step_id=1, timestamp="", source="agent", message="", tool_calls=calls),),
            final_metrics=FinalMetrics(
                total_prompt_tokens=0,
                total_completion_tokens=0,
                total_cached_tokens=0,
                total_cost_usd=0,
                total_steps=1,
            ),
            extra=RunExtra(ended_by="answer"),
        ),
        artifacts={"recording": path},
    )


def test_spoke_nearby_needs_both_the_person_and_a_speak_call(tmp_path: Path) -> None:
    near = _recording(tmp_path / "near.db", PERSON.x, PERSON.y)
    far = _recording(tmp_path / "far.db", PERSON.x + 10.0, PERSON.y)
    assert spoke_nearby(_outcome(near, spoke=True)) == 1.0
    assert spoke_nearby(_outcome(near, spoke=False)) == 0.0
    assert spoke_nearby(_outcome(far, spoke=True)) == 0.0


def test_suite_sends_the_robot_to_the_person_and_asks_it_to_speak() -> None:
    assert len(SUITE) == 1
    case = SUITE[0]
    assert case.id == "dimsim_speak_to_person"
    assert case.environment.config.blueprint == [
        "unitree-go2",
        "mcp-server",
        "unitree-skill-container",
        "speak-skill",
        "mcp-client",
    ]
    text = case.inputs.lower()
    assert "person" in text and "speak" in text
    assert "-3.57" not in case.inputs
