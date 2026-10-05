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

"""The same cases as Desktop's Rust diagnose.rs tests."""

from typing import Any

from dimos.server.diagnose import problems, steps

START = (
    "$ dimos --replay run unitree-go2-basic\n"
    "16:37:45.859 [inf][imos/cli/commands/lifecycle.py] Starting DimOS\n"
    "16:37:46.771 [inf][dination/module_coordinator.py] Building the blueprint\n"
    "16:37:46.772 [inf][dination/module_coordinator.py] Starting the modules\n"
    "16:37:48.056 [inf][/coordination/python_worker.py] Deployed module. module=MovementManager\n"
    "16:37:48.195 [inf][/coordination/python_worker.py] Deployed module. module=WebsocketVisModule\n"
)


def states(found: list[dict[str, Any]]) -> list[str]:
    return [step["state"] for step in found]


def test_steps_follow_the_output() -> None:
    starting = steps(START, "starting")
    assert states(starting) == ["done", "done", "now", "todo"]
    assert starting[2]["detail"] == "2 started"
    assert [step["label"] for step in starting] == [
        "Starting dimOS",
        "Building the blueprint",
        "Starting modules",
        "Running",
    ]
    assert states(steps(START, "running")) == ["done", "done", "done", "done"]
    assert steps(START, "stopped")[3]["label"] == "Stopped"
    assert states(steps("$ dimos run x\n", "starting")) == ["now", "todo", "todo", "todo"]
    assert states(steps("$ dimos run x\n", "failed")) == ["failed", "todo", "todo", "todo"]
    assert states(steps(START + "Error: boom\n", "failed")) == ["done", "done", "failed", "todo"]


def test_known_problems_get_their_fix() -> None:
    output = (
        '$ dimos run unitree-g1\nTraceback (most recent call last):\n  File "x.py"\n'
        "ModuleNotFoundError: No module named 'unitree_sdk2py'\n"
    )
    found = problems(output)
    assert len(found) == 1 and "`unitree_sdk2py`" in found[0]["text"] and found[0]["fix"]
    output = "$ dimos run unitree-go2-multi\nValueError: No robot IPs specified. Must have at least one IP.\n"
    assert problems(output)[0]["text"] == "This blueprint needs the robot's IP address."
    assert problems(output)[0]["level"] == "error"
    output = "$ dimos run x\nKeyError: \"None of ('go2_lidar', 'lidar') in dataset '/r/spot.db'; available: []\"\n"
    assert "doesn't have the streams" in problems(output)[0]["text"]


def test_unknown_errors_are_shown_as_they_are() -> None:
    found = problems(
        "$ dimos run x\n12:00:00.000 [inf][a] fine\nRuntimeError: the flux capacitor is cold\n"
    )
    assert found == [
        {
            "level": "error",
            "text": "RuntimeError: the flux capacitor is cold",
            "fix": None,
            "line": "RuntimeError: the flux capacitor is cold",
        }
    ]
    assert problems(START) == []
    many = "$ dimos run x\n" + "".join(f"RuntimeError: {i}\n" for i in range(5))
    assert [p["text"] for p in problems(many)] == [f"RuntimeError: {i}" for i in (2, 3, 4)]
    assert problems("$ dimos run x\nError: " + "x" * 400 + "\n")[0]["text"].endswith("…")
