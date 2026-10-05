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

"""A launch's output (`dimos run`'s stdout and stderr) read for a person: how far startup got, and what went wrong in
plain words with the fix. Only the patterns below; anything else falls back to the error line itself. The same rules as
Desktop's Rust (src/dimos/diagnose.rs), so a launch reads the same whichever server ran it."""

from __future__ import annotations

from typing import Any

# (the step's label, the output line that marks it), in order; the lines after the `$ dimos ...` command line count
MARKS = [
    ("Starting dimOS", "Starting DimOS"),
    ("Building the blueprint", "Building the blueprint"),
    ("Starting modules", "Starting the modules"),
]

# (a piece of the line, what it means, the fix)
KNOWN = [
    (
        "No robot IPs specified",
        "This blueprint needs the robot's IP address.",
        "Pick Robot as the source and fill in Robot IP.",
    ),
    (
        "IP address must be provided",
        "This blueprint needs the robot's IP address.",
        "Pick Robot as the source and fill in Robot IP.",
    ),
    (
        "in dataset",
        "That recording doesn't have the streams this blueprint replays.",
        "Pick a recording the Launcher marks as usable.",
    ),
    (
        "Address already in use",
        "A network port it needs is taken, most likely by another run.",
        "Stop the other run (Running, at the top) and launch again.",
    ),
    (
        "No route to host",
        "This computer can't reach the robot.",
        "Check the robot is on and on the same network, and that the IP is right.",
    ),
    (
        "Connection refused",
        "The robot (or a service it needs) refused the connection.",
        "Check the robot is on and finished booting, and that the IP is right.",
    ),
    (
        "timed out",
        "Something it connects to didn't answer in time.",
        "Check the robot is on and on the same network, and that the IP is right.",
    ),
    (
        "git lfs",
        "It needs data that isn't downloaded yet (git LFS).",
        "Run `git lfs pull` in the dimos checkout, or pick another recording.",
    ),
    (
        "unable to open database file",
        "A recording file it needs can't be opened.",
        "Check the recording still exists, or pick another one.",
    ),
    (
        "outside the range this Desktop supports",
        "This dimos version is newer or older than Desktop supports.",
        "Update Desktop or dimos (Settings), or set dimos.ignore_version_range.",
    ),
    ("MemoryError", "It ran out of memory.", "Stop other runs or apps, then launch again."),
    (
        "CUDA out of memory",
        "The GPU ran out of memory.",
        "Stop other runs using the GPU, then launch again.",
    ),
]


def steps(output: str, phase: str) -> list[dict[str, Any]]:
    """Each startup step's state: done, now, todo or failed; then Running (or Stopped)."""
    lines = output.splitlines()
    started = any(line.strip() for line in lines[1:])
    # the last step the output shows it reached (0 once dimos printed anything)
    reached = 0 if started else None
    for index, (_, mark) in enumerate(MARKS):
        if mark in output:
            reached = index
    deployed = output.count("Deployed module.")
    result = []
    for index, (label, _) in enumerate(MARKS):
        if phase in ("running", "stopped"):
            state = "done"
        elif reached is None:
            state = (
                "now"
                if index == 0 and phase == "starting"
                else "failed"
                if index == 0 and phase == "failed"
                else "todo"
            )
        elif index < reached:
            state = "done"
        elif index == reached:
            state = "failed" if phase == "failed" else "now"
        else:
            state = "todo"
        detail = f"{deployed} started" if index == 2 and deployed > 0 else None
        result.append({"label": label, "state": state, "detail": detail})
    result.append(
        {
            "label": "Stopped" if phase == "stopped" else "Running",
            "state": "done" if phase in ("running", "stopped") else "todo",
            "detail": None,
        }
    )
    return result


def short(line: str) -> str:
    line = line.strip()
    return line[:300] + "…" if len(line) > 300 else line


def missing_module(line: str) -> str | None:
    """`No module named 'unitree_sdk2py'` -> `unitree_sdk2py`"""
    at = line.find("No module named ")
    if at < 0:
        return None
    rest = line[at + len("No module named ") :].lstrip("'\"")
    ends = [i for i in (rest.find("'"), rest.find('"')) if i >= 0]
    return rest[: min(ends)] if ends else rest or None


def is_error_line(line: str) -> bool:
    if "[err]" in line or "[cri]" in line or line.startswith("Error: "):
        return True
    # a Python exception line: `SomeError: text` / `pkg.SomeError: text`
    head = line.split(":")[0]
    return " " not in head and head.endswith(("Error", "Exception")) and ": " in line


def problems(output: str) -> list[dict[str, Any]]:
    """What went wrong, most specific first: known patterns with their fix, else the error lines themselves (at most
    3, the last usually the one that ended it)."""
    found: list[dict[str, Any]] = []

    def add(level: str, text: str, fix: str | None, line: str) -> None:
        if not any(problem["text"] == text for problem in found):
            found.append({"level": level, "text": text, "fix": fix, "line": short(line)})

    generic = []
    for line in output.splitlines()[1:]:
        module = missing_module(line)
        if module:
            add(
                "error",
                f"dimos is missing the Python package `{module}`, which this blueprint needs.",
                "Install it into the dimos checkout's environment (`uv sync --all-extras` there installs every "
                "extra), or pick a blueprint for hardware you have.",
                line,
            )
            continue
        known = next((entry for entry in KNOWN if entry[0] in line), None)
        if known:
            level = "error" if is_error_line(line) or "Error" in line else "warning"
            add(level, known[1], known[2], line)
            continue
        if is_error_line(line):
            generic.append(line)
    if not found:
        seen: list[str] = []
        for line in reversed(generic):
            text = short(line)
            if text not in seen:
                seen.append(text)
            if len(seen) == 3:
                break
        found = [
            {"level": "error", "text": text, "fix": None, "line": text} for text in reversed(seen)
        ]
    return found
