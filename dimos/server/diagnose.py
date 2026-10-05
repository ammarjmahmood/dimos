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

"""How far a launch got and what went wrong, as stable codes with data, read from its structured log records.

dimos marks its startup with a `stage` field (`starting`, `run_log`, `building`, `starting_modules`,
`module_deployed`, `started`), marks refusals it knows with a `problem` field, and logs exceptions with
`exception_chain` / `exception_code` / `missing_module` (logging_config.exception_fields). Nothing here reads the
wording of a message. The words (and the fixes) are Desktop's, per code.
"""

from __future__ import annotations

import re
import subprocess
import time
from typing import Any, Literal

import psutil

StepCode = Literal["starting", "building", "starting_modules", "running", "stopped"]
StepState = Literal["done", "now", "todo", "failed"]
ProblemCode = Literal[
    "bad_arguments",
    "unknown_blueprint",
    "requirement_unmet",
    "missing_python_package",
    "robot_ip_missing",
    "robot_unreachable",
    "replay_streams_missing",
    "lfs_data_missing",
    "port_in_use",
    "host_unreachable",
    "connection_refused",
    "timed_out",
    "recording_unopenable",
    "out_of_memory",
    "gpu_out_of_memory",
    "error",
]

# the startup stages a step stands for, in order (the last, running, is dimos's run registry entry)
STAGES: tuple[StepCode, ...] = ("starting", "building", "starting_modules")

# a `problem` field dimos logs -> its code
PROBLEM_FIELDS: dict[str, ProblemCode] = {
    "bad_arguments": "bad_arguments",
    "unknown_blueprint": "unknown_blueprint",
    "requirement_unmet": "requirement_unmet",
}

# an exception class in a record's exception_chain -> its code (test_diagnose checks each dimos class still exists)
EXCEPTION_CODES: dict[str, ProblemCode] = {
    "ModuleNotFoundError": "missing_python_package",
    "dimos.core.global_config.MissingRobotIpError": "robot_ip_missing",
    "dimos.robot.unitree.go2.connection.MissingReplayStreamError": "replay_streams_missing",
    "dimos.simulation.dimsim.dimsim_process.LfsStubError": "lfs_data_missing",
    "unitree_webrtc_connect.unitree_auth.LocalSignalingPortError": "robot_unreachable",
    "ConnectionRefusedError": "connection_refused",
    "TimeoutError": "timed_out",
    "MemoryError": "out_of_memory",
    "torch.OutOfMemoryError": "gpu_out_of_memory",
}

# a record's exception_code (an errno name or a SQLite error name) -> its code
ERROR_CODES: dict[str, ProblemCode] = {
    "EADDRINUSE": "port_in_use",
    "EHOSTUNREACH": "host_unreachable",
    "ENETUNREACH": "host_unreachable",
    "ECONNREFUSED": "connection_refused",
    "ETIMEDOUT": "timed_out",
    "SQLITE_CANTOPEN": "recording_unopenable",
}

# record fields that are about the log call, not the problem
NOT_DATA = {
    "func_name",
    "lineno",
    "exception",
    "traceback_lines",
    "problem",
    "stage",
    "exception_chain",
    "exception_type",
    "exception_message",
}

ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")


def stage(record: dict[str, Any]) -> Any:
    return record["extra"].get("stage")


def steps(records: list[dict[str, Any]], phase: str) -> list[dict[str, Any]]:
    """starting, building, starting_modules (with how many modules started of how many), then running or stopped;
    each done, now, todo or failed."""
    reached: int | None = None
    deployed, total = 0, None
    for record in records:
        found = stage(record)
        if found in STAGES:
            reached = max(reached or 0, STAGES.index(found))
        if found == "starting_modules" and isinstance(record["extra"].get("modules"), int):
            total = record["extra"]["modules"]
        if found == "module_deployed":
            deployed += 1
    result = []
    for index, code in enumerate(STAGES):
        if phase in ("running", "stopping", "stopped"):
            state: StepState = "done"
        elif reached is None:
            state = "todo" if index else "failed" if phase == "failed" else "now"
        elif index < reached:
            state = "done"
        elif index == reached:
            state = "failed" if phase == "failed" else "now"
        else:
            state = "todo"
        data = {"deployed": deployed, "total": total} if code == "starting_modules" else {}
        result.append({"code": code, "state": state, "data": data})
    last: StepCode = "stopped" if phase == "stopped" else "running"
    result.append(
        {
            "code": last,
            "state": "done" if phase in ("running", "stopping", "stopped") else "todo",
            "data": {},
        }
    )
    return result


def classify(extra: dict[str, Any]) -> ProblemCode:
    if extra.get("problem") in PROBLEM_FIELDS:
        return PROBLEM_FIELDS[extra["problem"]]
    chain = extra.get("exception_chain")
    for name in chain if isinstance(chain, list) else []:
        if name in EXCEPTION_CODES:
            return EXCEPTION_CODES[name]
    return ERROR_CODES.get(str(extra.get("exception_code")), "error")


# the address an OSError names, `('127.0.0.1', 3030)` (asyncio's and socket's bind errors print the address tuple)
ADDRESS = re.compile(r"\(\s*'[^']*'\s*,\s*(\d{1,5})\s*\)")
HOLDER_TTL_S = 10.0
_holders: dict[int, tuple[float, dict[str, Any]]] = {}


def port_of(*texts: Any) -> int | None:
    """The port in the address an OSError's text names, when it names one."""
    for text in texts:
        found = ADDRESS.search(str(text or ""))
        if found:
            return int(found.group(1))
    return None


def listening_pid(port: int) -> int | None:
    """lsof (macOS and most Linux; no root needed for the user's own processes), else psutil (Linux, no lsof)."""
    try:
        listed = subprocess.run(
            ["lsof", "-nP", f"-iTCP:{port}", "-sTCP:LISTEN", "-Fp"],
            capture_output=True,
            text=True,
            timeout=5,
        ).stdout
        return next((int(line[1:]) for line in listed.splitlines() if line[:1] == "p"), None)
    except FileNotFoundError:
        pass
    except (OSError, subprocess.SubprocessError, ValueError):
        return None
    try:
        return next(
            (
                c.pid
                for c in psutil.net_connections(kind="tcp")
                if c.status == psutil.CONN_LISTEN and c.laddr and c.laddr.port == port and c.pid
            ),
            None,
        )
    except psutil.Error:
        return None


def port_holder(port: int) -> dict[str, Any]:
    """Which process listens on this TCP port: `{holder_pid, holder_command}`, or {} when that can't be told (lsof,
    which needs no root for the user's own processes; kept 10 s, the launch is re-read every second)."""
    now = time.monotonic()
    cached = _holders.get(port)
    if cached and now - cached[0] < HOLDER_TTL_S:
        return cached[1]
    holder: dict[str, Any] = {}
    pid = listening_pid(port)
    if pid is not None:
        holder["holder_pid"] = pid
        try:
            holder["holder_command"] = " ".join(psutil.Process(pid).cmdline())[:300] or None
        except psutil.Error:
            holder["holder_command"] = None
    _holders[port] = (now, holder)
    return holder


def problem(record: dict[str, Any]) -> dict[str, Any]:
    extra = record["extra"]
    message = extra.get("exception_message") or extra.get("error") or record["event"]
    code = classify(extra)
    data = {key: value for key, value in extra.items() if key not in NOT_DATA}
    if code == "port_in_use":
        port = port_of(extra.get("exception_message"), extra.get("error"), record["event"])
        if port is not None:
            data["port"] = port
            data.update(port_holder(port))
    return {
        "code": code,
        "level": "error",
        "message": str(message),
        "data": data,
        "timestamp": record["timestamp"],
        "logger": record["logger"],
    }


def problems(records: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Every error record as a problem, the ones with a known code first; when none has one, the last three
    distinct errors (the last is usually the one that ended it)."""
    found = [
        problem(record)
        for record in records
        if record["level"] in ("error", "critical") or "problem" in record["extra"]
    ]
    distinct: list[dict[str, Any]] = []
    for item in found:
        if not any((p["code"], p["message"]) == (item["code"], item["message"]) for p in distinct):
            distinct.append(item)
    known = [p for p in distinct if p["code"] != "error"]
    return known or distinct[-3:]


def error_text(problems_found: list[dict[str, Any]], output: str) -> str:
    """One line saying why a launch failed: its first problem, else its output's last line."""
    if problems_found:
        return str(problems_found[0]["message"])
    lines = [ANSI.sub("", line).strip() for line in output.splitlines()[1:]]
    return next((line for line in reversed(lines) if line), "dimos exited during startup")
