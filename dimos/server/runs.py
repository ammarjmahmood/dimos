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

"""Blueprint runs: `dimos run` started in its own session (so it outlives the server), plus the run registry.

The server remembers only the last launch it started (launch.json); whether it is starting, running or gone is
re-derived from the registry and the pid on every call, so a restarted server picks up where the last one was.
"""

from __future__ import annotations

import asyncio
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
from typing import Any

from dimos.server import config


class RunError(Exception):
    """A launch or stop that can't happen (a 400 / 500 with a readable message)."""


def launch_file() -> Path:
    return config.state_file("launch.json")


def launch_log() -> Path:
    return config.logs_dir() / "launch.log"


def is_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def registry_runs() -> list[dict[str, Any]]:
    """Live runs from dimos's run registry, newest first (runs started from a terminal too)."""
    from dimos.core.run_registry import list_runs

    runs = [
        {
            "run_id": entry.run_id,
            "pid": entry.pid,
            "blueprint": entry.blueprint,
            "started_at": entry.started_at,
            "log_dir": entry.log_dir,
        }
        for entry in list_runs(alive_only=True)
    ]
    return sorted(runs, key=lambda run: str(run["run_id"]), reverse=True)


def _tail(path: Path, size: int) -> str:
    try:
        with path.open("rb") as handle:
            end = handle.seek(0, 2)
            handle.seek(max(0, end - size))
            return handle.read().decode("utf-8", "replace")
    except OSError:
        return ""


def current_launch() -> dict[str, Any] | None:
    """The last launch this server started, with its phase: starting, running, stopped or failed."""
    try:
        record = json.loads(launch_file().read_text())
        pid, blueprint, started_at = (
            int(record["pid"]),
            str(record["blueprint"]),
            str(record["started_at"]),
        )
    except (OSError, ValueError, KeyError, TypeError):
        return None
    entry = next((run for run in registry_runs() if run["pid"] == pid), None)
    if entry and not record.get("ever_ran"):
        # dimos registers a run once every module is built: from then on it "ran"
        record["ever_ran"] = True
        config.write_atomic(launch_file(), json.dumps(record))
    output = _tail(launch_log(), 200_000)
    if entry:
        phase = "running"
    elif is_alive(pid):
        phase = "starting"
    elif record.get("ever_ran"):
        phase = "stopped"
    else:
        phase = "failed"
    error = None
    if phase == "failed":
        lines = output.splitlines()
        error = next((line for line in lines if line.startswith("Error: ")), None)
        error = error or (output.strip().splitlines() or ["dimos exited during startup"])[-1]
    return {
        "blueprint": blueprint,
        "phase": phase,
        "startedAt": started_at,
        "pid": pid,
        "output": output,
        "runId": entry["run_id"] if entry else None,
        "logDir": entry["log_dir"] if entry else None,
        "error": error,
    }


def now_iso() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def start(dimos_dir: Path, blueprint: str, global_flags: list[str]) -> dict[str, Any]:
    """`dimos [flags] run <blueprint>` in the foreground of its own session (not `--daemon`: on macOS the daemon's
    post-fork build segfaults inside CoreFoundation)."""
    previous = current_launch()
    if previous and previous["phase"] in ("starting", "running"):
        raise RunError(f"{previous['blueprint']} is still {previous['phase']}; stop it first")
    program = config.dimos_bin(dimos_dir)
    if not program.exists():
        raise RunError(f"no dimos at {dimos_dir} (no {program})")
    args = [*global_flags, "run", blueprint]
    launch_log().parent.mkdir(parents=True, exist_ok=True)
    launch_log().write_text(f"$ dimos {' '.join(args)}\n")
    venv = dimos_dir / ".venv"
    env = {
        **os.environ,
        # the checkout's venv first, as an activated venv would: dimos spawns tools from it by name (mjpython)
        "PATH": f"{venv / 'bin'}{os.pathsep}{os.environ.get('PATH', '')}",
        "VIRTUAL_ENV": str(venv),
        "PYTHONUNBUFFERED": "1",
        "NO_COLOR": "1",
    }
    with launch_log().open("a") as log:
        child = subprocess.Popen(
            [str(program), *args],
            cwd=dimos_dir,
            env=env,
            stdin=subprocess.DEVNULL,
            stdout=log,
            stderr=log,
            # its own session: stopping the server leaves the robot running
            start_new_session=True,
        )
    # reap it, so a finished run doesn't linger as a zombie that still looks alive
    threading.Thread(target=child.wait, daemon=True).start()
    record = {"blueprint": blueprint, "started_at": now_iso(), "pid": child.pid, "ever_ran": False}
    config.write_atomic(launch_file(), json.dumps(record))
    launch = current_launch()
    assert launch is not None
    return launch


async def stop(run_id: str | None) -> str:
    """Stops this server's launch, or any live registry run by id: its process group gets Ctrl-C, then SIGTERM, then
    SIGKILL, as a terminal would (not `dimos stop`, which picks its own target)."""
    if run_id:
        run = next((r for r in registry_runs() if r["run_id"] == run_id), None)
        if run is None:
            raise RunError(f"no live run {run_id}")
        pid, name = int(run["pid"]), str(run["blueprint"])
    else:
        launch = current_launch()
        if not launch or launch["phase"] not in ("starting", "running"):
            raise RunError("the dimos server hasn't launched anything that's still running")
        pid, name = launch["pid"], launch["blueprint"]
    for signum, wait in ((signal.SIGINT, 20), (signal.SIGTERM, 10), (signal.SIGKILL, 5)):
        try:
            # a launch is its own session, so its pgid is its pid; a run from a terminal may not be
            os.killpg(pid, signum)
        except (ProcessLookupError, PermissionError):
            try:
                os.kill(pid, signum)
            except ProcessLookupError:
                pass
        for _ in range(wait * 4):
            if not is_alive(pid):
                return f"stopped {name} (pid {pid})"
            await asyncio.sleep(0.25)
    raise RunError(f"{name} (pid {pid}) won't stop")
