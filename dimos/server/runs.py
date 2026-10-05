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
from dataclasses import asdict
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import threading
import time
from typing import Any

from dimos.core.run_registry import is_pid_alive
from dimos.server import config, diagnose, logs


class RunError(Exception):
    """A launch or stop that can't happen (a 500 with a readable message)."""


class StillRunningError(RunError):
    """A launch while the last one is still starting or running (a 400)."""


def launch_file() -> Path:
    return config.state_file("launch.json")


def launch_log() -> Path:
    """The launch's console output (shown as is; never read for meaning)."""
    return config.logs_dir() / "launch.log"


def launch_records_dir() -> Path:
    """Where the launch's structured log starts (DIMOS_RUN_LOG_DIR): dimos logs there until it knows its run id, then
    says where it goes on (a `stage: run_log` record) and moves to the run's own log dir."""
    return config.server_dir() / "launch"


# the records a launch's diagnosis needs (a stage, a problem, an error) carry one of these; the rest of a log, which
# can be megabytes, is skipped without parsing it
_WANTED = (b'"stage"', b'"problem"', b'"level": "error"', b'"level": "critical"')
_records_cache: dict[Path, tuple[tuple[int, int], list[dict[str, Any]]]] = {}


def _records(file: Path) -> list[dict[str, Any]]:
    """`file`'s stage, problem and error records (its last 4 MB), cached while the file is unchanged."""
    try:
        stat = file.stat()
    except OSError:
        return []
    key = (stat.st_size, stat.st_mtime_ns)
    cached = _records_cache.get(file)
    if cached and cached[0] == key:
        return cached[1]
    with file.open("rb") as handle:
        handle.seek(max(0, stat.st_size - logs.MAX_READ))
        lines = [line for line in handle if any(wanted in line for wanted in _WANTED)]
    parsed = [logs.parse_line(line.decode("utf-8", "replace")) for line in lines]
    records = [record for record in parsed if record is not None and record["level"] != "raw"]
    _records_cache[file] = (key, records)
    return records


def launch_records() -> list[dict[str, Any]]:
    """The current launch's records: from before it had a run id, then from its run's main.jsonl."""
    records = _records(launch_records_dir() / "main.jsonl")
    moved = next(
        (r["extra"].get("log_dir") for r in records if r["extra"].get("stage") == "run_log"), None
    )
    if isinstance(moved, str):
        records = records + _records(Path(moved) / "main.jsonl")
    return records


def registry_runs() -> list[dict[str, Any]]:
    """Live runs from dimos's run registry, newest first (runs started from a terminal too), with RegistryRun's fields
    of each RunEntry."""
    from dimos.core.run_registry import list_runs
    from dimos.server.models import RegistryRun

    runs = [
        {key: value for key, value in asdict(entry).items() if key in RegistryRun.model_fields}
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
    elif is_pid_alive(pid):
        phase = "starting"
    elif record.get("ever_ran"):
        phase = "stopped"
    else:
        phase = "failed"
    records = launch_records()
    problems = diagnose.problems(records)
    error = diagnose.error_text(problems, output) if phase == "failed" else None
    overrides = record.get("overrides")
    return {
        "blueprint": blueprint,
        "phase": phase,
        "startedAt": started_at,
        "pid": pid,
        "output": output,
        "runId": entry["run_id"] if entry else None,
        "logDir": entry["log_dir"] if entry else None,
        "error": error,
        "overrides": overrides if isinstance(overrides, dict) else {},
        "steps": diagnose.steps(records, phase),
        "problems": problems,
    }


def last_launch_args() -> tuple[str, dict[str, Any]] | None:
    """The blueprint and global config of the last launch (to launch it again), even after it stopped."""
    try:
        record = json.loads(launch_file().read_text())
        overrides = record.get("overrides")
        return str(record["blueprint"]), overrides if isinstance(overrides, dict) else {}
    except (OSError, ValueError, KeyError, TypeError, AttributeError):
        return None


def now_iso() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def start(dimos_dir: Path, blueprint: str, overrides: dict[str, Any]) -> dict[str, Any]:
    """`dimos [--key value ...] run <blueprint>` with GlobalConfig `overrides`, in the foreground of its own session
    (not `--daemon`: on macOS the daemon's post-fork build segfaults inside CoreFoundation). The launch keeps its
    overrides, so it can be launched again the same way."""
    previous = current_launch()
    if previous and previous["phase"] in ("starting", "running"):
        raise StillRunningError(
            f"{previous['blueprint']} is still {previous['phase']}; stop it first"
        )
    program = config.dimos_bin(dimos_dir)
    if not program.exists():
        raise RunError(f"no dimos at {dimos_dir} (no {program})")
    args = [*config.global_config_flags(overrides), "run", blueprint]
    launch_log().parent.mkdir(parents=True, exist_ok=True)
    launch_log().write_text(f"$ dimos {' '.join(args)}\n")
    shutil.rmtree(launch_records_dir(), ignore_errors=True)
    venv = config.venv_dir(dimos_dir)
    env = {
        **os.environ,
        # the checkout's venv first, as an activated venv would: dimos spawns tools from it by name (mjpython)
        "PATH": f"{venv / 'bin'}{os.pathsep}{os.environ.get('PATH', '')}",
        "VIRTUAL_ENV": str(venv),
        "PYTHONUNBUFFERED": "1",
        "NO_COLOR": "1",
        # its structured log starts here, so even what it logs before it has a run id can be read
        "DIMOS_RUN_LOG_DIR": str(launch_records_dir()),
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
    record = {
        "blueprint": blueprint,
        "started_at": now_iso(),
        "pid": child.pid,
        "ever_ran": False,
        "overrides": overrides,
    }
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
            if not is_pid_alive(pid):
                return f"stopped {name} (pid {pid})"
            await asyncio.sleep(0.25)
    raise RunError(f"{name} (pid {pid}) won't stop")
