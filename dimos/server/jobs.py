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

"""Background jobs (an extras install), shaped like Desktop's own (its docs/events.md, "Jobs").

A job runs one command (never through a shell) and publishes each output line on zenoh at `<ns>/dimos/jobs/<job>` as
`{type: "line", n, line}`, then `{type: "done", ok, error, failure, lines}`; a `job` event says one started. The
snapshot is `GET /dimos/jobs/<job>/log?after=<n>`. Success is the exit code alone. On a failure, `error` says the exit
code and `failure` is the output's last lines (uv prints its error last, and has no structured output for an install,
so nothing is picked out by its wording); the full log stays in `lines`. Finished jobs are kept for 30 minutes.
"""

from __future__ import annotations

import asyncio
from collections.abc import Awaitable, Callable
from dataclasses import dataclass, field
import itertools
import os
from pathlib import Path
import re
import signal
import time
from typing import Any

from dimos.server.discovery import now_iso

KEEP_S = 30 * 60
ANSI = re.compile(r"\x1b\[[0-9;?]*[A-Za-z]")
FAILURE_TAIL = 15


@dataclass
class Job:
    id: str
    title: str
    kind: str
    command: list[str]
    lines: list[str] = field(default_factory=list)
    done: bool = False
    ok: bool | None = None
    error: str | None = None
    failure: list[str] = field(default_factory=list)
    started_at: str = field(default_factory=now_iso)
    finished_at: str | None = None
    ended: float | None = None
    process: asyncio.subprocess.Process | None = None
    cancelled: bool = False

    def summary(self) -> dict[str, Any]:
        return {
            "job": self.id,
            "title": self.title,
            "kind": self.kind,
            "done": self.done,
            "ok": self.ok,
            "started_at": self.started_at,
            "finished_at": self.finished_at,
        }

    def log(self, after: int = 0) -> dict[str, Any]:
        after = max(after, 0)
        return {
            **self.summary(),
            "command": self.command,
            "lines": self.lines[after:],
            "next": len(self.lines),
            "error": self.error,
            "failure": self.failure,
        }


def failure_lines(lines: list[str]) -> list[str]:
    """The output's last non-empty lines: where a failing command says why."""
    return [line for line in lines if line.strip()][-FAILURE_TAIL:]


class Jobs:
    def __init__(
        self,
        send: Callable[[dict[str, Any]], None],
        publish: Callable[[str, dict[str, Any]], None],
    ) -> None:
        self.send = send
        self.publish = publish
        self.jobs: dict[str, Job] = {}
        self.ids = itertools.count(1)

    def prune(self) -> None:
        cutoff = time.monotonic() - KEEP_S
        for key, job in list(self.jobs.items()):
            if job.ended is not None and job.ended < cutoff:
                del self.jobs[key]

    def get(self, job_id: str) -> Job:
        self.prune()
        if job_id not in self.jobs:
            raise KeyError(f"no job {job_id}")
        return self.jobs[job_id]

    def listing(self) -> list[dict[str, Any]]:
        self.prune()
        return [job.summary() for job in self.jobs.values()]

    def running(self, kind: str) -> Job | None:
        return next((j for j in self.jobs.values() if j.kind == kind and not j.done), None)

    def start(
        self,
        title: str,
        kind: str,
        command: list[str],
        cwd: Path,
        env: dict[str, str] | None = None,
        then: Callable[[Job], Awaitable[None] | None] | None = None,
    ) -> Job:
        job = Job(
            id=f"{kind}-{next(self.ids)}-{int(time.time())}",
            title=title,
            kind=kind,
            command=command,
        )
        self.jobs[job.id] = job
        self.send({"type": "job", "job": job.id, "title": title, "kind": kind})
        asyncio.get_running_loop().create_task(self.run(job, cwd, env, then))
        return job

    def line(self, job: Job, text: str) -> None:
        text = ANSI.sub("", text).rstrip("\r\n")
        for part in text.split("\r"):
            if part.strip() or not job.lines or job.lines[-1].strip():
                self.publish(f"jobs/{job.id}", {"type": "line", "n": len(job.lines), "line": part})
                job.lines.append(part)

    async def run(
        self,
        job: Job,
        cwd: Path,
        env: dict[str, str] | None,
        then: Callable[[Job], Awaitable[None] | None] | None,
    ) -> None:
        self.line(job, "$ " + " ".join(job.command))
        try:
            job.process = await asyncio.create_subprocess_exec(
                *job.command,
                cwd=cwd,
                env={**os.environ, "NO_COLOR": "1", **(env or {})},
                stdin=asyncio.subprocess.DEVNULL,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.STDOUT,
                start_new_session=True,
            )
            assert job.process.stdout
            async for raw in job.process.stdout:
                self.line(job, raw.decode("utf-8", "replace"))
            code = await job.process.wait()
            job.ok = code == 0 and not job.cancelled
            if job.cancelled:
                job.error = "cancelled"
            elif code != 0:
                job.failure = failure_lines(job.lines)
                job.error = f"{job.title} failed (exit {code})"
        except Exception as error:
            job.ok = False
            job.error = f"{job.title} couldn't start: {type(error).__name__}: {error}"
            job.failure = [job.error]
            self.line(job, job.error)
        job.done = True
        job.finished_at = now_iso()
        job.ended = time.monotonic()
        self.publish(
            f"jobs/{job.id}",
            {
                "type": "done",
                "ok": job.ok,
                "error": job.error,
                "failure": job.failure,
                "lines": len(job.lines),
            },
        )
        if then is not None:
            result = then(job)
            if asyncio.iscoroutine(result):
                await result

    def cancel(self, job_id: str) -> Job:
        job = self.get(job_id)
        if not job.done and job.process is not None and job.process.returncode is None:
            job.cancelled = True
            try:
                os.killpg(job.process.pid, signal.SIGTERM)
            except (ProcessLookupError, PermissionError):
                pass
        return job
