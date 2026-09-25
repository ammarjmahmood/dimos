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

"""Put eval runs side by side: one column per run, one row per case.

A cell reads ``PASS 1.00 10/10``: pass or fail, the score, and milestones
reached out of total when the grader reported them.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any

# Agent options worth showing in a column title; the rest stay in manifest.json.
LABEL_KWARGS = ("model", "provider", "modules", "thinking")


@dataclass(frozen=True, kw_only=True)
class Run:
    path: Path
    label: str  # the agent module, plus the options that tell two runs of it apart
    results: dict[str, dict[str, Any]]  # case_id -> results.jsonl row


def load_run(path: Path) -> Run:
    manifest = json.loads((path / "manifest.json").read_text())
    agent = manifest.get("agent") or {}
    module = str(agent.get("module") or "unknown").rsplit(".", 1)[-1]
    kwargs = agent.get("kwargs") or {}
    shown = [f"{k}={kwargs[k]}" for k in LABEL_KWARGS if k in kwargs]
    rows = [
        json.loads(line)
        for line in (path / "results.jsonl").read_text().splitlines()
        if line.strip()
    ]
    return Run(
        path=path,
        label=module + (f" ({', '.join(shown)})" if shown else ""),
        results={row["case_id"]: row for row in rows},
    )


def latest_runs(out_dir: Path, count: int) -> list[Path]:
    """The ``count`` newest complete runs, oldest first."""
    runs = sorted(
        (p for p in out_dir.glob("run-*") if (p / "results.jsonl").exists()),
        key=lambda p: p.name,
    )
    return runs[-count:]


def cell(result: dict[str, Any] | None) -> str:
    if result is None:
        return "-"
    if result.get("error"):
        return "ERROR"
    details = result.get("details") or {}
    reached = (
        f" {details['milestones_reached']}/{details['milestones_total']}"
        if "milestones_total" in details
        else ""
    )
    return f"{'PASS' if result['passed'] else 'fail'} {result['score']:.2f}{reached}"


def table(runs: Sequence[Run]) -> str:
    """A markdown table of every case any run has, then per-run totals."""
    cases = list(dict.fromkeys(case for run in runs for case in run.results))
    lines = [
        "| case | " + " | ".join(run.label for run in runs) + " |",
        "|---|" + "---|" * len(runs),
    ]
    lines += [
        f"| {case} | " + " | ".join(cell(run.results.get(case)) for run in runs) + " |"
        for case in cases
    ]

    def total(run: Run, key: str) -> float:
        return sum(float(r.get(key) or 0.0) for r in run.results.values())

    def summary(name: str, value: Any) -> str:
        return f"| **{name}** | " + " | ".join(value(run) for run in runs) + " |"

    lines += [
        summary(
            "passed",
            lambda run: f"{sum(r['passed'] for r in run.results.values())}/{len(run.results)}",
        ),
        summary(
            "mean score",
            lambda run: f"{total(run, 'score') / max(len(run.results), 1):.2f}",
        ),
        summary("errors", lambda run: str(sum(bool(r.get("error")) for r in run.results.values()))),
        summary("minutes", lambda run: f"{total(run, 'duration_s') / 60:.1f}"),
        summary(
            "tokens in/out",
            lambda run: f"{int(total(run, 'prompt_tokens'))}/{int(total(run, 'completion_tokens'))}",
        ),
        summary("run", lambda run: run.path.name),
    ]
    return "\n".join(lines)
