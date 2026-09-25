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

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from dimos.evals.compare import latest_runs, load_run, table


def _run(path: Path, module: str, rows: list[dict[str, Any]], **kwargs: Any) -> Path:
    path.mkdir()
    (path / "manifest.json").write_text(json.dumps({"agent": {"module": module, "kwargs": kwargs}}))
    (path / "results.jsonl").write_text("\n".join(json.dumps(r) for r in rows) + "\n")
    return path


def _row(case_id: str, score: float, *, reached: int | None = None) -> dict[str, Any]:
    details = {"milestones_reached": reached, "milestones_total": 2} if reached is not None else {}
    return {
        "case_id": case_id,
        "score": score,
        "passed": score >= 1.0,
        "error": "",
        "duration_s": 60.0,
        "prompt_tokens": 10,
        "completion_tokens": 2,
        "details": details,
    }


def test_runs_side_by_side_with_repeats_folded(tmp_path: Path) -> None:
    scripted = _run(
        tmp_path / "run-1",
        "dimos.evals.agents.scripted_plan",
        [_row("pick.r1", 1.0, reached=2), _row("pick.r2", 0.5, reached=1)],
    )
    agent = _run(
        tmp_path / "run-2",
        "dimos.evals.agents.mcp_client_adapter",
        [_row("pick", 0.5, reached=1), _row("move", 1.0)],
        modules=["r1pro-classical-open-space-sim-agent"],
    )
    text = table([load_run(scripted), load_run(agent)])
    lines = text.splitlines()
    assert lines[0].startswith("| case | scripted_plan | mcp_client_adapter (modules=")
    assert "| pick | 1/2 pass · 0.75 | fail 0.50 1/2 |" in lines
    assert "| move | - | PASS 1.00 |" in lines
    assert "| **passed** | 1/2 | 1/2 |" in lines


def test_latest_runs_are_the_newest_complete_ones(tmp_path: Path) -> None:
    for name in ("run-20260101-000000-a", "run-20260102-000000-b", "run-20260103-000000-c"):
        _run(tmp_path / name, "m", [_row("x", 1.0)])
    (tmp_path / "run-20260104-000000-unfinished").mkdir()
    assert [p.name for p in latest_runs(tmp_path, 2)] == [
        "run-20260102-000000-b",
        "run-20260103-000000-c",
    ]
