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


"""SceneReplica's two scores and failure split, read from one or more eval run folders.

    python -m dimos.evals.suites.lib.scenereplica_summary ~/.local/state/dimos/evals/run-*

Grasp success counts a trial whose object was lifted; pick-and-place success counts a
trial whose object was also set down in the drop-off. Both are printed over the trials
actually found in the given runs, so a partial run reads correctly.
"""

from __future__ import annotations

from collections import Counter
from collections.abc import Iterable, Sequence
from dataclasses import dataclass
import json
from pathlib import Path
import re
import sys
from typing import Any

SUITE_SIZE = 100
_CASE_ID = re.compile(r"^scenereplica-(scene-\d+)-(.+)$")


@dataclass(frozen=True)
class Trial:
    scene: str
    object: str
    grasped: bool
    placed: bool
    cause: str  # perception, planning, execution, "error", or "" for a success


def load_trials(run_dirs: Iterable[Path]) -> list[Trial]:
    """Every SceneReplica results row in the run folders, one trial each; other suites'
    rows are skipped. The same case in two runs gives two trials."""
    trials = []
    for run_dir in run_dirs:
        results = Path(run_dir) / "results.jsonl"
        if not results.is_file():
            continue
        for line in results.read_text().splitlines():
            if line.strip():
                trial = _trial(json.loads(line))
                if trial is not None:
                    trials.append(trial)
    return trials


def _trial(row: dict[str, Any]) -> Trial | None:
    match = _CASE_ID.match(str(row.get("case_id", "")))
    if match is None:
        return None
    details = row.get("details") or {}
    grasped = not row.get("error") and float(details.get("lifted", 0.0)) >= 1.0
    placed = grasped and float(details.get("placed", 0.0)) >= 1.0
    if placed:
        cause = ""
    elif row.get("error") or not details:
        cause = "error"
    else:
        cause = str(details.get("cause") or "execution")
    return Trial(match.group(1), match.group(2), grasped, placed, cause)


def _table(title: str, trials: Sequence[Trial], key: str) -> list[str]:
    groups = sorted({getattr(t, key) for t in trials})
    width = max([len(title), *(len(g) for g in groups)])
    lines = [f"{title:<{width}}  n  grasp  place"]
    for group in groups:
        rows = [t for t in trials if getattr(t, key) == group]
        lines.append(
            f"{group:<{width}}  {len(rows)}  {sum(t.grasped for t in rows):<5}  "
            f"{sum(t.placed for t in rows)}"
        )
    return lines


def report(trials: Sequence[Trial]) -> str:
    """The summary text: both success counts, the cause split, then per-object and
    per-scene tables."""
    n = len(trials)
    if n == 0:
        return "No SceneReplica trials found."
    grasped = sum(t.grasped for t in trials)
    placed = sum(t.placed for t in trials)
    causes = Counter(t.cause for t in trials if t.cause)
    lines = [
        f"SceneReplica: {n} of {SUITE_SIZE} trials run, {len({t.scene for t in trials})} scene(s)",
        f"grasp success          {grasped:3d} / {n}  ({grasped / n:.0%})",
        f"pick-and-place success {placed:3d} / {n}  ({placed / n:.0%})",
        "failures: "
        + ", ".join(f"{c} {causes.get(c, 0)}" for c in ("perception", "planning", "execution"))
        + (f", error {causes['error']}" if causes.get("error") else ""),
        "",
        *_table("object", trials, "object"),
        "",
        *_table("scene", trials, "scene"),
    ]
    return "\n".join(lines)


def main(argv: Sequence[str] | None = None) -> None:
    args = list(sys.argv[1:] if argv is None else argv)
    if not args:
        raise SystemExit(f"usage: {Path(sys.argv[0]).name} RUN_DIR [RUN_DIR ...]")
    print(report(load_trials(Path(a) for a in args)))


if __name__ == "__main__":
    main()
