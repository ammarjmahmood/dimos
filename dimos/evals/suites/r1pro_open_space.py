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

"""R1 Pro in the open-space MuJoCo arena: classical pick, place, hand swaps and the tray.

Five platforms of different heights, one object on each, a tray on the
worktable. Per seed the cases go from one skill to the whole job (see
``lib/r1pro_tasks.py``). All motion is planned by the classical stack
(GraspGenX grasps, IK, collision-checked trajectories); agents only choose
skills. Graders read the simulator's object state, not the agent's reply.

    # the skills alone, no model: a scripted plan per case
    dimos evals run dimos.evals.suites.r1pro_open_space --agent dimos.evals.agents.scripted_plan
    # dimOS's own agent
    dimos evals run dimos.evals.suites.r1pro_open_space --agent dimos.evals.agents.mcp_client_adapter \\
        --set 'modules=["r1pro-classical-open-space-sim-agent"]'
    # TypeSafe's Jev choosing the next skill (needs TYPESAFE_API_KEY)
    dimos evals run dimos.evals.suites.r1pro_open_space --agent dimos.evals.agents.jev_planner
"""

from __future__ import annotations

import json
from pathlib import Path

from dimos.evals.environments.r1pro_scene import R1ProScene
from dimos.evals.suites.lib.r1pro_tasks import Layout, Task, curriculum, grader
from dimos.evals.types import EvalCase, Suite

LAYOUTS = Path(__file__).with_name("r1pro_open_space.json")


def case(layout: Layout, task: Task, *, headless: bool = True) -> EvalCase:
    return EvalCase(
        id=task.id,
        inputs=task.instruction,
        environment=R1ProScene(
            seed=layout.seed,
            expected_objects=layout.expected_objects(),
            reference_plan=list(task.plan),
            headless=headless,
        ),
        grade=grader(task),
        timeout_s=task.timeout_s,
        threshold=1.0,
        tags=frozenset({"r1pro", "mujoco", "manipulation", f"seed{layout.seed}", *task.tags}),
    )


def layouts() -> list[Layout]:
    return [Layout.from_json(d) for d in json.loads(LAYOUTS.read_text())["layouts"]]


SUITE: Suite = [case(layout, task) for layout in layouts() for task in curriculum(layout)]
