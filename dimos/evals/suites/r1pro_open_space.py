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

from pathlib import Path

from dimos.evals.suites.lib.r1pro_tasks import curriculum, eval_case, load_layouts
from dimos.evals.types import Suite

LAYOUTS = Path(__file__).with_name("r1pro_open_space.json")

SUITE: Suite = [
    eval_case(layout, task) for layout in load_layouts(LAYOUTS) for task in curriculum(layout)
]
