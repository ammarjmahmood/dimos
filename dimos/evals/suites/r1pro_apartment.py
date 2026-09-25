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

"""R1 Pro in the MuJoCo apartment: the open-space curriculum through rooms and doorways.

Objects start on the worktable, the kitchen counter and the dining table. The
tray fits only on the worktable here, so tray cases load it and leave it
there. Same agents, same grading as ``r1pro_open_space``; case IDs start with
``apt_``.

    dimos evals run dimos.evals.suites.r1pro_apartment --agent dimos.evals.agents.scripted_plan
    dimos evals run dimos.evals.suites.r1pro_apartment --agent dimos.evals.agents.mcp_client_adapter \\
        --set 'modules=["r1pro-classical-apartment-sim-agent"]'
"""

from __future__ import annotations

from pathlib import Path

from dimos.evals.suites.lib.r1pro_tasks import curriculum, eval_case, load_layouts
from dimos.evals.types import Suite

LAYOUTS = Path(__file__).with_name("r1pro_apartment.json")

SUITE: Suite = [
    eval_case(layout, task) for layout in load_layouts(LAYOUTS) for task in curriculum(layout)
]
