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

"""Run each case's reference skill plan with no model in the loop.

The plan fixes which skill runs next; every motion is still planned live by
the classical stack. Scores therefore measure the robot stack alone, and set
the ceiling a model-driven agent can reach on the same cases.
"""

from __future__ import annotations

import json
from pathlib import Path
import time
from typing import TYPE_CHECKING, Any

from dimos.agents.mcp.mcp_adapter import McpAdapter
from dimos.evals.agents.base import Agent, AgentConfig
from dimos.evals.agents.lib.trajectory_builder import TrajectoryBuilder
from dimos.evals.environments.lib.r1pro_actions import run_action
from dimos.evals.types import RunningEnvironment, ToolCall, Trajectory

if TYPE_CHECKING:
    from dimos.evals.environments.base import Environment


class ScriptedPlanConfig(AgentConfig):
    # Seconds one skill may move before it is stopped.
    action_timeout_s: float = 900.0
    # Run the remaining steps after one fails. Off by default: later steps
    # assume the earlier ones happened.
    continue_on_failure: bool = False


class ScriptedPlan(Agent):
    """Call the skills listed in the environment's ``reference_plan``, in order."""

    config: ScriptedPlanConfig

    def available_tools(self, environment_tools: tuple[str, ...]) -> tuple[str, ...]:
        return environment_tools

    def preflight(self, environment: Environment) -> None:
        if not getattr(getattr(environment, "config", None), "reference_plan", None):
            raise RuntimeError(
                f"ScriptedPlan needs an environment with a reference_plan; "
                f"{type(environment).__name__} has none"
            )

    def run(
        self, inputs: str, env: RunningEnvironment, run_dir: Path, *, timeout_s: float
    ) -> Trajectory:
        plan: list[dict[str, Any]] = json.loads(Path(env.artifacts["reference_plan"]).read_text())
        mcp = McpAdapter(env.mcp_url)
        raw = run_dir / "raw"
        raw.mkdir(parents=True, exist_ok=True)
        trajectory = TrajectoryBuilder(inputs, name=type(self).__name__, model="scripted-plan")
        deadline = time.monotonic() + timeout_s
        failures: list[str] = []
        for i, step in enumerate(plan, start=1):
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return trajectory.build("timeout")
            request = raw / f"{i:03d}-request.json"
            response = raw / f"{i:03d}-response.json"
            request.write_text(json.dumps(step, indent=2))
            call = ToolCall(
                tool_call_id=f"call_{i}", function_name=step["tool"], arguments=step["arguments"]
            )
            started = time.time()
            try:
                outcome = run_action(
                    mcp,
                    step["tool"],
                    step["arguments"],
                    timeout_s=min(self.config.action_timeout_s, remaining),
                )
            except Exception as e:
                outcome = {"state": "error", "success": False, "error": repr(e)}
            response.write_text(json.dumps(outcome, indent=2))
            trajectory.step(
                message=f"step {i}/{len(plan)}: {step['tool']} {json.dumps(step['arguments'])}",
                request=request,
                response=response,
                tool_calls=(call,),
                latency_s=time.time() - started,
                at=started,
            )
            trajectory.observe(call.tool_call_id, json.dumps(outcome))
            if outcome.get("success") is False:
                failures.append(
                    f"step {i} {step['tool']} {outcome.get('state')}: "
                    f"{outcome.get('error') or outcome.get('reason') or ''}".strip()
                )
                if not self.config.continue_on_failure:
                    break
        summary = raw / "summary.json"
        summary.write_text(json.dumps({"steps": len(plan), "failures": failures}, indent=2))
        trajectory.step(
            message="; ".join(failures) if failures else f"all {len(plan)} steps completed",
            request=summary,
            response=summary,
        )
        return trajectory.build("answer")
