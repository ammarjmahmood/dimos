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

"""Original robosuite Lift, driven by ordinary DimOS manipulation skills.

The upstream environment owns placement and success. The agent gets its wrist
camera and planner skills, never the success oracle or privileged object poses.
"""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING, cast

from dimos.evals.environments.mujoco_sim import MujocoEnvironment
from dimos.evals.suites.mujoco_xarm import PERCEPTION_MODULES
from dimos.evals.types import EvalCase, Outcome, Suite
from dimos.porcelain.dimos import Dimos
from dimos.sim2.scene_types import SceneControlSpec, SceneState

if TYPE_CHECKING:
    from dimos.memory.store.base import Store


class LiftEnvironment(MujocoEnvironment):
    def prepare_recording(self, recording: Store, path: Path, deadline: float) -> dict[str, Path]:
        super().prepare_recording(recording, path, deadline)
        result = path.with_name("lift-result.json")
        result.unlink(missing_ok=True)
        # Registered last: capture before closing the recording and simulator.
        self._resources.callback(self._save_result, result)
        return {"task_result": result}

    def _save_result(self, path: Path) -> None:
        app = Dimos.connect()
        try:
            state = cast("SceneControlSpec", app.SimulationModule).scene_state()
            _task_success(state)
            path.write_text(state.model_dump_json(indent=2) + "\n")
        finally:
            app.stop()


def _task_success(state: SceneState) -> float:
    if state.scene_id != "robosuite:Lift" or state.task_success is None:
        raise ValueError("Lift result does not contain the upstream task's success result")
    return float(state.task_success)


def upstream_success(outcome: Outcome) -> float:
    state = SceneState.model_validate_json(
        outcome.artifacts["task_result"].read_text(), strict=True
    )
    return _task_success(state)


SUITE: Suite = [
    EvalCase(
        id="xarm_robosuite_lift",
        inputs=(
            "Pick up the red cube and hold it at least 10 cm above the table. "
            "Find it through the wrist camera. The tabletop is at world z=0.8 m, "
            "centred on x=y=0, and 0.8 m square. The arm base is at (-0.4, 0, 0.8). "
            "For top-down moves omit roll/pitch/yaw to keep the initial gripper orientation, "
            "or use roll=3.1416, pitch=0. roll=pitch=yaw=0 points the gripper up."
        ),
        environment=LiftEnvironment(
            blueprint=["xarm-robosuite-lift", "mcp-server", "observe-skill"],
            disable=PERCEPTION_MODULES,
        ),
        grade=upstream_success,
        timeout_s=600.0,
        tags=frozenset({"robosuite", "mujoco", "manipulation", "pick"}),
    )
]
