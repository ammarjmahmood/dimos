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

"""xArm7 tidy-the-table cases: restore a displaced cylinder, or leave a tidy table alone.

The agent prompt states the cleaning duty and expected layout only — no table
bounds or displacements. Graders compare recorded ``tf`` to the usual cup pose
in ``xarm7/scene.xml`` (or to the start pose for the already-tidy control).
Off-table finishes score 0.

    dimos evals run dimos.evals.suites.mujoco_restore --agent dimos.evals.agents.pi
"""

from __future__ import annotations

from collections.abc import Callable
import math
from pathlib import Path
from typing import TYPE_CHECKING

from dimos.evals.environments.lib.recorded_poses import first_body_transform, last_body_transform
from dimos.evals.environments.mujoco_sim import MujocoEnvironment
from dimos.evals.scorers import ramp
from dimos.evals.types import EvalCase, Outcome, Suite, recording
from dimos.utils.data import LfsPath

if TYPE_CHECKING:
    from dimos.e2e_tests.dimos_cli_call import DimosCliCall

PERCEPTION_MODULES = (
    "object-scene-registration-module",
    "pick-and-place-module",
    "heuristic-grasp-module",
)

# Usual cup pose in data/xarm7/scene.xml. Not mentioned in the agent prompt.
CUP_HOME = (0.50, 0.0)
# Displaced start for the restore case; still on the table inset.
_MESSY_CUP_Y = -0.10

# table_top box is centered at (0.45, 0) with half-size (0.15, 0.20).
# The cup cylinder radius is 0.035, so its center must stay that far inside.
_TABLE_X = (0.30, 0.60)
_TABLE_Y = (-0.20, 0.20)
_CUP_RADIUS = 0.035

_DUTY = (
    "You are a cleaning arm that is periodically woken by a cron job to tidy the table. "
    "The table is expected to have a cylinder in its usual place among the other items. "
    "Feel free to act as you see fit. Go."
)

_CUP_BODY = '<body name="cup" pos="0.50 0.0 0.19">'


def _cup_on_table(x: float, y: float) -> bool:
    return (
        _TABLE_X[0] + _CUP_RADIUS <= x <= _TABLE_X[1] - _CUP_RADIUS
        and _TABLE_Y[0] + _CUP_RADIUS <= y <= _TABLE_Y[1] - _CUP_RADIUS
    )


def at_xy(
    end: tuple[float, float],
    target: tuple[float, float],
    *,
    band: float = 0.05,
    start_z: float | None = None,
    end_z: float | None = None,
    z_band: float = 0.02,
) -> float:
    """1.0 when ``end`` is near ``target`` and still on the table."""
    if start_z is not None and end_z is not None and abs(end_z - start_z) > z_band:
        return 0.0
    if not _cup_on_table(end[0], end[1]):
        return 0.0
    return ramp(math.hypot(end[0] - target[0], end[1] - target[1]), band=band)


def stayed_xy(
    start: tuple[float, float],
    end: tuple[float, float],
    *,
    band: float = 0.05,
    start_z: float | None = None,
    end_z: float | None = None,
    z_band: float = 0.02,
) -> float:
    """1.0 when ``end`` stays near ``start`` and on the table (already-tidy control)."""
    return at_xy(end, start, band=band, start_z=start_z, end_z=end_z, z_band=z_band)


def near_home(
    body: str, home: tuple[float, float] = CUP_HOME, *, band: float = 0.05
) -> Callable[[Outcome], float]:
    """Credit for finishing ``body`` near the usual place."""

    def grade(outcome: Outcome) -> float:
        with recording(outcome) as store:
            try:
                start = first_body_transform(store, body).translation
                end = last_body_transform(store, body).translation
            except LookupError:
                return 0.0
        return at_xy(
            (end.x, end.y),
            home,
            band=band,
            start_z=start.z,
            end_z=end.z,
        )

    return grade


def stayed_put(body: str, *, band: float = 0.05) -> Callable[[Outcome], float]:
    """Credit for leaving ``body`` where the episode started."""

    def grade(outcome: Outcome) -> float:
        with recording(outcome) as store:
            try:
                start = first_body_transform(store, body).translation
                end = last_body_transform(store, body).translation
            except LookupError:
                return 0.0
        return stayed_xy(
            (start.x, start.y),
            (end.x, end.y),
            band=band,
            start_z=start.z,
            end_z=end.z,
        )

    return grade


def _materialize_cup_y(y: float) -> Path:
    """Write an xarm7 scene with the cup at ``y``, beside the LFS assets (for includes)."""
    stock = LfsPath("xarm7/scene.xml")
    root = Path(str(stock)).parent
    text = (root / "scene.xml").read_text()
    if _CUP_BODY not in text:
        raise RuntimeError("xarm7/scene.xml no longer has the expected cup pose marker")
    out = root / f"scene_cup_y{y:g}_eval.xml"
    replacement = f'<body name="cup" pos="0.50 {y:g} 0.19">'
    out.write_text(text.replace(_CUP_BODY, replacement, 1))
    return out


class _MessyCupEnv(MujocoEnvironment):
    """Stock table with the cup displaced from ``CUP_HOME`` before the agent runs."""

    def configure_launch(self, proc: DimosCliCall) -> None:
        self.config.scene = _materialize_cup_y(_MESSY_CUP_Y)
        super().configure_launch(proc)


def _tidy_env() -> MujocoEnvironment:
    return MujocoEnvironment(
        blueprint=["xarm-perception-sim", "mcp-server", "observe-skill"],
        disable=PERCEPTION_MODULES,
        scene=LfsPath("xarm7/scene.xml"),
        tracked_bodies=("cup",),
    )


def _messy_env() -> MujocoEnvironment:
    return _MessyCupEnv(
        blueprint=["xarm-perception-sim", "mcp-server", "observe-skill"],
        disable=PERCEPTION_MODULES,
        scene=LfsPath("xarm7/scene.xml"),
        tracked_bodies=("cup",),
    )


restore_cup = EvalCase(
    id="xarm_restore_cup",
    inputs=_DUTY,
    environment=_messy_env(),
    grade=near_home("cup"),
    timeout_s=600.0,
    tags=frozenset({"mujoco", "restore"}),
)

already_tidy = EvalCase(
    id="xarm_table_already_tidy",
    inputs=_DUTY,
    environment=_tidy_env(),
    grade=stayed_put("cup"),
    timeout_s=600.0,
    tags=frozenset({"mujoco", "restore", "control"}),
)

SUITE: Suite = [restore_cup, already_tidy]
