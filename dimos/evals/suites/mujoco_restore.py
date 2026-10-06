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

The agent prompt says the cylinder belongs in the middle of the table — no
meter values. Graders score distance from the table-top center; finishes
outside a 0.1 m radius (or off the table) get 0.

    dimos evals run dimos.evals.suites.mujoco_restore --agent dimos.evals.agents.pi
"""

from __future__ import annotations

from collections.abc import Callable
import math
from pathlib import Path
from typing import TYPE_CHECKING, Any

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

# table_top box is centered at (0.45, 0) with half-size (0.15, 0.20).
# Usual place for the agent: middle of the table. Not given as coordinates.
TABLE_CENTER = (0.45, 0.0)
# Full credit at the center; 0 at and beyond this radius (matches messy offset).
_SCORE_BAND_M = 0.10
# Messy start: 0.10 m from the center along -y, still on the table inset.
_MESSY_CUP = (TABLE_CENTER[0], TABLE_CENTER[1] - _SCORE_BAND_M)

# The cup cylinder radius is 0.035, so its center must stay that far inside.
_TABLE_X = (0.30, 0.60)
_TABLE_Y = (-0.20, 0.20)
_CUP_RADIUS = 0.035

_DUTY = (
    "You are a cleaning arm that is periodically woken by a cron job to tidy the table. "
    "The table is expected to have a cylinder in the middle of the table among the other "
    "items. Feel free to act as you see fit. Go."
)

_STOCK_CUP_BODY = '<body name="cup" pos="0.50 0.0 0.19">'


def _cup_on_table(x: float, y: float) -> bool:
    return (
        _TABLE_X[0] + _CUP_RADIUS <= x <= _TABLE_X[1] - _CUP_RADIUS
        and _TABLE_Y[0] + _CUP_RADIUS <= y <= _TABLE_Y[1] - _CUP_RADIUS
    )


def at_xy(
    end: tuple[float, float],
    target: tuple[float, float],
    *,
    band: float = _SCORE_BAND_M,
    start_z: float | None = None,
    end_z: float | None = None,
    z_band: float = 0.02,
) -> float:
    """1.0 at ``target``, linear to 0 at ``band``, and 0 off the table."""
    if start_z is not None and end_z is not None and abs(end_z - start_z) > z_band:
        return 0.0
    if not _cup_on_table(end[0], end[1]):
        return 0.0
    return ramp(math.hypot(end[0] - target[0], end[1] - target[1]), band=band)


def stayed_xy(
    start: tuple[float, float],
    end: tuple[float, float],
    *,
    band: float = _SCORE_BAND_M,
    start_z: float | None = None,
    end_z: float | None = None,
    z_band: float = 0.02,
) -> float:
    """1.0 when ``end`` stays near ``start`` and on the table (already-tidy control)."""
    return at_xy(end, start, band=band, start_z=start_z, end_z=end_z, z_band=z_band)


def near_table_center(
    body: str, center: tuple[float, float] = TABLE_CENTER, *, band: float = _SCORE_BAND_M
) -> Callable[[Outcome], float]:
    """Credit for finishing ``body`` near the middle of the table."""

    def grade(outcome: Outcome) -> float:
        with recording(outcome) as store:
            try:
                start = first_body_transform(store, body).translation
                end = last_body_transform(store, body).translation
            except LookupError:
                return 0.0
        return at_xy(
            (end.x, end.y),
            center,
            band=band,
            start_z=start.z,
            end_z=end.z,
        )

    return grade


def stayed_put(body: str, *, band: float = _SCORE_BAND_M) -> Callable[[Outcome], float]:
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


def _materialize_cup_at(x: float, y: float) -> Path:
    """Write an xarm7 scene with the cup at ``(x, y)``, beside the LFS assets."""
    stock = LfsPath("xarm7/scene.xml")
    root = Path(str(stock)).parent
    text = (root / "scene.xml").read_text()
    if _STOCK_CUP_BODY not in text:
        raise RuntimeError("xarm7/scene.xml no longer has the expected cup pose marker")
    out = root / f"scene_cup_x{x:g}_y{y:g}_eval.xml"
    replacement = f'<body name="cup" pos="{x:g} {y:g} 0.19">'
    out.write_text(text.replace(_STOCK_CUP_BODY, replacement, 1))
    return out


class _CupSceneEnv(MujocoEnvironment):
    """Launch with the cup rewritten to a fixed ``(x, y)`` on the stock table."""

    def __init__(self, cup_xy: tuple[float, float], **kwargs: Any) -> None:
        self._cup_xy = cup_xy
        super().__init__(**kwargs)

    def configure_launch(self, proc: DimosCliCall) -> None:
        self.config.scene = _materialize_cup_at(*self._cup_xy)
        super().configure_launch(proc)


def _env(cup_xy: tuple[float, float]) -> MujocoEnvironment:
    return _CupSceneEnv(
        cup_xy,
        blueprint=["xarm-perception-sim", "mcp-server", "observe-skill"],
        disable=PERCEPTION_MODULES,
        scene=LfsPath("xarm7/scene.xml"),
        tracked_bodies=("cup",),
    )


restore_cup = EvalCase(
    id="xarm_restore_cup",
    inputs=_DUTY,
    environment=_env(_MESSY_CUP),
    grade=near_table_center("cup"),
    timeout_s=600.0,
    tags=frozenset({"mujoco", "restore"}),
)

already_tidy = EvalCase(
    id="xarm_table_already_tidy",
    inputs=_DUTY,
    environment=_env(TABLE_CENTER),
    grade=stayed_put("cup"),
    timeout_s=600.0,
    tags=frozenset({"mujoco", "restore", "control"}),
)

SUITE: Suite = [restore_cup, already_tidy]
