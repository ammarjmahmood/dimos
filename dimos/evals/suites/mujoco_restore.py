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

"""xArm7 moves the cylinder 0.10 m in +y and leaves it on the table.

Graded on the recorded ``tf`` pose. A height change past 2 cm scores 0.

    dimos evals run dimos.evals.suites.mujoco_restore --agent dimos.evals.agents.pi
"""

from __future__ import annotations

from collections.abc import Callable
import math

from dimos.evals.environments.lib.recorded_poses import first_body_transform, last_body_transform
from dimos.evals.environments.mujoco_sim import MujocoEnvironment
from dimos.evals.scorers import ramp
from dimos.evals.types import EvalCase, Outcome, Suite, recording
from dimos.utils.data import LfsPath

PERCEPTION_MODULES = (
    "object-scene-registration-module",
    "pick-and-place-module",
    "heuristic-grasp-module",
)


def restored_offset(
    dx: float,
    dy: float,
    start: tuple[float, float],
    end: tuple[float, float],
    band: float = 0.05,
    start_z: float | None = None,
    end_z: float | None = None,
    z_band: float = 0.02,
) -> float:
    """1.0 when ``end`` is ``(dx, dy)`` from ``start`` and the height did not change.

    A height change past ``z_band`` means the body was lifted or fell off the table.
    """
    if start_z is not None and end_z is not None and abs(end_z - start_z) > z_band:
        return 0.0
    err = math.hypot((end[0] - start[0]) - dx, (end[1] - start[1]) - dy)
    return ramp(err, band=band)


def restored(body: str, dx: float, dy: float, *, band: float = 0.05) -> Callable[[Outcome], float]:
    """Credit for finishing ``body`` at ``(dx, dy)`` from its first recorded pose."""

    def grade(outcome: Outcome) -> float:
        with recording(outcome) as store:
            try:
                start = first_body_transform(store, body).translation
                end = last_body_transform(store, body).translation
            except LookupError:
                return 0.0
        return restored_offset(
            dx=dx,
            dy=dy,
            start=(start.x, start.y),
            end=(end.x, end.y),
            band=band,
            start_z=start.z,
            end_z=end.z,
        )

    return grade


restore_cup = EvalCase(
    id="xarm_restore_cup",
    inputs=(
        "Move the cylinder 0.10 m in +y from where it started and leave it on the table. "
        "The table spans roughly x=0.30 to 0.60 m and y=-0.20 to 0.20 m, with the top at "
        "z=0.13 m. Find the cylinder through the wrist camera. The gripper starts pointing "
        "straight down: for top-down moves omit roll/pitch/yaw in move_to_pose to keep the "
        "current orientation, or pass roll=3.1416, pitch=0."
    ),
    environment=MujocoEnvironment(
        blueprint=["xarm-perception-sim", "mcp-server", "observe-skill"],
        disable=PERCEPTION_MODULES,
        scene=LfsPath("xarm7/scene.xml"),
        tracked_bodies=("cup",),
    ),
    grade=restored("cup", dx=0.0, dy=0.10),
    timeout_s=600.0,
    tags=frozenset({"mujoco", "restore"}),
)

SUITE: Suite = [restore_cup]
