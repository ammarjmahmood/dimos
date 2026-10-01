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

"""xArm7 at a table, planner skills and the wrist camera only. Two picks with a red ball
(``apple``) and a cylinder (``cup``): the pick is graded on what its probe saw in the simulator,
the placement on the bodies' recorded poses. One pour, in a scene with two cups: a ball tipped
from a handled cup into a wider one, graded on its probe.

    dimos evals run dimos.evals.suites.mujoco_xarm --agent dimos.evals.agents.pi
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field
import math
from typing import TYPE_CHECKING

from dimos.evals.environments.lib.recorded_poses import last_body_transform
from dimos.evals.environments.lib.recorded_probe import probe_ctx
from dimos.evals.environments.mujoco_sim import MujocoEnvironment
from dimos.evals.scorers import ramp
from dimos.evals.types import EvalCase, Outcome, Suite, recording
from dimos.simulation.engines.mujoco_probe import Contact, MujocoProbe, ProbeSetup

if TYPE_CHECKING:
    import mujoco

TRACKED = ("apple", "cup")

PERCEPTION_MODULES = (
    "object-scene-registration-module",
    "pick-and-place-module",
    "heuristic-grasp-module",
)

SCENE = (
    "The table top is at z=0.13 m in the world frame and spans roughly x=0.30 to 0.60 m "
    "ahead of the arm base. On it are a red ball about 8 cm wide, an orange ball about 9 cm "
    "wide and a cylinder about 7 cm wide and 12 cm tall; find where they are by looking "
    "through the wrist camera. The gripper starts pointing straight down: for top-down moves "
    "omit roll/pitch/yaw in move_to_pose to keep the current orientation, or pass "
    "roll=3.1416, pitch=0. roll=pitch=yaw=0 points the gripper up."
)

ARM_LINKS = frozenset({*(f"link{i}" for i in range(1, 8)), "xarm_gripper_base_link"})


@dataclass
class Pick:
    """What the pick probe saw in the simulator."""

    cup_start_z: float | None = None
    """Height of the cylinder's centre on the first physics step."""
    cup_z: float = 0.0
    """Its height on the latest step."""
    table_hits: list[str] = field(default_factory=list)
    """Arm links that hit the table top, in order; the fingers may touch it."""


def pick_probe(probe: MujocoProbe) -> Pick:
    pick = Pick()

    def track_cup(model: mujoco.MjModel, data: mujoco.MjData) -> None:
        pick.cup_z = float(data.body("cup").xpos[2])
        if pick.cup_start_z is None:
            pick.cup_start_z = pick.cup_z

    def hit(contact: Contact) -> None:
        if contact.other in ARM_LINKS:
            pick.table_hits.append(contact.other)

    probe.on_tick(track_cup)
    probe.on_contact_begin("table_top", hit)
    return pick


@dataclass
class Pour:
    """What the pour probe saw in the simulator."""

    ball_in_target: bool = False
    """The ball's centre is inside the yellow cup on the latest step."""
    target_tilt_deg: float = 0.0
    """How far the yellow cup's axis is from vertical on the latest step."""
    source_peak_tilt_deg: float = 0.0
    """The furthest the blue cup was tipped."""
    ball_touched: list[str] = field(default_factory=list)
    """Everything other than the two cups that the ball touched, in order."""


def pour_probe(probe: MujocoProbe) -> Pour:
    pour = Pour()

    def track(model: mujoco.MjModel, data: mujoco.MjData) -> None:
        target = data.body("target_cup")
        axes = target.xmat.reshape(3, 3)
        across, along, up = axes.T @ (data.body("ball").xpos - target.xpos)
        # The yellow cup is 8.2 cm wide inside and 7 cm tall, measured from the centre of its base.
        pour.ball_in_target = bool(math.hypot(across, along) < 0.041 and 0.0 < up < 0.07)
        pour.target_tilt_deg = _tilt_deg(float(axes[2, 2]))
        source_tilt = _tilt_deg(float(data.body("source_cup").xmat[8]))
        pour.source_peak_tilt_deg = max(pour.source_peak_tilt_deg, source_tilt)

    def touched(contact: Contact) -> None:
        if contact.other not in ("source_cup", "target_cup", *pour.ball_touched):
            pour.ball_touched.append(contact.other)

    probe.on_tick(track)
    probe.on_contact_begin("ball", touched)
    return pour


def _tilt_deg(up_z: float) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, up_z))))


def environment(
    probe: ProbeSetup | None = None,
    *,
    scene: str = "xarm-perception-sim",
    tracked_bodies: tuple[str, ...] = TRACKED,
) -> MujocoEnvironment:
    return MujocoEnvironment(
        blueprint=[scene, "mcp-server", "observe-skill"],
        disable=PERCEPTION_MODULES,
        tracked_bodies=tracked_bodies,
        probe=probe,
    )


def picked_up(*, by_m: float) -> Callable[[Outcome], float]:
    """How far the cylinder ended above where it started, full credit at ``by_m``; 0.0 if an
    arm link hit the table top."""

    def grade(outcome: Outcome) -> float:
        with recording(outcome) as store:
            pick = probe_ctx(store, Pick)
        if pick.table_hits or pick.cup_start_z is None:
            return 0.0
        return min(max((pick.cup_z - pick.cup_start_z) / by_m, 0.0), 1.0)

    return grade


def poured(outcome: Outcome) -> float:
    """1.0 if the ball ended in the upright yellow cup having touched only the two cups, 0.5 if
    it ended there after touching anything else (carried or dropped, not poured), else 0.0."""
    with recording(outcome) as store:
        pour = probe_ctx(store, Pour)
    if not pour.ball_in_target or pour.target_tilt_deg > 15.0:
        return 0.0
    return 0.5 if pour.ball_touched else 1.0


def stacked_on(
    top: str, base: str, *, rise_m: tuple[float, float], band_m: float
) -> Callable[[Outcome], float]:
    """``top`` ended resting on ``base``: its centre ``rise_m`` above the base's, 1.0 centred and
    0.0 at ``band_m`` off. A body held higher than the resting height scores 0.0."""

    def grade(outcome: Outcome) -> float:
        with recording(outcome) as store:
            try:
                t = last_body_transform(store, top).translation
                b = last_body_transform(store, base).translation
            except LookupError:
                return 0.0
        if not rise_m[0] <= t.z - b.z <= rise_m[1]:
            return 0.0
        return ramp(math.hypot(t.x - b.x, t.y - b.y), band=band_m)

    return grade


SUITE: Suite = [
    EvalCase(
        id="xarm_pick_cylinder",
        inputs=f"Pick up the cylinder from the table and hold it in the air. {SCENE}",
        environment=environment(pick_probe),
        grade=picked_up(by_m=0.05),
        timeout_s=600.0,
        tags=frozenset({"mujoco", "manipulation", "pick"}),
    ),
    EvalCase(
        id="xarm_ball_on_cylinder",
        inputs=f"Pick up the red ball and place it on top of the cylinder. {SCENE}",
        environment=environment(),
        grade=stacked_on("apple", "cup", rise_m=(0.08, 0.12), band_m=0.07),
        timeout_s=900.0,
        threshold=0.5,
        tags=frozenset({"mujoco", "manipulation", "pick", "place"}),
    ),
    EvalCase(
        id="xarm_pour_ball",
        inputs="Pour the ball into the empty cup.",
        environment=environment(pour_probe, scene="xarm-pour-sim", tracked_bodies=()),
        grade=poured,
        timeout_s=900.0,
        tags=frozenset({"mujoco", "manipulation", "pour"}),
    ),
]
