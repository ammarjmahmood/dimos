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

"""xArm7 at a table with a red ball (``apple``) and a cylinder (``cup``): pick up the cylinder,
then put the red ball on top of it, graded on the bodies' recorded poses. Each task comes in
two variants. ``raw`` gives the agent the planner skills and a wrist-camera image only.
``perception`` keeps the scene-registration and pick-and-place modules in the launch, so the
agent can also scan the view for objects, pick one by id and place it at a position.

    dimos evals run dimos.evals.suites.mujoco_xarm --agent dimos.evals.agents.pi --tags raw
    dimos evals run dimos.evals.suites.mujoco_xarm --agent dimos.evals.agents.pi --tags perception
"""

from __future__ import annotations

from collections.abc import Callable
import math
from typing import TYPE_CHECKING, Literal

from dimos.evals.environments.lib.recorded_poses import first_body_transform, last_body_transform
from dimos.evals.environments.mujoco_sim import MujocoEnvironment
from dimos.evals.scorers import ramp
from dimos.evals.types import EvalCase, Outcome, Suite, recording

if TYPE_CHECKING:
    from dimos.memory.store.base import Store
    from dimos.msgs.geometry_msgs.Vector3 import Vector3

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

PERCEPTION_SCENE = (
    f"{SCENE} You also have perception skills: scan_objects(prompts=[...]) looks for the named "
    "objects in the current wrist-camera view and returns an object_id for each one it finds, "
    "pick_object(object_id) grasps that object, and place_at(x, y, z) lowers the held object "
    "to that world-frame position and releases it. Neither reports where an object is; work "
    "that out from the camera and the table layout above."
)

Variant = Literal["raw", "perception"]


def environment(variant: Variant) -> MujocoEnvironment:
    """The table scene. ``raw`` launches it without the perception modules."""
    return MujocoEnvironment(
        blueprint=["xarm-perception-sim", "mcp-server", "observe-skill"],
        disable=() if variant == "perception" else PERCEPTION_MODULES,
        tracked_bodies=TRACKED,
    )


# Where a body's recorded position comes from: readers of its first and last
# world-frame position in the recording, raising LookupError when it is missing.
Positions = tuple[Callable[["Store", str], "Vector3"], Callable[["Store", str], "Vector3"]]


def _first_tf(store: Store, body: str) -> Vector3:
    return first_body_transform(store, body).translation


def _last_tf(store: Store, body: str) -> Vector3:
    return last_body_transform(store, body).translation


# MujocoEnvironment's simulator publishes ground truth as world -> body transforms on tf.
TF_POSITIONS: Positions = (_first_tf, _last_tf)


def lifted(
    body: str, *, by_m: float, positions: Positions = TF_POSITIONS
) -> Callable[[Outcome], float]:
    """How far the body ended above where it started, full credit at ``by_m``."""
    first, last = positions

    def grade(outcome: Outcome) -> float:
        with recording(outcome) as store:
            try:
                start = first(store, body).z
                end = last(store, body).z
            except LookupError:
                return 0.0
        return min(max((end - start) / by_m, 0.0), 1.0)

    return grade


def stacked_on(
    top: str,
    base: str,
    *,
    rise_m: tuple[float, float],
    band_m: float,
    positions: Positions = TF_POSITIONS,
) -> Callable[[Outcome], float]:
    """``top`` ended resting on ``base``: its centre ``rise_m`` above the base's, 1.0 centred and
    0.0 at ``band_m`` off. A body held higher than the resting height scores 0.0."""
    _, last = positions

    def grade(outcome: Outcome) -> float:
        with recording(outcome) as store:
            try:
                t = last(store, top)
                b = last(store, base)
            except LookupError:
                return 0.0
        if not rise_m[0] <= t.z - b.z <= rise_m[1]:
            return 0.0
        return ramp(math.hypot(t.x - b.x, t.y - b.y), band=band_m)

    return grade


def cases(variant: Variant) -> list[EvalCase]:
    """Both tasks for one variant. The ``raw`` ids are bare; ``perception`` ids carry a suffix."""
    suffix = "" if variant == "raw" else f"_{variant}"
    scene = SCENE if variant == "raw" else PERCEPTION_SCENE
    tags = frozenset({"mujoco", "manipulation", "pick", variant})
    return [
        EvalCase(
            id=f"xarm_pick_cylinder{suffix}",
            inputs=f"Pick up the cylinder from the table and hold it in the air. {scene}",
            environment=environment(variant),
            grade=lifted("cup", by_m=0.05),
            timeout_s=600.0,
            tags=tags,
        ),
        EvalCase(
            id=f"xarm_ball_on_cylinder{suffix}",
            inputs=f"Pick up the red ball and place it on top of the cylinder. {scene}",
            environment=environment(variant),
            grade=stacked_on("apple", "cup", rise_m=(0.08, 0.12), band_m=0.07),
            timeout_s=900.0,
            threshold=0.5,
            tags=tags | {"place"},
        ),
    ]


SUITE: Suite = [*cases("raw"), *cases("perception")]
