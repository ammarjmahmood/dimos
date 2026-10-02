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

"""Physical success checks for the fixed robosuite_xarm exports.

Dimensions below belong to the exported seed-0 assets, not arbitrary robosuite
scenes. ToolHang uses the assembly, threading and contact criteria described by
robosuite 1.5.2, with release and final-window stability required for all placements.
"""

from __future__ import annotations

from collections.abc import Callable
from itertools import pairwise
import math

import numpy as np
from numpy.typing import NDArray

from dimos.evals.types import Outcome, recording
from dimos.simulation.engines.mujoco_evaluation import MujocoEvaluationState as State


def touching(state: State, a: str, b: str) -> bool:
    return (a, b) in state.contacts or (b, a) in state.contacts


def lift(state: State) -> bool:
    return bool(
        state.positions["cube_main"][2] >= state.sites["table_top"][2] + 0.02 + 0.05 - 0.001
        and "cube_main" in state.robot_contacts
    )


def door(state: State) -> bool:
    relative = state.rotations["Door_frame"].T @ state.rotations["Door_door"]
    angle = math.atan2(relative[1, 0], relative[0, 0])
    return angle >= 0.3 and "Door_door" not in state.robot_contacts


def pick_place(state: State) -> bool:
    delta = state.positions["Can_main"] - state.positions["VisualCan_main"]
    return bool(
        np.all(np.abs(delta) <= [0.05, 0.075, 0.005])
        and state.rotations["Can_main"][2, 2] > math.cos(math.radians(10))
        and "Can_main" not in state.robot_contacts
    )


def stack(state: State) -> bool:
    delta = state.positions["cubeA_main"] - state.positions["cubeB_main"]
    return bool(
        np.linalg.norm(delta[:2]) < 0.015
        and abs(delta[2] - 0.045) < 0.004
        and abs(state.positions["cubeB_main"][2] - state.sites["table_top"][2] - 0.025) < 0.004
        and abs(state.rotations["cubeA_main"][2, 2]) > 0.98
        and abs(state.rotations["cubeB_main"][2, 2]) > 0.98
        and touching(state, "cubeA_g0", "cubeB_g0")
        and not state.robot_contacts.intersection({"cubeA_main", "cubeB_main"})
    )


def nut_assembly(state: State) -> bool:
    nut = state.positions["SquareNut_main"]
    rotation = state.rotations["SquareNut_main"]
    # The square hole's half-width is 0.02275 m; the peg's is 0.016 m.
    corners = np.array([[x, y, 0] for x in (-0.016, 0.016) for y in (-0.016, 0.016)])
    corners = corners @ state.rotations["peg1"].T + state.positions["peg1"]
    corners[:, 2] = nut[2]
    in_nut = (corners - nut) @ rotation
    return bool(
        np.all(np.abs(in_nut[:, :2]) < 0.02375)
        and abs(nut[2] - state.sites["table_top"][2] - 0.01) < 0.004
        and abs(rotation[2, 2]) > 0.98
        and "SquareNut_main" not in state.robot_contacts
        and any(touching(state, f"SquareNut_g{i}", "table_collision") for i in range(5))
    )


def _opposite_sides(
    a: NDArray[np.float64], b: NDArray[np.float64], direction: NDArray[np.float64]
) -> bool:
    return bool(np.dot(np.cross(a, direction), np.cross(b, direction)) < 0)


def tool_hang(state: State) -> bool:
    sites, geoms = state.sites, state.geoms
    shaft = sites["stand_mount_site"] - geoms["stand_base"]
    shaft_length = float(np.linalg.norm(shaft))
    if shaft_length == 0 or shaft[2] / shaft_length < math.cos(math.radians(10)):
        return False
    if np.linalg.norm(sites["frame_tip_site"] - geoms["stand_base"]) >= 0.05:
        return False
    mount = sites["frame_mount_site"]
    post = sites["frame_intersection_site"] - mount
    for a, b in ((0, 2), (1, 3)):
        if not _opposite_sides(
            geoms[f"stand_wall{a}"] - mount, geoms[f"stand_wall{b}"] - mount, post
        ):
            return False

    end = sites["frame_hang_site"]
    hook = sites["frame_intersection_site"] - end
    length = float(np.linalg.norm(hook))
    if length == 0:
        return False
    direction = hook / length
    offset = sites["tool_hole1_center"] - end
    projection = float(np.dot(offset, direction))
    # 10.5 mm inner hole radius minus 3.75 mm hook half-thickness.
    if np.linalg.norm(offset - projection * direction) >= 0.00675:
        return False
    return bool(
        0.05 < projection / length < 1.0
        and _opposite_sides(
            geoms["tool_hole1_hc_0"] - end, geoms["tool_hole1_hc_4"] - end, direction
        )
        and any(touching(state, f"tool_hole1_hc_{i}", "frame_horizontal_frame") for i in range(8))
        and not state.robot_contacts.intersection({"stand_root", "frame_root", "tool_root"})
    )


def sustained(
    predicate: Callable[[State], bool], bodies: tuple[str, ...]
) -> Callable[[Outcome], float]:
    """Binary success throughout the final second, with <=5 mm / 0.05 rad drift.

    Missing, nonfinite, stale, sparse or incomplete terminal evidence fails closed.
    No live simulator or robosuite installation is needed to grade a recording.
    """

    def grade(outcome: Outcome) -> float:
        with recording(outcome) as store:
            if "evaluation_state" not in store.streams:
                return 0.0
            states: list[State] = []
            for record in store.streams.evaluation_state.order_by("ts", desc=True):
                state = record.data
                if not isinstance(state, State) or not math.isfinite(state.ts):
                    return 0.0
                states.append(state)
                if states[0].ts - state.ts >= 1.0:
                    break
            if len(states) < 2 or states[0].ts - states[-1].ts < 1.0:
                return 0.0
            for name in ("coordinator_joint_state", "color_image"):
                if name in store.streams:
                    try:
                        latest = getattr(store.streams, name).last().data.ts
                    except LookupError:
                        continue
                    if latest - states[0].ts > 0.5:
                        return 0.0
        try:
            for newer, older in pairwise(states):
                if not 0 < newer.ts - older.ts <= 0.25:
                    return 0.0
            for state in states:
                arrays = (
                    *state.positions.values(),
                    *state.rotations.values(),
                    *state.sites.values(),
                    *state.geoms.values(),
                )
                if not all(np.isfinite(value).all() for value in arrays) or not predicate(state):
                    return 0.0
                for body in bodies:
                    if np.linalg.norm(state.positions[body] - states[0].positions[body]) > 0.005:
                        return 0.0
                    relative = state.rotations[body].T @ states[0].rotations[body]
                    angle = math.acos(float(np.clip((np.trace(relative) - 1) / 2, -1, 1)))
                    if angle > 0.05:
                        return 0.0
        except (KeyError, ValueError, IndexError):
            return 0.0
        return 1.0

    return grade
