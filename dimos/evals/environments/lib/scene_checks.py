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


"""Checks on what an object did during a trial, read from sim2's recorded ``sim_truth``.

Each function takes the opened recording and returns a score 0..1, so a suite can
combine them with ``dimos.evals.scorers.weighted``. Entities and regions are named by
their ids in the scene's ``scene.json`` (``cracker_box``, ``dropoff/top``).
"""

from __future__ import annotations

from collections.abc import Sequence
import math
from typing import TYPE_CHECKING, cast

from dimos.evals.environments.lib.sim_truth import first_entity_pose, last_entity_pose

if TYPE_CHECKING:
    from dimos.memory.store.base import Store
    from dimos.sim2.scene_types import SceneState


def lifted_by(recording: Store, entity: str, by_m: float) -> float:
    """How high the object was lifted above where it started, at any moment of the trial.

    Returns 1.0 once the object rose ``by_m`` metres or more, scaled down to 0.0 for no
    rise. The peak counts, not the end, so an object that was lifted and then dropped
    still scores as lifted; that is what a grasp-success count needs.
    """
    if by_m <= 0:
        raise ValueError("by_m must be positive")
    start = first_entity_pose(recording, entity).position.z
    peak = max(
        _state(row).entities[entity].pose.position.z
        for row in recording.streams.sim_truth
        if entity in _state(row).entities
    )
    return min(max((peak - start) / by_m, 0.0), 1.0)


def resting_in_region(
    recording: Store,
    entity: str,
    region: str,
    *,
    surface_band_m: tuple[float, float] = (-0.05, 0.02),
    at_rest_m_s: float = 0.01,
) -> float:
    """1.0 when the object ended inside the region's footprint, sitting on its surface and
    still; otherwise 0.0.

    The footprint is the region's size in its own x and y. "On its surface" means the
    object's lowest point is within ``surface_band_m`` (metres below, metres above) of the
    region's height, which rejects an object still held in the air over the region or
    one that fell to the floor beside it. "Still" means slower than ``at_rest_m_s``.
    """
    state = _state(recording.streams.sim_truth.last())
    try:
        target = state.entities[entity]
        area = state.regions[region]
    except KeyError as e:
        raise LookupError(f"No sim_truth record for {e.args[0]!r}") from None
    # The object's position in the region's own frame, so a turned region still works.
    local = area.pose.orientation.inverse().rotate_vector(target.pose.position - area.pose.position)
    inside = abs(local.x) <= area.size[0] / 2 and abs(local.y) <= area.size[1] / 2
    low, high = surface_band_m
    on_surface = low <= target.bounds_min[2] - area.pose.position.z <= high
    still = math.hypot(*target.velocity) <= at_rest_m_s
    return float(inside and on_surface and still)


def unmoved(recording: Store, entities: Sequence[str], *, tolerance_m: float = 0.02) -> float:
    """The share of ``entities`` that ended within ``tolerance_m`` metres of where they
    started: 1.0 when nothing else on the table was disturbed."""
    if not entities:
        return 1.0
    still = sum(displacement(recording, entity) <= tolerance_m for entity in entities)
    return still / len(entities)


def displacement(recording: Store, entity: str) -> float:
    """How far, in metres, the object's final position is from its starting one."""
    start = first_entity_pose(recording, entity).position
    end = last_entity_pose(recording, entity).position
    return (end - start).length()


def _state(row: object) -> SceneState:
    return cast("SceneState", getattr(row, "data", row))
