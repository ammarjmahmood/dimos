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

"""Ground-truth object poses that sim2 published on ``sim_truth``."""

from __future__ import annotations

from typing import TYPE_CHECKING, cast

if TYPE_CHECKING:
    from dimos.memory.store.base import Store
    from dimos.msgs.geometry_msgs.Pose import Pose
    from dimos.sim2.scene_types import SceneState


def last_entity_pose(recording: Store, entity: str) -> Pose:
    """The newest world-frame pose of a scene entity (its id in the scene's ``scene.json``)."""
    return _entity_pose(recording, entity, newest=True)


def first_entity_pose(recording: Store, entity: str) -> Pose:
    """The oldest world-frame pose of a scene entity (its id in the scene's ``scene.json``)."""
    return _entity_pose(recording, entity, newest=False)


def _entity_pose(recording: Store, entity: str, *, newest: bool) -> Pose:
    if "sim_truth" not in recording.streams:
        raise LookupError("No sim_truth recorded; was truth enabled on the SimulationModule?")
    stream = recording.streams.sim_truth
    state = cast("SceneState", (stream.last() if newest else stream.first()).data)
    try:
        return state.entities[entity].pose
    except KeyError:
        raise LookupError(
            f"No sim_truth entity {entity!r}; the scene has {sorted(state.entities)}"
        ) from None
