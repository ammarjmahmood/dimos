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

"""Opt-in, synchronized ground truth for offline manipulation grading."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np
from numpy.typing import NDArray

if TYPE_CHECKING:
    import mujoco


@dataclass(frozen=True)
class MujocoEvaluationState:
    ts: float
    positions: dict[str, NDArray[np.float64]]
    rotations: dict[str, NDArray[np.float64]]
    sites: dict[str, NDArray[np.float64]]
    geoms: dict[str, NDArray[np.float64]]
    contacts: frozenset[tuple[str, str]]
    robot_contacts: frozenset[str]


class MujocoEvaluationRecorder:
    """Resolve names once; capture from the physics thread after a step.

    Contact with any descendant of a tracked body counts towards that body.
    This includes compound objects such as the wrench's separate hole bodies.
    """

    def __init__(
        self,
        model: mujoco.MjModel,
        *,
        bodies: list[str],
        sites: list[str],
        geoms: list[str],
        robot_body: str,
    ) -> None:
        self.bodies = {name: model.body(name).id for name in bodies}
        self.sites = {name: model.site(name).id for name in sites}
        self.geoms = {name: model.geom(name).id for name in geoms}
        robot_id = model.body(robot_body).id
        tracked_ids = {index: name for name, index in self.bodies.items()}
        self.robot_geoms: set[int] = set()
        self.owners: dict[int, str] = {}
        self.geom_names = [model.geom(i).name or f"geom:{i}" for i in range(model.ngeom)]
        for geom in range(model.ngeom):
            body = int(model.geom_bodyid[geom])
            while body:
                if body == robot_id:
                    self.robot_geoms.add(geom)
                if body in tracked_ids and geom not in self.owners:
                    self.owners[geom] = tracked_ids[body]
                body = int(model.body_parentid[body])

    def capture(self, data: mujoco.MjData, ts: float) -> MujocoEvaluationState:
        contacts: set[tuple[str, str]] = set()
        robot_contacts: set[str] = set()
        for contact in data.contact[: data.ncon]:
            if contact.dist > 0:
                continue
            a, b = int(contact.geom1), int(contact.geom2)
            contacts.add((self.geom_names[a], self.geom_names[b]))
            for robot, other in ((a, b), (b, a)):
                if robot in self.robot_geoms and other in self.owners:
                    robot_contacts.add(self.owners[other])
        return MujocoEvaluationState(
            ts=ts,
            positions={name: data.xpos[i].copy() for name, i in self.bodies.items()},
            rotations={name: data.xmat[i].reshape(3, 3).copy() for name, i in self.bodies.items()},
            sites={name: data.site_xpos[i].copy() for name, i in self.sites.items()},
            geoms={name: data.geom_xpos[i].copy() for name, i in self.geoms.items()},
            contacts=frozenset(contacts),
            robot_contacts=frozenset(robot_contacts),
        )
