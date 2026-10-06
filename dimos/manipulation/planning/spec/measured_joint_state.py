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

"""Canonicalize measured state consistently for independent model consumers."""

from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING

import numpy as np

from dimos.msgs.sensor_msgs.JointState import JointState

if TYPE_CHECKING:
    from dimos.manipulation.planning.spec.config import RobotModelConfig


def canonicalize_measured_joint_state(
    msg: JointState, model: RobotModelConfig, *, aliases: Mapping[str, str] | None = None
) -> JointState:
    """Validate and project a complete measurement without changing its stamp.

    Transformed coordinates require their measured source, even if a same-named
    canonical target appears in the message. This prevents accidental unit mixing.
    """
    aliases = aliases or {}
    transforms = model.joint_state_transforms
    sources = {mapping.source: target for target, mapping in transforms.items()}
    canonical_names = [sources.get(name, aliases.get(name, name)) for name in msg.name]
    if (
        len(msg.name) != len(msg.position)
        or len(set(canonical_names)) != len(canonical_names)
        or not np.isfinite(msg.position).all()
        or not np.isfinite(msg.ts)
    ):
        raise ValueError("Malformed measured joint state")
    name_to_idx = {name: i for i, name in enumerate(canonical_names)}
    names = model.joint_names
    missing = [name for name in names if name not in name_to_idx]
    if missing:
        raise ValueError(f"Measured state is missing model joints: {missing}")
    indices = [name_to_idx[name] for name in names]
    positions = [float(msg.position[index]) for index in indices]
    velocities = (
        [float(msg.velocity[index]) for index in indices]
        if len(msg.velocity) == len(msg.name)
        else []
    )
    for index, name in enumerate(names):
        mapping = transforms.get(name)
        if mapping is None:
            continue
        if msg.name[indices[index]] != mapping.source:
            raise ValueError(f"Missing configured measured source for '{name}'")
        positions[index] = mapping.position(positions[index])
        if velocities:
            velocities[index] *= mapping.scale
    return JointState(ts=msg.ts, name=list(names), position=positions, velocity=velocities)
