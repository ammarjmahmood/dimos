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

"""Read-only worker-local model/data; physics publishes only integration state."""

from __future__ import annotations

from typing import Any

import mujoco

from dimos.sim2.ipc.abi import ChannelDescriptor
from dimos.sim2.ipc.channel import RobotChannel
from dimos.sim2.runtime import STATE


class WorldReader:
    def __init__(self, description: dict[str, Any]) -> None:
        self.model = mujoco.MjModel.from_binary_path(description["model"])
        self.data = mujoco.MjData(self.model)
        self.channel = RobotChannel.attach(ChannelDescriptor.from_dict(description["snapshot"]))
        self.sequence = -1
        self.timestamp = 0.0

    def update(self) -> bool:
        if self.channel.lifecycle != "ready":
            return False
        frame = self.channel.read_observation()
        if frame is None or frame.metadata.sequence == self.sequence:
            return False
        mujoco.mj_setState(self.model, self.data, frame.values["state"], STATE)
        mujoco.mj_forward(self.model, self.data)
        self.sequence = frame.metadata.sequence
        self.timestamp = float(frame.values["wall_time"][0])
        return True

    def close(self) -> None:
        self.channel.close()
