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

"""Go2 locomotion policies: proprioception and a velocity command in, joint targets out."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Protocol

import numpy as np
from numpy.typing import NDArray
import onnxruntime as ort

from dimos.utils.data import get_data


@dataclass(frozen=True)
class Proprioception:
    """What the policy senses, with joints in the policy's own order."""

    angular_velocity: NDArray[np.float64]
    gravity: NDArray[np.float64]
    joint_position: NDArray[np.float64]
    joint_velocity: NDArray[np.float64]


class Go2Policy(Protocol):
    """A 50 Hz joint-position policy driving the Go2's PD motors."""

    @property
    def joint_names(self) -> tuple[str, ...]: ...

    @property
    def default_pose(self) -> NDArray[np.float64]: ...

    @property
    def kp(self) -> NDArray[np.float64]: ...

    @property
    def kd(self) -> NDArray[np.float64]: ...

    def reset(self) -> None: ...

    def act(self, obs: Proprioception, command: NDArray[np.float64]) -> NDArray[np.float64]:
        """Joint position targets for a (vx, vy, vyaw) command."""
        ...


LEGS = ("FR", "FL", "RR", "RL")


class OnnxGo2Policy:
    """The open rl_sar robot_lab policy: a 45-observation MLP with the legged-gym layout."""

    joint_names = tuple(f"{leg}_{part}_joint" for leg in LEGS for part in ("hip", "thigh", "calf"))
    default_pose = np.array([0.0, 0.8, -1.5] * 4)
    kp = np.full(12, 20.0)
    kd = np.full(12, 0.5)
    action_scale = np.array([0.125, 0.25, 0.25] * 4)
    angular_velocity_scale = 0.25
    joint_velocity_scale = 0.05
    clip_observation = 100.0
    clip_action = 100.0

    def __init__(self, path: str | Path) -> None:
        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        self._session = ort.InferenceSession(str(path), options, providers=["CPUExecutionProvider"])
        self._input = self._session.get_inputs()[0].name
        self._last_action = np.zeros(12)

    @classmethod
    def load(cls) -> OnnxGo2Policy:
        return cls(get_data("go2_sim") / "go2_policy" / "policy.onnx")

    def reset(self) -> None:
        self._last_action = np.zeros(12)

    def act(self, obs: Proprioception, command: NDArray[np.float64]) -> NDArray[np.float64]:
        raw = np.concatenate(
            [
                obs.angular_velocity * self.angular_velocity_scale,
                obs.gravity,
                command,
                obs.joint_position - self.default_pose,
                obs.joint_velocity * self.joint_velocity_scale,
                self._last_action,
            ]
        )
        x = np.clip(raw, -self.clip_observation, self.clip_observation).astype(np.float32)
        (out,) = self._session.run(None, {self._input: x[None, :]})
        action = np.clip(out[0].astype(np.float64), -self.clip_action, self.clip_action)
        self._last_action = action
        targets: NDArray[np.float64] = self.default_pose + action * self.action_scale
        return targets
