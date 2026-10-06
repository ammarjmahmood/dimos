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

import collections
from dataclasses import dataclass
from pathlib import Path
import struct
from typing import Protocol

import numpy as np
from numpy.typing import NDArray
import onnxruntime as ort

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.utils.data import get_data


@dataclass(frozen=True)
class Proprioception:
    """What the policy senses, with joints in the policy's own order."""

    angular_velocity: NDArray[np.float64]
    gravity: NDArray[np.float64]
    joint_position: NDArray[np.float64]
    joint_velocity: NDArray[np.float64]


class Go2Policy(Protocol):
    """A joint-position policy driving the Go2's PD motors."""

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
        self._last_action = np.zeros(len(self.joint_names))

    @classmethod
    def load(cls) -> OnnxGo2Policy:
        return cls(get_data("go2_sim") / "go2_policy" / "policy.onnx")

    def reset(self) -> None:
        self._last_action = np.zeros(len(self.joint_names))

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


Layer = tuple[NDArray[np.float64], NDArray[np.float64]]


def _elu(x: NDArray[np.float64]) -> NDArray[np.float64]:
    out: NDArray[np.float64] = np.where(x > 0, x, np.expm1(np.minimum(x, 0)))
    return out


def _mlp(layers: list[Layer], x: NDArray[np.float64]) -> NDArray[np.float64]:
    """ELU on every hidden layer, linear on the last."""
    for w, b in layers[:-1]:
        x = _elu(x @ w + b)
    w, b = layers[-1]
    out: NDArray[np.float64] = x @ w + b
    return out


class _Reader:
    def __init__(self, blob: bytes) -> None:
        self.blob = blob
        self.pos = 0

    def u32(self) -> int:
        value: int = struct.unpack_from("<I", self.blob, self.pos)[0]
        self.pos += 4
        return value

    def f32(self, n: int) -> NDArray[np.float64]:
        values = np.frombuffer(self.blob, "<f4", n, self.pos).astype(np.float64)
        self.pos += 4 * n
        return values

    def block(self) -> list[Layer]:
        layers = []
        for _ in range(self.u32()):
            nin, nout = self.u32(), self.u32()
            layers.append((self.f32(nin * nout).reshape(nin, nout), self.f32(nout)))
        return layers


@dataclass(frozen=True)
class _Band:
    kind: int
    encoder: list[Layer]
    actor: list[Layer]


BAND_WALK = 0
BAND_FAST = 1
BAND_SPRINT = 2
BAND_ROTATE = 3
FREE_OBS = 45


class FreePolicy:
    """A go2web "FREE" v1 blob: speed-banded HIMLoco experts with their normalization and gains."""

    joint_names = tuple(
        f"{leg}_{part}_joint"
        for leg in ("FL", "FR", "RL", "RR")
        for part in ("hip", "thigh", "calf")
    )

    def __init__(self, blob: bytes) -> None:
        reader = _Reader(blob)
        if blob[:4] != b"FREE":
            raise ValueError("not a FREE policy blob")
        reader.pos = 4
        if (version := reader.u32()) != 1:
            raise ValueError(f"unsupported FREE version {version}")
        self.hist, obs_per_frame, act_dim = reader.u32(), reader.u32(), reader.u32()
        self.enc_vel, self.enc_lat = reader.u32(), reader.u32()
        if obs_per_frame != FREE_OBS or act_dim != len(self.joint_names):
            raise ValueError(
                f"expected a {FREE_OBS} observation, 12 action net, got {obs_per_frame}, {act_dim}"
            )
        self.clip_obs, self.clip_act = (float(v) for v in reader.f32(2))
        self.ob_mean, self.ob_scale = reader.f32(FREE_OBS), reader.f32(FREE_OBS)
        self.act_mean, self.act_scale = reader.f32(act_dim), reader.f32(act_dim)
        self.default_pose = reader.f32(act_dim)
        self.kp, self.kd = reader.f32(act_dim), reader.f32(act_dim)
        reader.f32(6)
        self.bands = {}
        for _ in range(reader.u32()):
            kind = reader.u32()
            self.bands[kind] = _Band(kind, reader.block(), reader.block())
        if not self.bands:
            raise ValueError("FREE blob has no band models")
        self._history: collections.deque[NDArray[np.float64]] = collections.deque(maxlen=self.hist)
        self._last_action = np.zeros(act_dim)

    @classmethod
    def load(cls, path: str | Path) -> FreePolicy:
        """Read a blob from a path outside the repository."""
        resolved = Path(path).expanduser().resolve()
        if resolved.is_relative_to(DIMOS_PROJECT_ROOT.resolve()):
            raise ValueError(f"policy blobs are loaded from outside the repository: {resolved}")
        return cls(resolved.read_bytes())

    def reset(self) -> None:
        self._history.clear()
        self._last_action = np.zeros(len(self.joint_names))

    def band_for(self, command: NDArray[np.float64]) -> _Band:
        """The speed-band expert the robot's state machine would pick, or the nearest present."""
        speed = max(abs(command[0]), abs(command[1]))
        if speed < 0.05 and abs(command[2]) > 0.05:
            want = BAND_ROTATE
        elif speed < 1.0:
            want = BAND_WALK
        elif speed < 5.0:
            want = BAND_FAST
        else:
            want = BAND_SPRINT
        for kind in (want, max(want - 1, 0), BAND_WALK):
            if kind in self.bands:
                return self.bands[kind]
        return next(iter(self.bands.values()))

    def act(self, obs: Proprioception, command: NDArray[np.float64]) -> NDArray[np.float64]:
        raw = np.concatenate(
            [
                command,
                obs.angular_velocity,
                obs.gravity,
                obs.joint_position,
                obs.joint_velocity,
                self._last_action,
            ]
        )
        frame = np.clip((raw - self.ob_mean) * self.ob_scale, -self.clip_obs, self.clip_obs)
        if not self._history:
            self._history.extend([frame] * self.hist)
        else:
            self._history.append(frame)
        p_obs = np.concatenate(list(self._history)[::-1])
        band = self.band_for(command)
        encoded = _mlp(band.encoder, p_obs)
        velocity, latent = encoded[: self.enc_vel], encoded[self.enc_vel :]
        latent = latent / max(float(np.linalg.norm(latent)), 1e-12)
        action = _mlp(band.actor, np.concatenate([frame, velocity, latent]))
        self._last_action = np.clip(action, -self.clip_act, self.clip_act)
        targets: NDArray[np.float64] = self._last_action * self.act_scale + self.act_mean
        return targets


def load_policy(path: str) -> Go2Policy:
    """The bundled open policy when path is empty, else an ONNX or FREE file by suffix."""
    if not path:
        return OnnxGo2Policy.load()
    suffix = Path(path).suffix
    if suffix == ".onnx":
        return OnnxGo2Policy(Path(path).expanduser())
    if suffix == ".bin":
        return FreePolicy.load(path)
    raise ValueError(f"unknown policy format {suffix!r}, expected .onnx or .bin")
