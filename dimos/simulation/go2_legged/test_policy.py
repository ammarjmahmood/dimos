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

from __future__ import annotations

import itertools
from pathlib import Path
import struct

import numpy as np
from numpy.typing import ArrayLike
import pytest

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.simulation.go2_legged.policy import (
    BAND_ROTATE,
    BAND_WALK,
    FREE_OBS,
    FreePolicy,
    Proprioception,
    load_policy,
)

ACT = 12


def _f32(values: ArrayLike) -> bytes:
    return np.asarray(values, "<f4").tobytes()


def _layer(rng: np.random.Generator, nin: int, nout: int) -> bytes:
    weights = rng.standard_normal(nin * nout).astype("<f4") * 0.1
    bias = rng.standard_normal(nout).astype("<f4") * 0.1
    return struct.pack("<II", nin, nout) + weights.tobytes() + bias.tobytes()


def _block(rng: np.random.Generator, sizes: list[int]) -> bytes:
    layers = [_layer(rng, a, b) for a, b in itertools.pairwise(sizes)]
    return struct.pack("<I", len(layers)) + b"".join(layers)


def _blob(
    hist: int = 2, enc_vel: int = 3, enc_lat: int = 4, kinds: tuple[int, ...] = (0, 3)
) -> bytes:
    rng = np.random.default_rng(0)
    head = b"FREE" + struct.pack("<IIIIII", 1, hist, FREE_OBS, ACT, enc_vel, enc_lat)
    head += _f32([100.0, 100.0])
    head += _f32(np.zeros(FREE_OBS)) + _f32(np.ones(FREE_OBS))
    head += _f32([0.0, 0.9, -1.8] * 4) + _f32([0.25] * ACT)
    head += _f32([0.0, 0.9, -1.8] * 4) + _f32([40.0] * ACT) + _f32([1.0] * ACT)
    head += _f32(np.zeros(6))
    bands = b""
    for kind in kinds:
        encoder = _block(rng, [hist * FREE_OBS, 8, enc_vel + enc_lat])
        actor = _block(rng, [FREE_OBS + enc_vel + enc_lat, 8, ACT])
        bands += struct.pack("<I", kind) + encoder + actor
    return head + struct.pack("<I", len(kinds)) + bands


def _obs() -> Proprioception:
    return Proprioception(
        np.zeros(3), np.array([0.0, 0.0, -1.0]), np.array([0.0, 0.9, -1.8] * 4), np.zeros(ACT)
    )


def test_reads_the_blob_layout() -> None:
    policy = FreePolicy(_blob())
    assert policy.hist == 2
    assert set(policy.bands) == {BAND_WALK, BAND_ROTATE}
    assert policy.default_pose == pytest.approx([0.0, 0.9, -1.8] * 4)
    assert policy.kp[0] == 40.0 and policy.kd[0] == 1.0
    assert policy.joint_names[0] == "FL_hip_joint" and policy.joint_names[3] == "FR_hip_joint"


def test_acts_deterministically_and_resumes_from_a_clean_history() -> None:
    a, b = FreePolicy(_blob()), FreePolicy(_blob())
    command = np.array([0.5, 0.0, 0.0])
    first = a.act(_obs(), command)
    assert first.shape == (ACT,)
    assert np.array_equal(first, b.act(_obs(), command))
    a.act(_obs(), command)
    a.reset()
    assert np.array_equal(a.act(_obs(), command), first)


def test_picks_the_rotate_band_for_a_pure_turn() -> None:
    policy = FreePolicy(_blob())
    assert policy.band_for(np.array([0.0, 0.0, 0.5])).kind == BAND_ROTATE
    assert policy.band_for(np.array([0.5, 0.0, 0.5])).kind == BAND_WALK
    assert policy.band_for(np.array([2.0, 0.0, 0.0])).kind == BAND_WALK


def test_rejects_other_blobs_and_weights_inside_the_repository(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="FREE"):
        FreePolicy(b"BLND" + bytes(64))
    inside = DIMOS_PROJECT_ROOT / "data" / "freewalk_mcf.bin"
    with pytest.raises(ValueError, match="outside the repository"):
        FreePolicy.load(inside)
    outside = tmp_path / "freewalk_mcf.bin"
    outside.write_bytes(_blob())
    assert isinstance(load_policy(str(outside)), FreePolicy)
    with pytest.raises(ValueError, match="format"):
        load_policy(str(tmp_path / "policy.pt"))
