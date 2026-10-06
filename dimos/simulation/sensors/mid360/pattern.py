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

"""Livox Mid-360 scan pattern.

Two rotors, a fast azimuth one near 181.08 Hz and a slow elevation one at 9.9 Hz.
Points go out at 200 kHz round-robin over four channels. Each channel's direction is
a 2D Fourier series in the two rotor phases.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
from numpy.typing import NDArray

POINT_RATE = 200_000.0
CHANNELS = 4


class Mid360Pattern:
    def __init__(
        self, f1: float, f2: float, order_fast: int, order_slow: int, coefs: NDArray[np.float64]
    ) -> None:
        self.f1 = f1
        self.f2 = f2
        self.order_fast = order_fast
        self.order_slow = order_slow
        self.coefs = coefs
        pairs = [
            (m, q)
            for m in range(order_fast + 1)
            for q in range(-order_slow, order_slow + 1)
            if not (m == 0 and q < 0)
        ]
        cos_cols, sin_cols, sin_pairs = [], [], []
        col = 0
        for p, pair in enumerate(pairs):
            cos_cols.append(col)
            col += 1
            if pair != (0, 0):
                sin_cols.append(col)
                sin_pairs.append(p)
                col += 1
        if col != coefs.shape[1]:
            raise ValueError(f"pattern has {coefs.shape[1]} coefficients, orders imply {col}")
        self._m_idx = np.array([m for m, _ in pairs])
        self._q_idx = np.array([q + order_slow for _, q in pairs])
        self._cos_cols = np.array(cos_cols)
        self._sin_cols = np.array(sin_cols)
        self._sin_pairs = np.array(sin_pairs)

    @classmethod
    def load(cls, path: str | Path) -> Mid360Pattern:
        z = np.load(path)
        return cls(
            f1=float(z["f1"]),
            f2=float(z["f2"]),
            order_fast=int(z["m1"]),
            order_slow=int(z["m2"]),
            coefs=np.asarray(z["coefs"], dtype=np.float64),
        )

    def directions(self, k0: int, n: int) -> NDArray[np.float64]:
        """Unit sensor-frame directions of global point indices k0 .. k0 + n."""
        k = np.arange(k0, k0 + n)
        t = (k // CHANNELS) / (POINT_RATE / CHANNELS)
        fast = np.exp(2j * np.pi * self.f1 * t)
        slow = np.exp(2j * np.pi * self.f2 * t)
        fast_pow = np.cumprod(np.column_stack([np.ones(n), *[fast] * self.order_fast]), axis=1)
        slow_pos = np.cumprod(np.column_stack([np.ones(n), *[slow] * self.order_slow]), axis=1)
        slow_pow = np.concatenate([np.conj(slow_pos[:, :0:-1]), slow_pos], axis=1)
        harmonic = fast_pow[:, self._m_idx] * slow_pow[:, self._q_idx]
        basis = np.empty((n, self.coefs.shape[1]))
        basis[:, self._cos_cols] = harmonic.real
        basis[:, self._sin_cols] = harmonic.imag[:, self._sin_pairs]
        out = np.empty((n, 3))
        for c in range(CHANNELS):
            first = (c - k0) % CHANNELS
            out[first::CHANNELS] = basis[first::CHANNELS] @ self.coefs[c]
        out /= np.linalg.norm(out, axis=1, keepdims=True)
        return out
