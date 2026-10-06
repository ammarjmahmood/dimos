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


"""TwistSmoother: a twist stream in, the same stream ramped out.

Chases the latest input on its own clock, acceleration capped and low-passed, so a
follower's 10 Hz steps (or a full-speed reversal) leave as a ramp. Goes idle once the
input is quiet and the output has wound down to zero.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Twist import Twist


class TwistSmootherConfig(NativeModuleConfig):
    cwd: str | None = "rust"
    executable: str = "target/release/twist_smoother"
    build_command: str | None = "cargo build --release"
    stdin_config: bool = True

    rate_hz: float = 50.0
    # Smoothing strength: the low-pass time constant, about how long the output takes
    # to cover 63% of a step (s). Zero passes through.
    time_constant_s: float = 0.1
    # Per-axis caps on how fast the output may change; a full +v to -v reversal takes
    # 2v / accel. Zero for none.
    max_linear_accel: float = 1.0
    max_angular_accel: float = 2.0
    # Input silent this long counts as a stop request.
    timeout_s: float = 0.5


class TwistSmoother(NativeModule):
    """Low-pass and acceleration-limit a Twist stream."""

    config: TwistSmootherConfig

    cmd_vel_in: In[Twist]
    cmd_vel_out: Out[Twist]


if TYPE_CHECKING:
    TwistSmoother()
