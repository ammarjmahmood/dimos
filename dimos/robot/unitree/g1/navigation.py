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

"""Conservative indoor navigation settings for G1 running GR00T WBC.

The body uses G1's existing clearance dimensions and the policy's nominal pelvis
height. Motion settings are initial tuning, not a measured gait fit: in particular
we apply no Go2 slip correction or Go2 swept-leg envelope. This describes walking
with the arms tucked, not navigation while reaching or carrying a payload.
"""

from dimos.navigation.embodiment.base import Embodiment
from dimos.navigation.trajectory_follower.fancy.controller import ControllerConfig
from dimos.robot.unitree.g1.config import G1

G1_GROOT_NAVIGATION = Embodiment(
    length=G1.width_clearance,
    width=G1.width_clearance,
    comfort=0.4,
    precision=0.1,
    max_speed=0.3,
    min_speed=0.1,
    speed_clearance=0.4,
    max_yaw_rate=0.5,
    command_slew=(0.5, 0.5, 1.0),
    gait_band=(0.05, 0.3),
    walk_gain=1.0,
    walk_slip=0.0,
    walk_slip_ramp=0.05,
    strafe=1.3,
    reverse=1.8,
    yaw_w=0.15,
    steppable=0.1,
    height=G1.height_clearance + 0.2,
    base_height=0.74,
    control=ControllerConfig(
        lookahead=0.3,
        k_pos=1.5,
        k_yaw=1.5,
        fan_yaw_per_m=3.0,
        fan_yaw_done=0.2,
        speed_lookahead=1.0,
    ),
)
