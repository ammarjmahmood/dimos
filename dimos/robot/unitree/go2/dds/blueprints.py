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


"""GO2DDS blueprints."""

from dimos.protocol.service.zenohservice import ZenohConfig
from dimos.robot.unitree.go2.dds.module import GO2DDS

# GO2DDS doubles as a zenoh router
go2_dds = GO2DDS.blueprint(
    iface="enP8p1s0", session=ZenohConfig(mode="router", listen=["tcp/0.0.0.0:7447"], connect=[])
).global_config(transport="zenoh", robot_model="unitree_go2")
