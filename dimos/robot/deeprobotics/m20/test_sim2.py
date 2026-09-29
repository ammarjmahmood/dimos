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

from contextlib import ExitStack
import threading
import time
from uuid import uuid4

import numpy as np
import pytest

from dimos.control.components import HardwareComponent, HardwareType
from dimos.control.hardware_interface import ConnectedWholeBody
from dimos.control.tasks.m20_locomotion_task.m20_locomotion_task import (
    KD,
    KP,
    M20LocomotionConfig,
    M20LocomotionTask,
)
from dimos.control.tick_loop import TickLoop
from dimos.hardware.whole_body.spec import WholeBodyConfig
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.deeprobotics.m20.sim2 import ASSETS, M20, POLICY_PATH
from dimos.sim2.control.adapters import WholeBodyAdapter
from dimos.sim2.runtime import SimulationRuntime
from dimos.sim2.spec import RobotInstance, WorldConfig

pytestmark = pytest.mark.mujoco


@pytest.fixture
def simulation():
    with ExitStack() as cleanup:
        world = SimulationRuntime(
            WorldConfig(
                ASSETS / "stairs.xml",
                {"m20": RobotInstance(M20, xyz=(0, 0, 0.6))},
                timestep=0.001,
            ),
            uuid4().hex,
        )
        cleanup.callback(world.close)
        adapter = WholeBodyAdapter(
            address=f"{world.snapshot_descriptor.sim_id}/m20", dof=16, definition=M20
        )
        cleanup.callback(adapter.disconnect)
        adapter.connect()
        hardware = ConnectedWholeBody(
            adapter,
            HardwareComponent(
                hardware_id="m20",
                hardware_type=HardwareType.WHOLE_BODY,
                joints=[j.name for j in M20.joints],
                wb_config=WholeBodyConfig(kp=KP, kd=KD),
            ),
        )
        policy = M20LocomotionTask("m20_locomotion", M20LocomotionConfig(model_path=POLICY_PATH))
        policy.start()
        loop = TickLoop(
            tick_rate=50,
            hardware={"m20": hardware},
            hardware_lock=threading.Lock(),
            tasks={policy.name: policy},
            task_lock=threading.Lock(),
            joint_to_hardware=dict.fromkeys(hardware.joint_names, "m20"),
        )
        yield world, policy, loop


def test_public_actor_balances_and_drives_through_coordinator_and_shm(simulation, record_property):
    world, policy, loop = simulation
    root = world.robots["m20"].root
    for _ in range(100):
        loop._tick()
        for _ in range(20):
            world.step()
    start = world.data.xpos[root].copy()
    assert start[2] > 0.3
    for _ in range(150):
        policy.on_twist_command(Twist(linear=Vector3(0.3, 0, 0)), time.perf_counter())
        loop._tick()
        for _ in range(20):
            world.step()
    end = world.data.xpos[root].copy()
    record_property("start_xyz", start.tolist())
    record_property("end_xyz", end.tolist())
    record_property("sim_seconds", float(world.data.time))
    assert np.isfinite(world.data.qpos).all()
    assert end[0] - start[0] > 0.25
    assert end[2] > 0.3
    assert world.data.xmat[root].reshape(3, 3)[2, 2] > 0.8
    assert loop.tick_count == 250
