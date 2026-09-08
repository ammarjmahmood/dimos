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

from dataclasses import replace
from uuid import uuid4

import numpy as np
import pytest

from dimos.hardware.whole_body.spec import POS_STOP, VEL_STOP, MotorCommand
from dimos.sim2.control.adapters import WholeBodyAdapter
from dimos.sim2.runtime import SimulationRuntime
from dimos.sim2.sensors.lidar.models.fibonacci import Fibonacci
from dimos.sim2.sensors.spec import Camera, Imu, Lidar, Mount
from dimos.sim2.spec import ControlInterface, Joint, RobotConfig, RobotInstance, WorldConfig
from experiments.robosuite_emulator.bridge import MotorEnvironment


@pytest.fixture
def config(tmp_path):
    scene = tmp_path / "scene.xml"
    scene.write_text('<mujoco><worldbody><geom type="plane" size="5 5 .1"/></worldbody></mujoco>')
    robot = tmp_path / "robot.xml"
    robot.write_text("""<mujoco><worldbody><body name="base"><freejoint/>
      <geom type="box" size=".1 .1 .1" mass="1"/>
      <body name="link" pos=".2 0 0"><joint name="hinge" axis="0 1 0"/>
      <geom type="box" size=".1 .03 .03" mass=".1"/></body>
      </body></worldbody><actuator><motor name="motor" joint="hinge" ctrlrange="-10 10"/></actuator>
      </mujoco>""")
    definition = RobotConfig(
        model=robot,
        root_body="base",
        control=ControlInterface.WHOLE_BODY,
        joints=(Joint("hinge", "hinge", "motor", home=0.1, kp=3, kd=0.2),),
        floating=True,
        sensors=(Imu("imu", Mount("base")),),
    )
    return WorldConfig(scene, {"robot": RobotInstance(definition, xyz=(0, 0, 1))}, timestep=0.005)


@pytest.fixture
def env(config):
    instance = MotorEnvironment(config, sense=False)
    try:
        yield instance
    finally:
        instance.close()


def test_physics_matches_native_sim2(config, env):
    sim_id = "spike-test-" + uuid4().hex
    native = SimulationRuntime(config, sim_id)
    adapter = WholeBodyAdapter(
        address=sim_id + "/robot",
        dof=1,
        definition=config.robots["robot"].config,
    )
    try:
        adapter.connect()
        commands = [MotorCommand(q=0.3, dq=0.05, kp=3, kd=0.2, tau=0.1)]
        adapter.write_motor_commands(commands)
        env.write_motor_commands(commands)
        for _ in range(5):
            env.advance()
            for _ in range(4):
                native.step()
        np.testing.assert_allclose(env.sim.data.qpos, native.data.qpos, atol=1e-10, rtol=0)
        np.testing.assert_allclose(env.sim.data.qvel, native.data.qvel, atol=1e-10, rtol=0)
        assert env.sim.data.time == pytest.approx(0.1)
    finally:
        adapter.disconnect()
        native.close()


def test_reset_restores_state_without_recompiling(env):
    env.reset()
    model = env.model
    initial = env.sim.data.qpos.copy()
    env.write_motor_commands([MotorCommand(q=0.5, kp=4, kd=0.3)])
    env.advance()
    assert not np.array_equal(initial, env.sim.data.qpos)
    env.reset()
    assert env.model is model
    np.testing.assert_array_equal(env.sim.data.qpos, initial)
    np.testing.assert_array_equal(env._commands, [[0.1, 0, 3, 0.2, 0]])


def test_motor_stop_sentinels_remove_gains(env):
    env.write_motor_commands([MotorCommand(q=POS_STOP, dq=VEL_STOP, kp=20, kd=4)])
    env.advance()
    np.testing.assert_array_equal(env.sim.data.ctrl, [0])


def test_sensor_extension_preserves_live_geometry_groups(config):
    definition = (
        config.robots["robot"]
        .config.with_sensor(
            Lidar(
                "scan",
                Mount("base", xyz=(0, 0, 0.2)),
                Fibonacci(elevation_min=-80, elevation_max=0),
            )
        )
        .with_sensor(Camera("wrist", Mount("link", xyz=(0.03, 0.02, 0.01))))
    )
    instance = replace(config.robots["robot"], config=definition)
    env = MotorEnvironment(replace(config, robots={"robot": instance}), sense=False)
    try:
        groups = env.model.geom_group.copy()
        lidar = next(s for s in definition.sensors if isinstance(s, Lidar))
        capture = env._lidar(lidar)
        points = capture({})
        assert points.shape[1] == 3 and len(points) > 0
        np.testing.assert_array_equal(env.model.geom_group, groups)
        camera = env.model.camera("robot/sensor/wrist")
        assert camera.bodyid == env.model.body("robot/link").id
        np.testing.assert_allclose(camera.pos, [0.03, 0.02, 0.01])
    finally:
        env.close()
