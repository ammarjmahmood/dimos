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

from uuid import uuid4

import numpy as np
import pytest
from robosuite.environments.manipulation.lift import Lift

from dimos.robot.deeprobotics.m20.sim2 import M20
from dimos.robot.manipulators.xarm.sim2 import XARM7
from dimos.robot.unitree.g1.sim2 import G1_GROOT
from dimos.sim2.robot import motor_controller_config
from dimos.sim2.runtime import SimulationRuntime
from dimos.sim2.spec import RobotInstance, WorldConfig

pytestmark = pytest.mark.mujoco


@pytest.fixture(params=[("g1", G1_GROOT), ("m20", M20), ("xarm", XARM7)])
def device_world(request, tmp_path):
    name, definition = request.param
    scene = tmp_path / "scene.xml"
    scene.write_text('<mujoco><worldbody><geom type="plane" size="3 3 .1"/></worldbody></mujoco>')
    world = SimulationRuntime(
        WorldConfig(scene, {name: RobotInstance(definition, xyz=(0.1, 0.2, 1))}), uuid4().hex
    )
    try:
        yield name, world
    finally:
        world.close()


def test_upstream_robot_observations_and_reset_share_runtime_state(device_world):
    name, world = device_world
    env = world.environment
    robot = env.robot_by_id[name]
    observations = env._get_observations(force_update=True)
    np.testing.assert_array_equal(observations[f"{name}/joint_pos"], robot._joint_positions)
    assert set(robot.arms) == {"g1": {"right", "left"}, "m20": set(), "xarm": {"right"}}[name]
    for arm in robot.arms:
        prefix = f"{name}/" + (f"{arm}_" if len(robot.arms) > 1 else "")
        np.testing.assert_array_equal(
            observations[prefix + "eef_pos"], world.data.site_xpos[robot.eef_site_id[arm]]
        )
    original = world.data.qpos.copy()
    for _ in range(4):
        world.step()
    world.reset()
    assert env.timestep == 0
    np.testing.assert_array_equal(world.data.qpos, original)
    np.testing.assert_array_equal(
        env._get_observations(force_update=True)[f"{name}/joint_pos"], robot._joint_positions
    )
    assert world.model.nu == len(robot.definition.joints)


@pytest.fixture
def xarm_lift():
    env = Lift(
        robots="XArm7Model",
        controller_configs=motor_controller_config(XARM7),
        base_types="NullBase",
        has_renderer=False,
        has_offscreen_renderer=False,
        use_camera_obs=False,
        renderer="mujoco",
        hard_reset=False,
        lite_physics=False,
        seed=42,
    )
    try:
        yield env
    finally:
        env.close()


def test_same_xarm_uses_upstream_lift_sampling_gripper_and_success(xarm_lift):
    env = xarm_lift
    model = env.sim.model._model
    positions = [env.reset()["cube_pos"].copy() for _ in range(3)]
    assert len({tuple(position) for position in positions}) == 3
    assert env.sim.model._model is model
    assert not env._check_success()
    gripper = env.robots[0].gripper["right"]
    assert gripper.important_geoms["left_fingerpad"] == [
        "gripper0_right_left_finger_pad_1",
        "gripper0_right_left_finger_pad_2",
    ]
    assert not env._check_grasp(gripper, env.cube.contact_geoms)

    command = env.robots[0].composite_controller.home.copy()
    command[0, 0] = 0.2
    for _ in range(5):
        observations, _, _, _ = env.step(command.ravel())
    assert observations["0/joint_pos"][0] > 0.05

    # Validate the upstream oracle, not a claim that the policy solved Lift.
    pose = env.sim.data.get_joint_qpos(env.cube.joints[0]).copy()
    pose[2] = env.table_offset[2] + 0.2
    env.sim.data.set_joint_qpos(env.cube.joints[0], pose)
    env.sim.forward()
    assert env._check_success()
    assert env.reward() == 1.0


def test_upstream_grasp_helper_recognizes_both_xarm_finger_contacts(xarm_lift):
    env = xarm_lift
    env.reset()
    hand = env.robots[0].gripper["right"]
    assert not env._check_grasp(hand, env.cube.contact_geoms)
    # Place a test contact state; no claim of a policy-generated grasp.
    for joint in hand.joints:
        env.sim.data.set_joint_qpos(joint, 0.65)
    env.sim.forward()
    pads = hand.important_geoms["left_fingerpad"] + hand.important_geoms["right_fingerpad"]
    center = np.mean(
        [env.sim.data.geom_xpos[env.sim.model.geom_name2id(name)] for name in pads], axis=0
    )
    quat = env.sim.data.body_xquat[env.sim.model.body_name2id(hand.root_body)]
    env.sim.data.set_joint_qpos(env.cube.joints[0], np.r_[center, quat])
    env.sim.forward()
    assert env._check_grasp(hand, env.cube.contact_geoms)
