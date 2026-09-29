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

from types import SimpleNamespace

import numpy as np
import pytest

from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.control.tasks.m20_locomotion_task.m20_locomotion_task import (
    HOME,
    KD,
    KP,
    M20LocomotionConfig,
    M20LocomotionTask,
)
from dimos.hardware.whole_body.spec import IMUState, MotorCommand
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3


@pytest.fixture
def task(mocker, tmp_path):
    constructor = mocker.patch(
        "dimos.control.tasks.m20_locomotion_task.m20_locomotion_task.ort.InferenceSession"
    )
    session = constructor.return_value
    session.get_inputs.return_value = [SimpleNamespace(name="obs", shape=[1, 57])]
    session.get_outputs.return_value = [SimpleNamespace(name="actions", shape=[1, 16])]
    session.run.return_value = [np.ones((1, 16), dtype=np.float32)]
    instance = M20LocomotionTask("policy", M20LocomotionConfig(model_path=tmp_path / "policy.onnx"))
    instance.start()
    return instance, session


def _state(task, t_now=1.0):
    return CoordinatorState(
        joints=JointStateSnapshot(
            joint_positions=dict(zip(task.joint_names, np.asarray(HOME) + 0.1, strict=True)),
            joint_velocities=dict.fromkeys(task.joint_names, 2.0),
        ),
        imu={"m20": IMUState(gyroscope=(1, 2, 3))},
        t_now=t_now,
        dt=0.02,
    )


def test_sdk_observation_order_and_mixed_outputs(task):
    instance, session = task
    instance.on_twist_command(Twist(linear=Vector3(0.4, -0.2, 0), angular=Vector3(0, 0, 0.3)), 1.0)
    output = instance.compute(_state(instance))
    obs = session.run.call_args.args[1]["obs"][0]
    np.testing.assert_allclose(
        obs,
        np.r_[
            [0.25, 0.5, 0.75],
            [0, 0, -1],
            [0.4, -0.2, 0.3],
            np.full(12, 0.1),
            np.zeros(4),
            np.full(16, 0.1),
            np.zeros(16),
        ],
        atol=1e-7,
    )
    assert output.mode is None
    assert output.motor_commands[:3] == [
        MotorCommand(q=float(np.float32(HOME[i]) + np.float32(scale)), dq=0, kp=80, kd=2)
        for i, scale in enumerate((0.125, 0.25, 0.25))
    ]
    assert output.motor_commands[12:] == [MotorCommand(q=0, dq=5, kp=0, kd=0.6)] * 4
    instance.compute(_state(instance, t_now=2.0))
    obs = session.run.call_args.args[1]["obs"][0]
    np.testing.assert_array_equal(obs[6:9], 0)
    np.testing.assert_array_equal(obs[41:], 1)


def test_stop_dispatches_damping_and_reset_clears_actor_history(task):
    instance, session = task
    instance.compute(_state(instance))
    instance.stop()
    assert instance.is_active()
    assert instance.compute(_state(instance)).motor_commands == [
        MotorCommand(q=0, dq=0, kp=0, kd=kd) for kd in KD
    ]
    assert not instance.is_active()
    assert instance.compute(_state(instance)) is None
    instance.reset_runtime_state(reactivate=True)
    instance.compute(_state(instance))
    np.testing.assert_array_equal(session.run.call_args.args[1]["obs"][0, 41:], 0)


def test_complete_command_retains_all_policy_gains(task):
    instance, _ = task
    output = instance.compute(_state(instance))
    assert [(c.kp, c.kd) for c in output.motor_commands] == list(zip(KP, KD, strict=True))


def test_actor_failure_dispatches_damping_on_next_tick(task):
    instance, session = task
    session.run.side_effect = RuntimeError("inference failed")
    with pytest.raises(RuntimeError, match="inference failed"):
        instance.compute(_state(instance))
    assert instance.compute(_state(instance)).motor_commands == [
        MotorCommand(q=0, dq=0, kp=0, kd=kd) for kd in KD
    ]
    assert not instance.is_active()
