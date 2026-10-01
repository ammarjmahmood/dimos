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

import numpy as np
import pytest

from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.manipulation_spec import PlanningGroupState
from dimos.manipulation.sdk import Arm
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.simulation.behavior.connection import BehaviorConnection
from dimos.simulation.behavior.demo_radio import press_toggle, radio_blueprint, set_gripper_and_wait
from dimos.simulation.behavior.probe import BehaviorProbe
from dimos.simulation.behavior.r1pro_bridge import BehaviorCoordinator, BehaviorR1ProBridge
from dimos.simulation.behavior.r1pro_model import MODEL_JOINTS
from dimos.simulation.behavior.types import TaskSelection


@pytest.fixture
def probe():
    module = BehaviorProbe()
    try:
        yield module
    finally:
        module.stop()


def test_policy_observation_requires_sensors_and_excludes_task_truth(probe):
    with pytest.raises(RuntimeError, match="Observation streams unavailable"):
        probe.observation()
    image = Image(data=np.ones((2, 2, 3), dtype=np.uint8), ts=123, frame_id="camera")
    messages = {
        "color_image": image,
        "depth_image": Image(data=np.ones((2, 2), dtype=np.float32)),
        "camera_info": CameraInfo(),
        "joint_state": JointState(),
        "tf": TFMessage(),
    }
    for name, value in messages.items():
        probe._record(name, value)
    probe._record("task_truth", {"radio_state": "oracle"})
    actual = probe.observation()
    assert actual.keys() == messages.keys()
    assert actual["color_image"].ts == 123
    assert actual["color_image"].frame_id == "camera"
    actual["color_image"].data[:] = 0
    np.testing.assert_array_equal(image.data, np.ones((2, 2, 3), dtype=np.uint8))


def test_gripper_acceptance_waits_for_measured_feedback(mocker):
    arm = mocker.Mock(spec=Arm)
    arm.state.side_effect = [PlanningGroupState(None, None, value) for value in (None, 0.3, 0.02)]
    sleep = mocker.patch("dimos.simulation.behavior.demo_radio.time.sleep")
    assert set_gripper_and_wait(arm, 0.0) == 0.02
    arm.set_gripper_position.assert_called_once_with(0.0)
    assert sleep.call_count == 2


def test_missing_gripper_feedback_never_counts_as_closed(mocker):
    arm = mocker.Mock(spec=Arm)
    arm.state.return_value = PlanningGroupState(None, None, None)
    mocker.patch("dimos.simulation.behavior.demo_radio.time.monotonic", side_effect=[0, 11])
    with pytest.raises(TimeoutError, match="measured target"):
        set_gripper_and_wait(arm, 0.0)


def test_radio_composition_binds_selected_gripper_without_base_trajectory():
    bp = radio_blueprint(TaskSelection(activity="turning_on_radio"), arm="right_arm")
    atoms = {a.module: a for a in bp.blueprints}
    hardware = atoms[BehaviorCoordinator].kwargs["hardware"][0]
    assert hardware.joints == [f"r1pro/{n}" for n in MODEL_JOINTS]
    task = atoms[BehaviorCoordinator].kwargs["tasks"][1]
    assert task.name == "r1pro_gripper"
    assert task.joint_names == ["r1pro/right_gripper_finger_joint1"]
    model = atoms[ManipulationModule].kwargs["model"]
    assert atoms[ManipulationModule].kwargs["planner"].backend == "roboplan"
    assert [g.name for g in model.planning_groups] == ["right_arm", "torso"]
    assert model.gripper_hardware_id == "r1pro"
    assert atoms[BehaviorR1ProBridge].kwargs["command_joints"] == MODEL_JOINTS
    assert not any(
        "base_" in n
        for n in atoms[ManipulationModule].kwargs["trajectory_tasks"]["joint_trajectory"]
    )
    assert atoms[BehaviorConnection].kwargs["development_task_spawn"] is False


def test_custom_task_spawn_is_explicitly_marked_development():
    bp = radio_blueprint(TaskSelection(activity="turning_on_radio"), spawn_position=(1, 2, 0.1))
    atom = next(a for a in bp.blueprints if a.module is BehaviorConnection)
    assert atom.kwargs["development_task_spawn"] is True
    assert atom.kwargs["spawn_position"] == (1, 2, 0.1)
    assert atom.kwargs["task"].activity == "turning_on_radio"


@pytest.mark.parametrize("auxiliary_groups", [(), ("torso",)])
def test_press_uses_pose_sdk_then_holds_simulation_steps_and_retracts(mocker, auxiliary_groups):
    arm = mocker.Mock(spec=Arm)
    arm.rpc = mocker.Mock()
    step = mocker.Mock(side_effect=[10, 10, 18])
    sleep = mocker.patch("dimos.simulation.behavior.demo_radio.time.sleep")
    result = press_toggle(arm, [1, 2, 3], [2, 0, 0], step, auxiliary_groups=auxiliary_groups)
    arm.move_pose.assert_called_once()
    assert arm.move_pose.call_args.args[0] == [0.97, 2.0, 3.0]
    assert [call.args for call in arm.move_linear.call_args_list] == [
        (0.03, 0.0, 0.0),
        (-0.03, -0.0, -0.0),
    ]
    assert all(call.kwargs["check_collision"] for call in arm.move_linear.call_args_list)
    assert all(
        call.kwargs["auxiliary_groups"] == auxiliary_groups
        for call in arm.move_linear.call_args_list
    )
    if auxiliary_groups:
        assert arm.move_pose.call_args.kwargs["auxiliary_groups"] == auxiliary_groups
    arm.move_joints.assert_not_called()
    assert result == {"development_only": True, "motion_completed": True, "hold_steps": 8}
    sleep.assert_called_once()
    arm.rpc.cancel.assert_not_called()


def test_motion_timeout_cancels_and_does_not_retract_or_claim_completion(mocker):
    arm = mocker.Mock(spec=Arm)
    arm.rpc = mocker.Mock()
    arm.move_linear.side_effect = TimeoutError("Execution is still active")
    arm.rpc.cancel.return_value = "UNCERTAIN"
    with pytest.raises(RuntimeError, match="UNCERTAIN"):
        press_toggle(arm, [1, 2, 3], [1, 0, 0], mocker.Mock())
    arm.rpc.cancel.assert_called_once_with()
    assert arm.move_linear.call_count == 1


@pytest.mark.parametrize(
    "target,direction",
    [([1, 2], [1, 0, 0]), ([1, 2, float("nan")], [1, 0, 0]), ([1, 2, 3], [0, 0, 0])],
)
def test_invalid_contact_never_dispatches_motion(target, direction, mocker):
    arm = mocker.Mock(spec=Arm)
    with pytest.raises(ValueError):
        press_toggle(arm, target, direction, mocker.Mock())
    arm.move_pose.assert_not_called()
    arm.move_linear.assert_not_called()
