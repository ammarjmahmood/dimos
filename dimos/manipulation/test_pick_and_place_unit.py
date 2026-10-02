# Copyright 2025-2026 Dimensional Inc.
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

from collections.abc import Iterator
from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

from dimos_lcm.vision_msgs import BoundingBox3D, ObjectHypothesis, ObjectHypothesisWithPose
import numpy as np
import pytest

from dimos.manipulation.grasp_verification import GripperSettle
from dimos.manipulation.manipulation_skills import ManipulationSkills
from dimos.manipulation.pick_and_place_module import (
    VOXEL_MAP_EXCLUSION_ID,
    HeldObject,
    PickAndPlaceModule,
    ScannedObject,
)
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Header import Header
from dimos.msgs.vision_msgs.Detection3D import Detection3D
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray

CUP_AT = Vector3(0.1, 0.0, 0.2)
GROUP = SimpleNamespace(id="arm/tool", has_gripper=True, tip_frame="tool")
TOP_DOWN = Quaternion.from_euler(Vector3(-3.141592653589793, 0.0, 0.0))


def _detections(*found: tuple[str, str, Vector3]) -> Detection3DArray:
    """A scan result listing (object_id, label, centre) triples."""
    detections = [
        Detection3D(
            id=object_id,
            header=Header(1.0, "world"),
            results_length=1,
            results=[
                ObjectHypothesisWithPose(hypothesis=ObjectHypothesis(class_id=name, score=1.0))
            ],
            bbox=BoundingBox3D(center=Pose(center, Quaternion()), size=Vector3(0.1, 0.1, 0.1)),
        )
        for object_id, name, center in found
    ]
    return Detection3DArray(
        detections_length=len(detections), header=Header(1.0, "world"), detections=detections
    )


def _group_state(gripper_position: float = 0.5) -> SimpleNamespace:
    return SimpleNamespace(
        groups={
            "arm/tool": SimpleNamespace(
                joints=JointState(name=["j1"], position=[0.0]),
                gripper_position=gripper_position,
                end_effector_pose=PoseStamped(
                    frame_id="world",
                    position=Vector3(0.4, 0.0, 0.2),
                    orientation=Quaternion.from_euler(Vector3(0, 0, 0.7)),
                ),
            )
        }
    )


def _holding_cup() -> HeldObject:
    return HeldObject("cup-1", "cup", PoseStamped(frame_id="world", orientation=TOP_DOWN), CUP_AT)


@pytest.fixture
def module() -> Iterator[PickAndPlaceModule]:
    instance = PickAndPlaceModule(planning_frame="world", robot_ready_poll_interval=0.01)
    instance._scene = MagicMock()
    instance._grasp_generator = MagicMock()
    instance._manipulation = MagicMock()
    instance._manipulation.list_planning_groups.return_value = [GROUP]
    instance._manipulation.get_state.return_value = _group_state()
    instance._manipulation.plan_to_poses.return_value = SimpleNamespace(succeeded=True, message="")
    instance._manipulation.execute.return_value = SimpleNamespace(succeeded=True, message="")
    instance._manipulation.move_linear.return_value = SimpleNamespace(
        plan=SimpleNamespace(succeeded=True, message=""),
        execution=SimpleNamespace(succeeded=True, message=""),
    )
    instance._manipulation.set_gripper_position.return_value = SimpleNamespace(
        succeeded=True, message=""
    )
    instance._objects = {"cup-1": ScannedObject("cup-1", "cup", CUP_AT)}
    # The cup's points: a 4 cm cube standing on the table at z=0.18.
    instance._scene.get_object_pointcloud_by_object_id.return_value = PointCloud2.from_numpy(
        np.asarray([[0.08, -0.02, 0.18], [0.12, 0.02, 0.22]], dtype=np.float32),
        frame_id="world",
        timestamp=1.0,
    )
    # After a lift or a release the camera sees nothing where the cup was.
    instance._scene.scan_scene.return_value = _detections()
    instance._grasp_generator.propose_grasps.return_value = GraspCandidateArray(
        Header(1.0, "world"), [_candidate(0.1)]
    )
    yield instance
    instance.stop()


@pytest.fixture(autouse=True)
def settled_gripper(monkeypatch: pytest.MonkeyPatch) -> None:
    def settle(read: Any, target: float, config: Any, **_: Any) -> GripperSettle:
        position = 0.5 if target == config.closed_position else target
        return GripperSettle(True, position, True, 0.1)

    monkeypatch.setattr("dimos.manipulation.pick_and_place_module.await_gripper_settle", settle)


def _candidate(x: float, score: float = 1.0) -> GraspCandidate:
    return GraspCandidate(Pose(Vector3(x, 0.0, 0.2), TOP_DOWN), score)


def test_scan_objects_records_ids_and_positions(module: PickAndPlaceModule) -> None:
    scene: Any = module._scene
    scene.scan_scene.return_value = _detections(("cup-1", "cup", Vector3(0.3, 0.1, 0.2)))

    result = module.scan_objects([" cup "])

    assert result.is_success()
    assert result.metadata["objects"] == [{"object_id": "cup-1", "name": "cup"}]
    assert module.get_object("cup-1") == {"object_id": "cup-1", "name": "cup"}
    assert module._objects["cup-1"].position == Vector3(0.3, 0.1, 0.2)
    scene.scan_scene.assert_called_once_with(text=["cup"])


def test_pick_object_uses_first_provider_candidate(
    module: PickAndPlaceModule,
) -> None:
    grasp_generator: Any = module._grasp_generator
    first, second = _candidate(0.1, 0.1), _candidate(0.2, 0.9)
    grasp_generator.propose_grasps.return_value = GraspCandidateArray(
        Header(1.0, "world"), [first, second]
    )

    result = module.pick_object("cup-1")

    assert result.is_success()
    assert module.get_grasp_candidates().candidates == [first, second]
    assert module._held is not None
    assert module._held.grasp.position.x == pytest.approx(0.1)
    assert result.metadata["rank"] == 0


def test_pick_object_rejects_non_planning_frame(module: PickAndPlaceModule) -> None:
    grasp_generator: Any = module._grasp_generator
    grasp_generator.propose_grasps.return_value = GraspCandidateArray(
        Header(1.0, "camera"), [_candidate(0.1)]
    )

    result = module.pick_object("cup-1")

    assert not result.is_success()
    assert result.error_code == "GRASP_FRAME_MISMATCH"


def test_pick_falls_through_to_the_next_reachable_candidate(
    module: PickAndPlaceModule,
) -> None:
    """A learned provider's best-scoring pose is not always kinematically reachable."""
    manipulation: Any = module._manipulation
    grasp_generator: Any = module._grasp_generator
    grasp_generator.propose_grasps.return_value = GraspCandidateArray(
        Header(1.0, "world"), [_candidate(0.1, score=0.9), _candidate(0.3, score=0.4)]
    )
    manipulation.plan_to_poses.side_effect = [
        SimpleNamespace(succeeded=False, message="unreachable"),
        SimpleNamespace(succeeded=True, message=""),
        SimpleNamespace(succeeded=True, message=""),
        SimpleNamespace(succeeded=True, message=""),
    ]

    result = module.pick_object("cup-1")

    assert result.success
    assert result.metadata["rank"] == 1
    assert result.metadata["score"] == 0.4


def test_pick_stops_walking_candidates_on_a_drive_fault(module: PickAndPlaceModule) -> None:
    """An execution fault would repeat for every candidate, so it is not a demotion."""
    manipulation: Any = module._manipulation
    grasp_generator: Any = module._grasp_generator
    grasp_generator.propose_grasps.return_value = GraspCandidateArray(
        Header(1.0, "world"), [_candidate(0.1), _candidate(0.3)]
    )
    manipulation.execute.return_value = SimpleNamespace(succeeded=False, message="drive fault")

    result = module.pick_object("cup-1")

    assert not result.success
    assert result.error_code == "EXECUTION_FAILED"
    assert manipulation.plan_to_poses.call_count == 1


def test_pick_reports_no_reachable_candidate_when_every_attempt_fails(
    module: PickAndPlaceModule,
) -> None:
    manipulation: Any = module._manipulation
    grasp_generator: Any = module._grasp_generator
    grasp_generator.propose_grasps.return_value = GraspCandidateArray(
        Header(1.0, "world"), [_candidate(0.1), _candidate(0.3)]
    )
    manipulation.plan_to_poses.return_value = SimpleNamespace(
        succeeded=False, message="unreachable"
    )

    result = module.pick_object("cup-1")

    assert not result.success
    assert result.error_code == "PLANNING_FAILED"
    assert module.get_held_object() is None


def test_proposals_reach_the_viewer_as_they_are_generated(module: PickAndPlaceModule) -> None:
    """get_grasp_candidates only answers after the fact, which is no help live."""
    manipulation: Any = module._manipulation
    shown: list[GraspCandidateArray] = []
    manipulation.show_grasp_proposals.side_effect = lambda array: shown.append(array)

    assert module.pick_object("cup-1").success

    # The stale overlay is cleared first, then the fresh proposals go out.
    assert [[c.score for c in array.candidates] for array in shown] == [[], [1.0]]


def test_pick_object_rejects_empty_candidates(module: PickAndPlaceModule) -> None:
    manipulation: Any = module._manipulation
    grasp_generator: Any = module._grasp_generator
    grasp_generator.propose_grasps.return_value = GraspCandidateArray(Header(1.0, "world"), [])

    result = module.pick_object("cup-1")

    assert result.error_code == "GRASP_GENERATION_FAILED"
    manipulation.set_gripper_position.assert_not_called()


def test_pick_preserves_current_yaw_when_configured(module: PickAndPlaceModule) -> None:
    module.config.yaw_policy = "preserve_current"
    grasp_generator: Any = module._grasp_generator
    grasp_generator.propose_grasps.return_value = GraspCandidateArray(
        Header(1.0, "world"),
        [
            GraspCandidate(
                Pose(
                    Vector3(0.1, 0.0, 0.2),
                    Quaternion.from_euler(Vector3(-3.141592653589793, 0.0, 0.1)),
                ),
                1.0,
            )
        ],
    )

    result = module.pick_object("cup-1")

    assert result.is_success()
    assert module._held is not None
    assert module._held.grasp.orientation.to_euler().z == pytest.approx(0.7)


def test_pick_is_complete_when_the_target_left_its_spot(module: PickAndPlaceModule) -> None:
    """The camera sees the cup 10 cm up, moving with the gripper."""
    scene: Any = module._scene
    scene.scan_scene.return_value = _detections(("cup-1", "cup", Vector3(0.1, 0.0, 0.3)))

    result = module.pick_object("cup-1")

    assert result.success
    assert result.message == "Pick complete"
    assert module.get_held_object() == {"object_id": "cup-1", "name": "cup"}
    scene.scan_scene.assert_called_once_with(text=["cup"])


def test_pick_fails_when_the_target_is_still_where_it_was(module: PickAndPlaceModule) -> None:
    """Jaws that stopped on an edge read as a hold; the re-scan says the cup never moved."""
    scene: Any = module._scene
    manipulation: Any = module._manipulation
    scene.scan_scene.return_value = _detections(("cup-1", "cup", Vector3(0.11, 0.0, 0.2)))

    result = module.pick_object("cup-1")

    assert result.error_code == "GRASP_FAILED"
    assert "cup is still on the table" in result.message
    assert module.get_held_object() is None
    assert manipulation.set_gripper_position.call_args_list[-1].args[0] == 1.0


def test_pick_fails_when_the_jaws_empty_during_the_lift(module: PickAndPlaceModule) -> None:
    manipulation: Any = module._manipulation
    manipulation.get_state.return_value = _group_state(gripper_position=0.02)

    result = module.pick_object("cup-1")

    assert result.error_code == "GRASP_FAILED"
    assert "slipped out" in result.message
    assert module.get_held_object() is None


def test_pick_waits_for_the_manipulation_module_to_start(module: PickAndPlaceModule) -> None:
    """Modules start in parallel; the first pick can arrive before the robot model is loaded."""
    manipulation: Any = module._manipulation
    manipulation.list_planning_groups.side_effect = [(), (), [GROUP], [GROUP]]

    result = module.pick_object("cup-1")

    assert result.success
    assert manipulation.list_planning_groups.call_count == 3


def test_pick_waits_for_the_first_joint_state(module: PickAndPlaceModule) -> None:
    manipulation: Any = module._manipulation
    no_joints = _group_state()
    no_joints.groups["arm/tool"].joints = None
    manipulation.get_state.side_effect = [no_joints, no_joints, _group_state(), _group_state()]

    result = module.pick_object("cup-1")

    assert result.success


def test_pick_reports_a_robot_that_never_comes_up(module: PickAndPlaceModule) -> None:
    manipulation: Any = module._manipulation
    module.config.robot_ready_timeout = 0.03
    manipulation.list_planning_groups.return_value = ()

    result = module.pick_object("cup-1")

    assert result.error_code == "ROBOT_NOT_FOUND"
    assert "planning groups" in result.message
    manipulation.set_gripper_position.assert_not_called()


def test_pick_rejects_an_unknown_group_without_waiting(module: PickAndPlaceModule) -> None:
    manipulation: Any = module._manipulation
    module.config.robot_ready_timeout = 5.0

    result = module.pick_object("cup-1", planning_group="arm/other")

    assert result.error_code == "ROBOT_NOT_FOUND"
    assert manipulation.list_planning_groups.call_count == 1


def test_place_uses_local_axis_and_clears_held_state(module: PickAndPlaceModule) -> None:
    manipulation: Any = module._manipulation
    module._held = _holding_cup()

    result = module.place_at(0.4, 0.0, 0.2)

    assert result.is_success()
    preplace = manipulation.plan_to_poses.call_args_list[0].args[0]["arm/tool"]
    assert preplace.position.z == pytest.approx(0.3)
    assert module.get_held_object() is None


def test_place_refuses_without_a_held_object(module: PickAndPlaceModule) -> None:
    manipulation: Any = module._manipulation

    result = module.place_at(0.4, 0.0, 0.2)

    assert result.error_code == "PLACE_FAILED"
    assert result.message == "nothing is held; pick first"
    manipulation.plan_to_poses.assert_not_called()


def test_place_is_complete_when_the_object_rests_inside_the_tolerance(
    module: PickAndPlaceModule,
) -> None:
    scene: Any = module._scene
    module._held = _holding_cup()
    scene.scan_scene.return_value = _detections(("cup-1", "cup", Vector3(0.43, 0.04, 0.15)))

    result = module.place_at(0.4, 0.0, 0.2)

    assert result.message == "Place complete"
    assert result.metadata["offset_m"] == pytest.approx(0.05)
    assert result.metadata["verified_by"] == "detection"
    scene.scan_scene.assert_called_once_with(text=["cup"])


def test_place_is_incomplete_when_the_object_ends_outside_the_tolerance(
    module: PickAndPlaceModule,
) -> None:
    scene: Any = module._scene
    module._held = _holding_cup()
    scene.scan_scene.return_value = _detections(("cup-1", "cup", Vector3(0.5, 0.0, 0.15)))

    result = module.place_at(0.4, 0.0, 0.2)

    assert result.error_code == "PLACE_INCOMPLETE"
    assert result.message == "cup ended about 10 cm from the target"
    assert module.get_held_object() is None


def test_place_falls_back_to_the_release_pose_when_the_object_is_hidden(
    module: PickAndPlaceModule,
) -> None:
    """Right under the gripper the fingers can hide the object from the wrist camera."""
    module._held = _holding_cup()

    result = module.place_at(0.42, 0.0, 0.2)

    assert result.message == "Place complete"
    assert result.metadata["verified_by"] == "release_pose"
    assert result.metadata["offset_m"] == pytest.approx(0.02)


def test_place_fails_when_the_object_was_lost_on_the_way(module: PickAndPlaceModule) -> None:
    manipulation: Any = module._manipulation
    module._held = _holding_cup()
    manipulation.get_state.return_value = _group_state(gripper_position=0.02)

    result = module.place_at(0.4, 0.0, 0.2)

    assert result.error_code == "PLACE_FAILED"
    assert "no longer in the gripper" in result.message
    assert module.get_held_object() is None
    manipulation.plan_to_poses.assert_not_called()
    assert manipulation.set_gripper_position.call_args_list[-1].args[0] == 1.0


def test_scan_failure_clears_stale_objects(module: PickAndPlaceModule) -> None:
    scene: Any = module._scene
    scene.scan_scene.side_effect = RuntimeError("No aligned RGB-D frame")

    result = module.scan_objects(["cup"])

    assert not result.is_success()
    assert result.error_code == "PERCEPTION_FAILED"
    assert module.get_object("cup-1") is None


def test_pick_rejects_when_already_holding(module: PickAndPlaceModule) -> None:
    manipulation: Any = module._manipulation
    module._held = _holding_cup()

    pick = module.pick_object("cup-1")

    assert pick.error_code == "INVALID_STATE"
    manipulation.set_gripper_position.assert_not_called()


def test_failed_pick_clears_previous_proposals(module: PickAndPlaceModule) -> None:
    module._grasp_candidates = GraspCandidateArray(Header(1.0, "world"), [_candidate(0.1)])

    result = module.pick_object("missing")

    assert result.error_code == "OBJECT_NOT_DETECTED"
    assert module.get_grasp_candidates().candidates == []


def test_pick_retains_held_state_when_retract_fails(module: PickAndPlaceModule) -> None:
    manipulation: Any = module._manipulation
    # Approach in, then the retract out; both legs are linear servos now.
    manipulation.move_linear.side_effect = [
        SimpleNamespace(
            plan=SimpleNamespace(succeeded=True, message=""),
            execution=SimpleNamespace(succeeded=True, message=""),
        ),
        SimpleNamespace(
            plan=SimpleNamespace(succeeded=True, message=""),
            execution=SimpleNamespace(succeeded=False, message="retract failed"),
        ),
    ]

    result = module.pick_object("cup-1")

    assert result.error_code == "EXECUTION_FAILED"
    assert module.get_held_object() == {"object_id": "cup-1", "name": "cup"}


def test_final_grasp_leg_skips_collision_checking(module: PickAndPlaceModule) -> None:
    """The target is mapped geometry, so a checked plan into it always collides."""
    manipulation: Any = module._manipulation

    assert module.pick_object("cup-1").success
    assert manipulation.move_linear.call_args_list
    for call in manipulation.move_linear.call_args_list:
        assert call.kwargs["check_collision"] is False


def test_empty_grasp_reopens_before_failing(
    module: PickAndPlaceModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    manipulation: Any = module._manipulation

    def settle(read: Any, target: float, config: Any, **_: Any) -> GripperSettle:
        position = 0.0 if target == config.closed_position else target
        return GripperSettle(True, position, True, 0.1)

    monkeypatch.setattr("dimos.manipulation.pick_and_place_module.await_gripper_settle", settle)

    result = module.pick_object("cup-1")

    assert result.error_code == "GRASP_VERIFICATION_FAILED"
    assert module.get_held_object() is None
    assert manipulation.set_gripper_position.call_args_list[-1].args[0] == 1.0


def test_pick_rejects_jaws_that_never_closed(
    module: PickAndPlaceModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    def settle(read: Any, target: float, config: Any, **_: Any) -> GripperSettle:
        position = config.open_position if target == config.closed_position else target
        return GripperSettle(True, position, True, 0.1)

    monkeypatch.setattr("dimos.manipulation.pick_and_place_module.await_gripper_settle", settle)

    result = module.pick_object("cup-1")

    assert result.error_code == "GRASP_VERIFICATION_FAILED"
    assert module.get_held_object() is None


def test_empty_grasp_reports_failed_recovery(
    module: PickAndPlaceModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    manipulation: Any = module._manipulation
    manipulation.set_gripper_position.side_effect = [
        SimpleNamespace(succeeded=True, message=""),
        SimpleNamespace(succeeded=True, message=""),
        SimpleNamespace(succeeded=False, message="recovery open failed"),
    ]

    def settle(read: Any, target: float, config: Any, **_: Any) -> GripperSettle:
        position = 0.0 if target == config.closed_position else target
        return GripperSettle(True, position, True, 0.1)

    monkeypatch.setattr("dimos.manipulation.pick_and_place_module.await_gripper_settle", settle)

    result = module.pick_object("cup-1")

    assert result.error_code == "GRIPPER_FAILED"
    assert "recovery open failed" in result.message


def test_pick_fails_when_gripper_command_is_rejected(module: PickAndPlaceModule) -> None:
    manipulation: Any = module._manipulation
    manipulation.set_gripper_position.return_value = SimpleNamespace(
        succeeded=False, message="controller unavailable"
    )

    result = module.pick_object("cup-1")

    assert result.error_code == "GRIPPER_FAILED"


def test_pick_fails_when_gripper_feedback_is_unavailable(
    module: PickAndPlaceModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    def settle(read: Any, target: float, config: Any, **_: Any) -> GripperSettle:
        if target == config.closed_position:
            return GripperSettle(False, None, False, config.timeout)
        return GripperSettle(True, target, True, 0.1)

    monkeypatch.setattr("dimos.manipulation.pick_and_place_module.await_gripper_settle", settle)

    result = module.pick_object("cup-1")

    assert result.error_code == "GRASP_VERIFICATION_FAILED"
    assert module.get_held_object() is None


def test_place_retains_held_state_when_release_fails(
    module: PickAndPlaceModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    module._held = _holding_cup()

    def settle(read: Any, target: float, config: Any, **_: Any) -> GripperSettle:
        return GripperSettle(True, 0.5, True, 0.1)

    monkeypatch.setattr("dimos.manipulation.pick_and_place_module.await_gripper_settle", settle)

    result = module.place_at(0.4, 0.0, 0.2)

    assert result.error_code == "GRIPPER_FAILED"
    assert module.get_held_object() == {"object_id": "cup-1", "name": "cup"}


def test_motion_skills_declare_movement_capability() -> None:
    skills = [
        PickAndPlaceModule.pick_object,
        PickAndPlaceModule.place_at,
        ManipulationSkills.move_to_pose,
        ManipulationSkills.move_to_joints,
        ManipulationSkills.go_home,
        ManipulationSkills.go_init,
        ManipulationSkills.set_gripper,
        ManipulationSkills.open_gripper,
        ManipulationSkills.close_gripper,
    ]

    assert all(skill.__skill_uses__ == ["movement"] for skill in skills)


def _exclusions(manipulation: Any) -> list[tuple[Any, ...]]:
    """Every set_voxel_map_exclusion call as (center, size, planning_group)."""
    return [
        (c.args[1], c.args[2], c.kwargs.get("planning_group"))
        for c in manipulation.set_voxel_map_exclusion.call_args_list
    ]


def test_pick_hides_the_target_from_the_planner_then_carries_it(
    module: PickAndPlaceModule,
) -> None:
    """The target is mapped geometry; the approach goes into it and the lift takes it along."""
    manipulation: Any = module._manipulation

    assert module.pick_object("cup-1").success

    fixed, carried = _exclusions(manipulation)
    # The cup's bounds plus the 3 cm clearance each way, fixed where it stood.
    assert fixed[0].to_tuple() == pytest.approx((0.1, 0.0, 0.2))
    assert fixed[1].to_tuple() == pytest.approx((0.1, 0.1, 0.1))
    assert fixed[2] is None
    # Then the same box rides with the gripper: the grasp was at the cup's
    # centre, so the box centre is at the tip.
    assert carried[0].to_tuple() == pytest.approx((0.0, 0.0, 0.0), abs=1e-6)
    assert carried[1].to_tuple() == pytest.approx((0.1, 0.1, 0.1))
    assert carried[2] == "arm/tool"
    manipulation.clear_voxel_map_exclusion.assert_not_called()


def test_a_failed_pick_makes_the_target_an_obstacle_again(module: PickAndPlaceModule) -> None:
    manipulation: Any = module._manipulation
    manipulation.plan_to_poses.return_value = SimpleNamespace(succeeded=False, message="no")

    result = module.pick_object("cup-1")

    assert result.error_code == "PLANNING_FAILED"
    assert len(_exclusions(manipulation)) == 1
    manipulation.clear_voxel_map_exclusion.assert_called_once_with(VOXEL_MAP_EXCLUSION_ID)


def test_a_lift_that_left_the_object_behind_clears_the_box(module: PickAndPlaceModule) -> None:
    manipulation: Any = module._manipulation
    scene: Any = module._scene
    scene.scan_scene.return_value = _detections(("cup-2", "cup", CUP_AT))

    result = module.pick_object("cup-1")

    assert result.error_code == "GRASP_FAILED"
    manipulation.clear_voxel_map_exclusion.assert_called_once_with(VOXEL_MAP_EXCLUSION_ID)


def test_release_makes_the_placed_object_an_obstacle_again(module: PickAndPlaceModule) -> None:
    manipulation: Any = module._manipulation
    module._held = _holding_cup()

    assert module.place_at(0.4, 0.0, 0.2).success

    manipulation.clear_voxel_map_exclusion.assert_called_once_with(VOXEL_MAP_EXCLUSION_ID)


def test_an_empty_target_cloud_asks_for_no_exclusion(module: PickAndPlaceModule) -> None:
    manipulation: Any = module._manipulation
    scene: Any = module._scene
    scene.get_object_pointcloud_by_object_id.return_value = PointCloud2.from_numpy(
        np.zeros((0, 3), dtype=np.float32), frame_id="world", timestamp=1.0
    )

    assert module.pick_object("cup-1").success

    manipulation.set_voxel_map_exclusion.assert_not_called()


def test_an_empty_close_backs_out_before_the_target_is_an_obstacle_again(
    module: PickAndPlaceModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    manipulation: Any = module._manipulation
    manipulation.grasp_verification = None
    monkeypatch.setattr(
        "dimos.manipulation.pick_and_place_module.await_gripper_settle",
        lambda read, target, config, **_: GripperSettle(
            True, 0.0 if target == 0.0 else 1.0, True, 0.1
        ),
    )
    order: list[str] = []
    manipulation.move_linear.side_effect = lambda *a, **k: (
        order.append("servo"),
        SimpleNamespace(
            plan=SimpleNamespace(succeeded=True, message=""),
            execution=SimpleNamespace(succeeded=True, message=""),
        ),
    )[1]
    manipulation.clear_voxel_map_exclusion.side_effect = lambda name: order.append("clear")

    result = module.pick_object("cup-1")

    assert result.error_code == "GRASP_VERIFICATION_FAILED"
    # Down to the grasp, back up to the pregrasp, and only then the box goes.
    assert order == ["servo", "servo", "clear"]
    dz = [c.args[2] for c in manipulation.move_linear.call_args_list]
    assert dz[0] == pytest.approx(-dz[1])
