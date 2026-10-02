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

"""Capability-composed pick-and-place workflow."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
import math
import time
from typing import Any, Literal

import numpy as np
from pydantic import Field

from dimos.agents.annotation import skill
from dimos.agents.capabilities import CAP_MOVEMENT
from dimos.agents.skill_result import SkillResult
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.manipulation.grasp_verification import (
    GraspVerificationConfig,
    GripperSettle,
    await_gripper_settle,
    grasp_failure,
    open_failure,
)
from dimos.manipulation.grasping.grasp_gen_spec import GraspGenSpec
from dimos.manipulation.manipulation_spec import ManipulationSpec, PlanningGroupInfo
from dimos.manipulation.planning.spec.models import PlanningGroupID
from dimos.manipulation.skill_errors import ManipulationSkillError
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.perception.experimental.object_scene_registration_spec import ObjectSceneRegistrationSpec

# The one box of mapped cells a pick asks the planner to ignore.
VOXEL_MAP_EXCLUSION_ID = "pick-and-place/target"


class PickAndPlaceModuleConfig(ModuleConfig):
    planning_frame: str = "base_link"
    pregrasp_offset: float = Field(default=0.10, gt=0.0)
    # Margin (metres) added around the target's bounding box. Mapped cells in
    # that box are not obstacles during the pick, and the same box rides with
    # the gripper while the object is held. Too small, and a cell on the
    # object's edge or the table under it stops the plan; too large, and a
    # neighbour's cells are ignored too.
    target_clearance: float = Field(default=0.03, gt=0.0)
    # A learned provider returns a ranked spread whose best-scoring pose is not
    # always kinematically reachable; a single-candidate provider is unaffected.
    max_grasp_attempts: int = Field(default=5, gt=0)
    yaw_policy: Literal["generated", "preserve_current"] = "generated"
    grasp_verification: GraspVerificationConfig = Field(default_factory=GraspVerificationConfig)
    # A target still seen within this distance (metres) of where it was scanned
    # never left the table, whatever the jaws report.
    grasp_displacement_tolerance: float = Field(default=0.03, gt=0.0)
    # How far (metres, horizontally) a released object may rest from the requested point.
    place_tolerance: float = Field(default=0.07, gt=0.0)
    # How long (seconds) a pick or place waits for the manipulation module to list
    # its planning groups and receive the arm's first joint state.
    robot_ready_timeout: float = Field(default=10.0, gt=0.0)
    robot_ready_poll_interval: float = Field(default=0.25, gt=0.0)


@dataclass(frozen=True)
class ScannedObject:
    """One object from the latest scan: its id, label and centre in the planning frame."""

    object_id: str
    name: str
    position: Vector3


@dataclass(frozen=True)
class TargetBox:
    """Axis-aligned bounds of a target's point cloud, planning frame, metres."""

    center: Vector3
    size: Vector3


@dataclass(frozen=True)
class HeldObject:
    """The object a pick left in the gripper, and where it was taken from."""

    object_id: str
    name: str
    grasp: PoseStamped
    picked_from: Vector3


class PickAndPlaceModule(Module):
    """Coordinate scene registration, grasp generation, and manipulation execution."""

    config: PickAndPlaceModuleConfig
    _scene: ObjectSceneRegistrationSpec
    _grasp_generator: GraspGenSpec
    _manipulation: ManipulationSpec

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._objects: dict[str, ScannedObject] = {}
        self._grasp_candidates = GraspCandidateArray()
        self._held: HeldObject | None = None

    @skill
    def scan_objects(self, prompts: list[str]) -> SkillResult[ManipulationSkillError]:
        """Scan the latest RGB-D frame for prompted objects.

        Args:
            prompts: Object labels to detect. Use an ID from this scan with pick_object.
        """
        prompts = [prompt.strip() for prompt in prompts if prompt.strip()]
        if not prompts:
            return SkillResult.fail("INVALID_INPUT", "At least one object prompt is required")
        if self._held is None:
            self._clear_proposals()
        self._objects = {}
        try:
            detections = self._scene.scan_scene(text=prompts)
        except RuntimeError as exc:
            return SkillResult.fail("PERCEPTION_FAILED", str(exc))
        self._objects = {
            str(detection.id): ScannedObject(
                object_id=str(detection.id),
                name=str(detection.results[0].hypothesis.class_id),
                position=_center(detection),
            )
            for detection in detections.detections
            if detection.id and detection.results
        }
        return SkillResult.ok(
            f"Detected {detections.detections_length} object(s)",
            prompts=prompts,
            objects=[
                {"object_id": obj.object_id, "name": obj.name} for obj in self._objects.values()
            ],
        )

    @rpc
    def get_object(self, object_id: str) -> dict[str, Any] | None:
        obj = self._objects.get(object_id)
        return None if obj is None else {"object_id": obj.object_id, "name": obj.name}

    @rpc
    def get_held_object(self) -> dict[str, Any] | None:
        """The object the gripper holds after a verified pick, or None."""
        held = self._held
        return None if held is None else {"object_id": held.object_id, "name": held.name}

    @skill(uses=[CAP_MOVEMENT])
    def pick_object(
        self, object_id: str, planning_group: PlanningGroupID | None = None
    ) -> SkillResult[ManipulationSkillError]:
        """Grasp one object from the latest scan and lift it clear of the surface.

        "Pick complete" is confirmed by the camera: the object left the spot it was
        scanned at. GRASP_FAILED means it is still on the table; re-scan and try again.

        Args:
            object_id: Exact object ID returned by the latest scan_objects call.
            planning_group: Gripper-capable pose group; omitted only when unambiguous.
        """
        if self._held is not None:
            return SkillResult.fail(
                "INVALID_STATE", f"Still holding {self._held.name}; place it before picking again"
            )
        self._clear_proposals()
        target = self._objects.get(object_id)
        if target is None:
            return SkillResult.fail("OBJECT_NOT_DETECTED", f"Unknown object_id: {object_id}")
        group = self._await_group(planning_group)
        if isinstance(group, SkillResult):
            return group
        try:
            pointcloud = self._scene.get_object_pointcloud_by_object_id(object_id)
            if pointcloud is None:
                return SkillResult.fail(
                    "OBJECT_NOT_DETECTED", f"No pointcloud for object_id: {object_id}"
                )
            candidates = self._grasp_generator.propose_grasps(pointcloud)
        except (RuntimeError, ValueError) as exc:
            return SkillResult.fail("GRASP_GENERATION_FAILED", str(exc))
        self._grasp_candidates = candidates
        self._manipulation.show_grasp_proposals(candidates)
        if candidates.header.frame_id != self.config.planning_frame:
            return SkillResult.fail(
                "GRASP_FRAME_MISMATCH",
                f"Expected {self.config.planning_frame}, got {candidates.header.frame_id}",
            )
        if not candidates.candidates:
            return SkillResult.fail("GRASP_GENERATION_FAILED", "No grasp candidates generated")
        box = _target_box(pointcloud)
        # The target is mapped geometry: with its cells as obstacles the planner
        # refuses the approach into it, and the held object blocks every move after.
        self._exclude_target(box)
        try:
            return self._pick_from(candidates, target, box, group)
        finally:
            if self._held is None:
                self._manipulation.clear_voxel_map_exclusion(VOXEL_MAP_EXCLUSION_ID)

    def _pick_from(
        self,
        candidates: GraspCandidateArray,
        target: ScannedObject,
        box: TargetBox | None,
        group: PlanningGroupID,
    ) -> SkillResult[ManipulationSkillError]:
        object_id = target.object_id
        if failure := self._open_gripper(group, "pre-grasp open"):
            return failure

        unreachable: SkillResult[ManipulationSkillError] | None = None
        for rank, candidate in enumerate(candidates.candidates[: self.config.max_grasp_attempts]):
            grasp = self._apply_yaw_policy(
                PoseStamped(
                    ts=candidates.header.timestamp,
                    frame_id=candidates.header.frame_id,
                    position=candidate.pose.position,
                    orientation=candidate.pose.orientation,
                ),
                group,
            )
            pregrasp = self._offset_pose(grasp, self.config.pregrasp_offset)
            failure = self._move(pregrasp, group) or self._servo(pregrasp, grasp, group)
            if failure is not None:
                # Only an unreachable pose is worth demoting to the next candidate;
                # a drive or execution fault would repeat for every one of them.
                if failure.error_code != "PLANNING_FAILED":
                    return failure
                unreachable = failure
                continue
            if failure := self._close_and_verify(group):
                # Back out while the target's cells are still ignored: left at
                # the grasp pose, the arm would stand inside the obstacle the
                # target becomes again, and no later plan could start.
                self._servo(grasp, pregrasp, group)
                return failure

            # The jaws stopped on something; the camera has the last word after the lift.
            held = HeldObject(object_id, target.name, grasp, target.position)
            self._held = held
            self._carry_target(box, grasp, group)
            if failure := self._servo(grasp, pregrasp, group):
                return failure
            if failure := self._verify_lift(held, group):
                return failure
            return SkillResult.ok(
                "Pick complete",
                object_id=object_id,
                name=target.name,
                rank=rank,
                score=candidate.score,
                candidates=len(candidates.candidates),
            )
        return unreachable or SkillResult.fail(
            "PLANNING_FAILED", "No grasp candidate was reachable"
        )

    @rpc
    def get_grasp_candidates(self) -> GraspCandidateArray:
        return self._grasp_candidates

    @skill(uses=[CAP_MOVEMENT])
    def place_at(
        self,
        x: float,
        y: float,
        z: float,
        planning_group: PlanningGroupID | None = None,
    ) -> SkillResult[ManipulationSkillError]:
        """Lower the held object to a planning-frame position and release it.

        PLACE_FAILED means nothing is held: pick first. "Place complete" is confirmed by
        the camera after release; PLACE_INCOMPLETE says how far from the target the
        object ended.

        Args:
            x: Planning-frame X coordinate in meters.
            y: Planning-frame Y coordinate in meters.
            z: Planning-frame Z coordinate in meters.
            planning_group: Gripper-capable pose group; omitted only when unambiguous.
        """
        held = self._held
        if held is None:
            return SkillResult.fail("PLACE_FAILED", "nothing is held; pick first")
        group = self._await_group(planning_group)
        if isinstance(group, SkillResult):
            return group
        if failure := self._lost_in_transit(held, group):
            return failure
        place = PoseStamped(
            frame_id=self.config.planning_frame,
            position=Vector3(x, y, z),
            orientation=held.grasp.orientation,
        )
        preplace = self._offset_pose(place, self.config.pregrasp_offset)
        if failure := self._move(preplace, group):
            return failure
        if failure := self._servo(preplace, place, group):
            return failure
        if failure := self._open_gripper(group, "release"):
            return failure
        self._held = None
        self._clear_proposals()
        self._manipulation.clear_voxel_map_exclusion(VOXEL_MAP_EXCLUSION_ID)
        tip = self._tip_position(group)
        released_at = place.position if tip is None else tip
        if failure := self._servo(place, preplace, group):
            return failure
        return self._verify_place(held, place.position, released_at)

    def _verify_lift(
        self, held: HeldObject, planning_group: PlanningGroupID
    ) -> SkillResult[ManipulationSkillError] | None:
        """Confirm the lifted object came along, or why it did not.

        A close that stops early on an edge reads as a hold, so the jaws are only a first
        filter. The wrist camera now looks down at where the object was: if the target is
        still detected there, the grasp missed it.
        """
        jaws = self._gripper_position(planning_group)
        if jaws is not None and jaws <= self.config.grasp_verification.held_low:
            return self._grasp_failed(
                planning_group, f"{held.name} slipped out during the lift (jaws read {jaws:.3f})"
            )
        try:
            detections = self._scene.scan_scene(text=[held.name])
        except RuntimeError as exc:
            return SkillResult.fail(
                "PERCEPTION_FAILED",
                f"Lifted {held.name} but could not check whether it came along: {exc}",
            )
        tolerance = self.config.grasp_displacement_tolerance
        if any(center.distance(held.picked_from) <= tolerance for center in _centers(detections)):
            return self._grasp_failed(
                planning_group,
                f"{held.name} is still on the table at its original position; "
                "re-scan and try again, or try a different grasp",
            )
        return None

    def _grasp_failed(
        self, planning_group: PlanningGroupID, reason: str
    ) -> SkillResult[ManipulationSkillError]:
        self._held = None
        self._clear_proposals()
        if recovery := self._open_gripper(planning_group, "failed-grasp release"):
            return recovery
        return SkillResult.fail("GRASP_FAILED", reason)

    def _lost_in_transit(
        self, held: HeldObject, planning_group: PlanningGroupID
    ) -> SkillResult[ManipulationSkillError] | None:
        """Fail the place when the jaws have closed on nothing since the pick."""
        jaws = self._gripper_position(planning_group)
        if jaws is None or jaws > self.config.grasp_verification.held_low:
            return None
        self._held = None
        self._clear_proposals()
        self._manipulation.clear_voxel_map_exclusion(VOXEL_MAP_EXCLUSION_ID)
        if recovery := self._open_gripper(planning_group, "empty-gripper recovery"):
            return recovery
        return SkillResult.fail(
            "PLACE_FAILED",
            f"{held.name} is no longer in the gripper (jaws read {jaws:.3f}); "
            "re-scan and pick again",
        )

    def _verify_place(
        self, held: HeldObject, target: Vector3, released_at: Vector3
    ) -> SkillResult[ManipulationSkillError]:
        """Report where the released object rests relative to the requested point."""
        try:
            detections = self._scene.scan_scene(text=[held.name])
        except RuntimeError as exc:
            return SkillResult.fail(
                "PERCEPTION_FAILED",
                f"Released {held.name} but could not check where it landed: {exc}",
            )
        centers = _centers(detections)
        if centers:
            rest = min(centers, key=lambda center: _horizontal_distance(center, target))
            verified_by = "detection"
        else:
            # Right under the gripper the object can hide behind the fingers; where
            # the jaws opened is then the best estimate of where it rests.
            rest, verified_by = released_at, "release_pose"
        offset = _horizontal_distance(rest, target)
        if offset <= self.config.place_tolerance:
            return SkillResult.ok(
                "Place complete",
                object_id=held.object_id,
                name=held.name,
                offset_m=round(offset, 3),
                verified_by=verified_by,
            )
        return SkillResult.fail(
            "PLACE_INCOMPLETE", f"{held.name} ended about {offset * 100:.0f} cm from the target"
        )

    def _clear_proposals(self) -> None:
        self._grasp_candidates = GraspCandidateArray()
        self._manipulation.show_grasp_proposals(GraspCandidateArray())

    def _exclude_target(self, box: TargetBox | None) -> None:
        """Keep the target's own cells, and the table right under it, out of the planner."""
        if box is None:
            return
        self._manipulation.set_voxel_map_exclusion(
            VOXEL_MAP_EXCLUSION_ID, box.center, self._padded(box.size)
        )

    def _carry_target(
        self, box: TargetBox | None, grasp: PoseStamped, group: PlanningGroupID
    ) -> None:
        """Move the exclusion from the table to the gripper, where the object now is."""
        if box is None:
            return
        # The box centre relative to the tool tip, in the tip's own frame.
        offset = grasp.orientation.inverse().rotate_vector(box.center - grasp.position)
        self._manipulation.set_voxel_map_exclusion(
            VOXEL_MAP_EXCLUSION_ID, offset, self._padded(box.size), planning_group=group
        )

    def _padded(self, size: Vector3) -> Vector3:
        margin = 2.0 * self.config.target_clearance
        return Vector3(size.x + margin, size.y + margin, size.z + margin)

    def _await_group(
        self, planning_group: PlanningGroupID | None
    ) -> PlanningGroupID | SkillResult[ManipulationSkillError]:
        """Resolve the gripper group once the manipulation module and the arm are up.

        Modules start in parallel, so an early call can land before the manipulation
        module has loaded its robot model or received the arm's first joint state.
        """
        deadline = time.monotonic() + self.config.robot_ready_timeout
        while True:
            groups = self._manipulation.list_planning_groups()
            if groups:
                group = self._resolve_group(groups, planning_group)
                if group is None:
                    return SkillResult.fail(
                        "ROBOT_NOT_FOUND", "Gripper-capable planning group is missing or ambiguous"
                    )
                state = self._manipulation.get_state().groups.get(group)
                if state is not None and state.joints is not None:
                    return group
                waiting_for = "the arm's first joint state"
            else:
                waiting_for = "the manipulation module to list its planning groups"
            if time.monotonic() >= deadline:
                return SkillResult.fail(
                    "ROBOT_NOT_FOUND",
                    f"Waited {self.config.robot_ready_timeout:.0f}s for {waiting_for}; "
                    "is the arm driver running?",
                )
            time.sleep(self.config.robot_ready_poll_interval)

    @staticmethod
    def _resolve_group(
        groups: Sequence[PlanningGroupInfo], planning_group: PlanningGroupID | None
    ) -> PlanningGroupID | None:
        gripper_groups = [
            group for group in groups if group.has_gripper and group.tip_frame is not None
        ]
        if planning_group is not None:
            return (
                planning_group
                if any(group.id == planning_group for group in gripper_groups)
                else None
            )
        return gripper_groups[0].id if len(gripper_groups) == 1 else None

    def _apply_yaw_policy(self, pose: PoseStamped, group: PlanningGroupID) -> PoseStamped:
        if self.config.yaw_policy == "generated":
            return pose
        current = self._manipulation.get_state().groups[group].end_effector_pose
        if current is None:
            return pose
        euler = pose.orientation.to_euler()
        current_euler = current.orientation.to_euler()
        return PoseStamped(
            ts=pose.ts,
            frame_id=pose.frame_id,
            position=pose.position,
            orientation=Quaternion.from_euler(Vector3(euler.x, euler.y, current_euler.z)),
        )

    @staticmethod
    def _offset_pose(pose: PoseStamped, offset: float) -> PoseStamped:
        return PoseStamped(
            ts=pose.ts,
            frame_id=pose.frame_id,
            position=pose.position + pose.orientation.rotate_vector(Vector3(0.0, 0.0, -offset)),
            orientation=pose.orientation,
        )

    def _servo(
        self, start: PoseStamped, end: PoseStamped, planning_group: PlanningGroupID
    ) -> SkillResult[ManipulationSkillError] | None:
        """Drive the last leg as a straight line with collision checking off.

        The object being grasped is itself mapped geometry once a voxel map feeds
        the planner, so a collision-checked plan into it can only ever be
        rejected. This leg is short, straight, and deliberately ends in contact.
        """
        result = self._manipulation.move_linear(
            end.position.x - start.position.x,
            end.position.y - start.position.y,
            end.position.z - start.position.z,
            planning_group,
            check_collision=False,
        )
        if not result.plan.succeeded:
            # A planning failure demotes to the next candidate; a drive fault
            # would repeat for every one of them, so keep the two distinct.
            return SkillResult.fail("PLANNING_FAILED", result.plan.message)
        if result.execution is None or not result.execution.succeeded:
            message = "" if result.execution is None else result.execution.message
            return SkillResult.fail("EXECUTION_FAILED", message)
        return None

    def _move(
        self, pose: PoseStamped, planning_group: PlanningGroupID
    ) -> SkillResult[ManipulationSkillError] | None:
        plan = self._manipulation.plan_to_poses({planning_group: pose})
        if not plan.succeeded:
            return SkillResult.fail("PLANNING_FAILED", plan.message)
        execution = self._manipulation.execute(blocking=True)
        if not execution.succeeded:
            return SkillResult.fail("EXECUTION_FAILED", execution.message)
        return None

    def _command_and_settle(
        self,
        position: float,
        planning_group: PlanningGroupID,
        arrival_tolerance: float | None = None,
    ) -> GripperSettle | SkillResult[ManipulationSkillError]:
        result = self._manipulation.set_gripper_position(position, planning_group)
        if not result.succeeded:
            return SkillResult.fail(
                "GRIPPER_FAILED", result.message or "Gripper command was rejected"
            )
        return await_gripper_settle(
            lambda: self._gripper_position(planning_group),
            position,
            self.config.grasp_verification,
            arrival_tolerance=arrival_tolerance,
        )

    def _open_gripper(
        self, planning_group: PlanningGroupID, step: str
    ) -> SkillResult[ManipulationSkillError] | None:
        # Jaws resting against the open stop never move and never reach the
        # commanded extreme; open_tolerance is the band that already decides
        # whether where they stopped counts as open.
        settle = self._command_and_settle(
            self.config.grasp_verification.open_position,
            planning_group,
            arrival_tolerance=self.config.grasp_verification.open_tolerance,
        )
        if isinstance(settle, SkillResult):
            return settle
        if settle.position is None:
            return None
        if failure := open_failure(settle, self.config.grasp_verification):
            return SkillResult.fail("GRIPPER_FAILED", f"{step}: {failure}")
        return None

    def _close_and_verify(
        self, planning_group: PlanningGroupID
    ) -> SkillResult[ManipulationSkillError] | None:
        settle = self._command_and_settle(
            self.config.grasp_verification.closed_position, planning_group
        )
        if isinstance(settle, SkillResult):
            return settle
        if not self.config.grasp_verification.enabled:
            return None
        if failure := grasp_failure(settle, self.config.grasp_verification):
            if settle.position is not None and "nothing in the jaws" in failure:
                recovered = self._open_gripper(planning_group, "empty-grasp recovery")
                if recovered:
                    return recovered
            return SkillResult.fail("GRASP_VERIFICATION_FAILED", failure)
        return None

    def _gripper_position(self, planning_group: PlanningGroupID) -> float | None:
        state = self._manipulation.get_state().groups.get(planning_group)
        return state.gripper_position if state is not None else None

    def _tip_position(self, planning_group: PlanningGroupID) -> Vector3 | None:
        state = self._manipulation.get_state().groups.get(planning_group)
        if state is None or state.end_effector_pose is None:
            return None
        return state.end_effector_pose.position


def _target_box(pointcloud: PointCloud2) -> TargetBox | None:
    """The axis-aligned bounds of an object's points, or None for an empty cloud."""
    points = np.asarray(pointcloud.points_f32(), dtype=np.float64).reshape((-1, 3))
    if not len(points) or not np.isfinite(points).all():
        return None
    low, high = points.min(axis=0), points.max(axis=0)
    return TargetBox(Vector3(*((low + high) / 2.0)), Vector3(*(high - low)))


def _center(detection: Any) -> Vector3:
    position = detection.bbox.center.position
    return Vector3(position.x, position.y, position.z)


def _centers(detections: Detection3DArray) -> list[Vector3]:
    return [_center(detection) for detection in detections.detections]


def _horizontal_distance(a: Vector3, b: Vector3) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)
