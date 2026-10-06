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

"""Robot configuration for manipulation planning."""

from __future__ import annotations

import math
from pathlib import Path

from pydantic import BaseModel, Field, model_validator

from dimos.core.module import ModuleConfig
from dimos.manipulation.grasp_verification import GraspVerificationConfig
from dimos.manipulation.planning.groups.identifiers import assert_valid_joint_names
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.robot.assets.model import RobotModel


class JointStateTransform(BaseModel):
    """Explicit affine conversion from a measured source coordinate to model units.

    Bounds apply to the source position. Reject mismatched units rather than
    clamping an invalid measurement into a plausible model pose.
    """

    source: str
    scale: float
    offset: float
    source_bounds: tuple[float, float]

    @model_validator(mode="after")
    def validate_conversion(self) -> JointStateTransform:
        low, high = self.source_bounds
        if (
            not self.source
            or not all(math.isfinite(v) for v in (self.scale, self.offset, low, high))
            or self.scale == 0
            or low >= high
        ):
            raise ValueError("Invalid measured joint conversion")
        return self

    def position(self, value: float) -> float:
        low, high = self.source_bounds
        if not math.isfinite(value) or not low <= value <= high:
            raise ValueError(f"Measured joint '{self.source}' is outside {self.source_bounds}")
        return self.scale * value + self.offset


class RobotModelConfig(ModuleConfig):
    """Configuration for the logical robot model loaded into the world.

    Attributes:
        model: Portable robot model loaded by backend adapters
        srdf_path: Optional path to SRDF file containing planning group definitions
        base_pose: Placement transform for the model's base link in the world.
        joint_names: Ordered list of controllable joints in the canonical model
            namespace. This is not a planning group.
        joint_state_transforms: Measured source-to-model coordinate conversions,
            keyed by canonical target joint. These do not transform commands.
        base_link: Robot-scoped link that base_pose places in the world and
            current backends use for weld/placement.
        auto_convert_meshes: Auto-convert DAE/STL meshes to OBJ for Drake
        collision_exclusion_pairs: List of (link1, link2) pairs to exclude from collision.
            Useful for parallel linkage mechanisms like grippers where non-adjacent
            links may legitimately overlap (e.g., mimic joints).
    """

    model: RobotModel
    srdf_path: Path | None = None
    base_pose: PoseStamped = Field(default_factory=PoseStamped)
    joint_names: list[str]
    joint_state_transforms: dict[str, JointStateTransform] = Field(default_factory=dict)
    base_link: str = "base_link"
    planning_groups: list[PlanningGroupDefinition] = Field(default_factory=list)
    auto_convert_meshes: bool = False
    collision_exclusion_pairs: list[tuple[str, str]] = Field(default_factory=list)
    gripper_hardware_id: str | None = None
    # TF publishing for extra links (e.g., camera mount)
    tf_extra_links: list[str] = Field(default_factory=list)
    # Home/observe joint configuration for go_home skill
    home_joints: list[float] | None = None
    # Pre-grasp offset distance in meters (along approach direction)
    pre_grasp_offset: float = 0.10
    # Gripper feedback thresholds for pick/place.
    grasp_verification: GraspVerificationConfig = Field(default_factory=GraspVerificationConfig)

    def model_post_init(self, __context: object) -> None:
        """Validate canonical joint-name constraints."""
        assert_valid_joint_names(self.joint_names)
        if any(not name for name in self.joint_names):
            raise ValueError("RobotModelConfig.joint_names must contain non-empty names")
        if len(self.joint_names) != len(set(self.joint_names)):
            raise ValueError("RobotModelConfig contains duplicate canonical joint names")
        if set(self.joint_state_transforms) - set(self.joint_names):
            raise ValueError("Joint-state transform targets must be model joints")
        sources = [mapping.source for mapping in self.joint_state_transforms.values()]
        if len(sources) != len(set(sources)) or set(sources) & set(self.joint_names):
            raise ValueError("Joint-state transform sources must be unique non-model joints")
