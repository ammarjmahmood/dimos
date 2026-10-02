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

"""Commands and feedback shared by manipulation control and transports."""

from dataclasses import dataclass
from typing import Annotated, Any, Literal

from pydantic import BaseModel, ConfigDict, Field, TypeAdapter


class ArmCommand(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False, strict=True)
    id: str = Field(min_length=1, max_length=128)


class JointTarget(ArmCommand):
    kind: Literal["joints"]
    positions: list[float] = Field(min_length=1)
    timeout_s: float = Field(default=10.0, gt=0, le=30)


class DeltaTarget(ArmCommand):
    kind: Literal["delta"]
    xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)
    frame: Literal["base"] = "base"
    timeout_s: float = Field(default=10.0, gt=0, le=30)


class GripperTarget(ArmCommand):
    kind: Literal["gripper"]
    opening: float = Field(ge=0, le=1)
    timeout_s: float = Field(default=5.0, gt=0, le=30)


class StopCommand(ArmCommand):
    kind: Literal["stop"]


Command = JointTarget | DeltaTarget | GripperTarget | StopCommand
COMMAND: TypeAdapter[Command] = TypeAdapter(Annotated[Command, Field(discriminator="kind")])
FeedbackKind = Literal["state", "info", "status", "camera_pose", "overview_camera_pose"]


@dataclass(frozen=True)
class ManipulationFeedback:
    kind: FeedbackKind
    data: dict[str, Any]
    ts: float | None = None
