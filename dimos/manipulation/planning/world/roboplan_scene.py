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

"""Native scene construction shared by independent RoboPlan consumers."""

from collections.abc import Sequence
from pathlib import Path
from typing import Any


def create_roboplan_scene(
    core: Any, *, name: str, urdf: str, srdf: str, package_paths: Sequence[str], joint_limits: Path
) -> Any:
    description = core.loadUrdfSceneDescriptionFromXml(urdf, package_paths)
    scene = core.Scene(name, description)
    scene.importSrdf(srdf)
    scene.importJointLimitsFromConfig(core.loadJointLimitsConfig(joint_limits))
    return scene
