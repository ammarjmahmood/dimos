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

"""Robosuite models for existing DimOS MJCF assets, without physics retuning."""

from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from tempfile import TemporaryDirectory
from typing import Any
import xml.etree.ElementTree as ET

import mujoco
from robosuite.models.base import MujocoXML
from robosuite.models.robots.robot_model import RobotModel as SuiteRobotModel

Defaults = dict[str, dict[str, dict[str, str]]]


class MJCFInput:
    """Preserve nested/implicit defaults before robosuite merges XML components.

    Native MuJoCo resolves includes, angle units, implicit names and actuator
    shortcuts. Robosuite then consumes expanded defaults. No new asset format
    or persistent converted closure is created. Native XML serialization adds
    a cold preparation cost, which is included in the comparison measurements.
    """

    root: ET.Element
    asset: ET.Element
    folder: str
    mesh_directory: Path | None = None
    _authored_geoms: list[dict[str, str]]

    def __init__(self, fname: str, *args: Any, **kwargs: Any) -> None:
        source = Path(fname).resolve()
        spec = mujoco.MjSpec.from_file(str(source))
        if self.mesh_directory is not None:
            spec.meshdir = str(self.mesh_directory)
        for entries, directory in (
            (spec.meshes, spec.meshdir),
            (spec.textures, spec.texturedir),
            (spec.hfields, ""),
        ):
            for asset in entries:
                if asset.file:
                    asset.file = str((source.parent / directory / asset.file).resolve())
        spec.meshdir = ""
        spec.texturedir = ""
        for key in list(spec.keys):
            spec.delete(key)
        with TemporaryDirectory(prefix="dimos-robosuite-model-") as directory:
            path = Path(directory) / "model.xml"
            path.write_text(spec.to_xml())
            super().__init__(str(path), *args, **kwargs)  # type: ignore[call-arg]  # MJCF mixin

    def _get_default_classes(self, default: ET.Element) -> Defaults:
        result: Defaults = {}

        def visit(node: ET.Element, parent: dict[str, dict[str, str]]) -> None:
            values = deepcopy(parent)
            for child in node:
                if child.tag != "default":
                    values.setdefault(child.tag, {}).update(child.attrib)
            result[node.get("class", "main")] = values
            for child in node.findall("default"):
                visit(child, values)

        visit(default, {})
        return result

    def _replace_defaults_inline(self, default_dic: Defaults) -> None:
        def visit(node: ET.Element, inherited: str) -> None:
            if node.tag == "default":
                return
            active = node.attrib.pop("class", inherited)
            children = node.attrib.pop("childclass", inherited)
            values = default_dic[active].get(node.tag, {})
            node.attrib = {**values, **node.attrib}
            for child in node:
                visit(child, children)

        visit(self.root, "main")
        worldbody = self.root.find("worldbody")
        assert worldbody is not None
        self._authored_geoms = [dict(geom.attrib) for geom in worldbody.iter("geom")]
        for index, geom in enumerate(worldbody.iter("geom")):
            if "name" not in geom.attrib:
                geom.set("name", f"dimos_geom_{index}")

    def resolve_asset_dependency(self) -> None:
        for asset in self.asset:
            for key in tuple(asset.attrib):
                if key != "file" and not key.startswith("file"):
                    continue
                if not Path(asset.get(key, "")).is_absolute():
                    raise ValueError(
                        f"native MJCF serialization left an unresolved asset: {asset.attrib}"
                    )


class SceneModel(MJCFInput, MujocoXML):  # type: ignore[misc]  # Upstream is untyped.
    """An authored world component; no extra robot-control marker bodies."""


class RobotModel(MJCFInput, SuiteRobotModel):  # type: ignore[misc]  # Upstream is untyped.
    """Base for robot-local definitions served by DimOS motor firmware.

    Bodies/actuators remain ordinary robosuite model components. DimOS owns
    policy execution, so the firmware runtime does not require arm/EEF parts.
    """

    arms: tuple[str, ...] = ()

    def __init__(self, fname: Path, idn: str = "0", *, meshdir: Path | None = None) -> None:
        self.mesh_directory = meshdir
        # Keep authored collision appearance: sim2 renders all non-hidden geoms.
        super().__init__(str(fname), idn=idn)
        for before, after in zip(self._authored_geoms, self.worldbody.iter("geom"), strict=True):
            for key in ("rgba", "material"):
                if key in before:
                    value = before[key]
                    after.set(key, self.correct_naming(value) if key == "material" else value)
                elif key == "rgba":
                    after.attrib.pop(key, None)

    @property
    def naming_prefix(self) -> str:
        return f"{self.idn}/"

    def set_joint_attribute(self, attrib: str, values: Any, force: bool = True) -> None:
        # Upstream's force=False calls add unrequested damping/armature. Native
        # MuJoCo defaults are intentional for this hardware-emulation model.
        if force:
            super().set_joint_attribute(attrib, values, force=True)
