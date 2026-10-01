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

"""Continuous hardware emulation using robosuite model and runtime ownership."""

from __future__ import annotations

from copy import deepcopy
from typing import Any, cast
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from numpy.typing import NDArray
from robosuite.environments.manipulation.manipulation_env import ManipulationEnv
from robosuite.models.tasks import Task
from robosuite.robots import ROBOT_CLASS_MAPPING
from robosuite.utils.mjcf_utils import array_to_string

from dimos.sim2.models import SceneModel
from dimos.sim2.robot import MotorRobot, motor_controller_config
from dimos.sim2.scene import describe_scene, quaternion
from dimos.sim2.scene_types import SceneDescription, SceneEntity
from dimos.sim2.sensors.spec import Camera
from dimos.sim2.spec import RobosuiteTask, WorldConfig


class DeviceEnvironment(ManipulationEnv):  # type: ignore[misc]  # Upstream is untyped.
    """Common devices and timing around either an authored world or upstream task."""

    description: SceneDescription

    def __init__(self, config: WorldConfig, **parameters: Any) -> None:
        self.config = config
        super().__init__(
            robots=[value.config.model.__name__ for value in config.robots.values()],
            controller_configs=[
                motor_controller_config(value.config) for value in config.robots.values()
            ],
            base_types="NullBase",
            use_camera_obs=False,
            has_renderer=False,
            has_offscreen_renderer=False,
            renderer="mujoco",
            render_camera=None,
            control_freq=1 / config.timestep,
            hard_reset=False,
            ignore_done=True,
            lite_physics=False,
            **parameters,
        )
        if isinstance(config.scene, RobosuiteTask):
            self.description = SceneDescription(
                id=f"robosuite:{config.scene.environment.__name__}",
                entities={
                    obj.name: SceneEntity(
                        body=obj.root_body,
                        label=obj.name,
                        kind="object",
                        movable=obj.get_obj().find("joint[@type='free']") is not None,
                    )
                    for obj in self.model.mujoco_objects
                },
            )

    @property
    def robot_by_id(self) -> dict[str, MotorRobot]:
        return dict(zip(self.config.robots, self.robots, strict=True))

    def _load_robots(self) -> None:
        # Keep public DimOS instance namespaces rather than upstream's numeric IDs.
        for index, ((robot_id, instance), robot_config) in enumerate(
            zip(self.config.robots.items(), self.robot_configs, strict=True)
        ):
            self.robots[index] = ROBOT_CLASS_MAPPING[instance.config.model.__name__](
                robot_type=instance.config.model.__name__, idn=robot_id, **robot_config
            )
            self.robots[index].load_model()

    def _initialize_sim(self, xml_string: str | None = None) -> None:
        option = self.model.root.find("option")
        if option is None:
            option = ET.SubElement(self.model.root, "option")
        option.set("timestep", str(self.config.timestep))
        option.set("integrator", "implicitfast")
        super()._initialize_sim(xml_string)

    def initialize_time(self, control_freq: float) -> None:
        self.cur_time = 0.0
        self.model_timestep = self.config.timestep
        self.control_timestep = self.config.timestep

    def _setup_observables(self) -> dict[str, Any]:
        observables: dict[str, Any] = super()._setup_observables()
        for observable in observables.values():
            observable.set_sampling_rate(self.config.snapshot_hz)
        return observables

    def _reset_internal(self) -> None:
        super()._reset_internal()
        cameras = [
            s
            for r in self.config.robots.values()
            for s in r.config.sensors
            if isinstance(s, Camera)
        ]
        self.sim.model.vis.global_.offwidth = max([640, *[s.width for s in cameras]])
        self.sim.model.vis.global_.offheight = max([480, *[s.height for s in cameras]])


class EmulatorEnvironment(DeviceEnvironment):
    """Authored MJCF worlds reset to their explicitly captured initial state."""

    def __init__(self, config: WorldConfig) -> None:
        if isinstance(config.scene, RobosuiteTask):
            raise TypeError("an authored environment requires an MJCF path")
        self.description = describe_scene(config.scene)
        self._baseline: NDArray[np.float64] | None = None
        super().__init__(config)
        self.deterministic_reset = True

    def capture_initial_state(self) -> None:
        model, data = self.sim.model._model, self.sim.data._data
        kind = mujoco.mjtState.mjSTATE_INTEGRATION
        self._baseline = np.empty(mujoco.mj_stateSize(model, kind))
        mujoco.mj_getState(model, data, self._baseline, kind)

    def _reset_internal(self) -> None:
        super()._reset_internal()
        if self._baseline is not None:
            mujoco.mj_setState(
                self.sim.model._model,
                self.sim.data._data,
                self._baseline,
                mujoco.mjtState.mjSTATE_INTEGRATION,
            )

    def _load_model(self) -> None:
        super()._load_model()
        arena = SceneModel(str(self.config.scene))
        for robot_id, instance in self.config.robots.items():
            robot = self.robot_by_id[robot_id]
            model = robot.robot_model
            root = model.worldbody.find("body")
            assert root is not None  # Validated by MotorRobot.load_model.
            root.set("pos", array_to_string(instance.xyz))
            root.set("quat", array_to_string(quaternion(instance.rpy)))

        objects = []
        for item in self.config.objects:
            if item.name in self.description.entities:
                raise ValueError(f"object {item.name!r} conflicts with an authored scene entity")
            obj = item.model(name=item.name, **item.parameters)
            body = obj.get_obj()
            body.set("pos", array_to_string(item.xyz))
            self.description.entities[item.name] = SceneEntity(
                body=obj.root_body,
                label=item.name,
                kind="object",
                movable=body.find("joint[@type='free']") is not None,
            )
            objects.append(obj)
        self.model = Task(arena, [robot.robot_model for robot in self.robots], objects)
        # Task merges model components, not world-level solver/render settings.
        for tag in ("compiler", "option", "visual", "size", "statistic"):
            original = arena.root.find(tag)
            current = self.model.root.find(tag)
            if current is not None:
                self.model.root.remove(current)
            if original is not None:
                self.model.root.append(deepcopy(original))
        for geom in self.model.worldbody.iter("geom"):
            if int(geom.get("group", "0")) in self.description.hidden_geom_groups:
                rgba = geom.get("rgba", ".5 .5 .5 1").split()
                rgba[3] = "0"
                geom.set("rgba", " ".join(rgba))

    def reward(self, action: Any = None) -> float:
        return 0.0


def create_environment(config: WorldConfig) -> DeviceEnvironment:
    if isinstance(config.scene, RobosuiteTask):
        upstream = config.scene.environment
        environment = type(f"DimOS{upstream.__name__}", (DeviceEnvironment, upstream), {})
        return cast("DeviceEnvironment", environment(config, **config.scene.parameters))
    return EmulatorEnvironment(config)
