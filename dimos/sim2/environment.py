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
from typing import Any
import xml.etree.ElementTree as ET

from robosuite.environments.manipulation.manipulation_env import ManipulationEnv
from robosuite.models.tasks import Task
from robosuite.robots import ROBOT_CLASS_MAPPING
from robosuite.utils.mjcf_utils import array_to_string

from dimos.sim2.models import SceneModel
from dimos.sim2.robot import MotorRobot, motor_controller_config
from dimos.sim2.scene import quaternion
from dimos.sim2.scene_types import SceneDescription, SceneEntity
from dimos.sim2.sensors.spec import Camera, Imu
from dimos.sim2.spec import WorldConfig


class EmulatorEnvironment(ManipulationEnv):  # type: ignore[misc]  # Upstream is untyped.
    def __init__(self, config: WorldConfig, description: SceneDescription) -> None:
        self.config = config
        self.description = description
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
        )
        self.deterministic_reset = True

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

    def _load_model(self) -> None:
        super()._load_model()
        arena = SceneModel(str(self.config.scene))
        for robot_id, instance in self.config.robots.items():
            robot = self.robot_by_id[robot_id]
            model = robot.robot_model
            root = model.worldbody.find("body")
            if root is None or model.root_body != model.correct_naming(instance.config.root_body):
                raise ValueError(f"{robot_id}: model root disagrees with the device definition")
            root.set("pos", array_to_string(instance.xyz))
            root.set("quat", array_to_string(quaternion(instance.rpy)))
            if not instance.config.floating:
                root.set("mocap", "true")
            keyframes = model.root.find("keyframe")
            if keyframes is not None:
                model.root.remove(keyframes)
            for sensor in instance.config.sensors:
                body = model.worldbody.find(
                    f".//body[@name='{model.correct_naming(sensor.mount.link)}']"
                )
                if body is None:
                    raise ValueError(
                        f"{robot_id}/{sensor.name}: unknown mount {sensor.mount.link!r}"
                    )
                name = f"{robot_id}/sensor/{sensor.name}"
                attrs = {
                    "name": name,
                    "pos": array_to_string(sensor.mount.xyz),
                    "quat": array_to_string(quaternion(sensor.mount.rpy)),
                }
                if isinstance(sensor, Camera):
                    ET.SubElement(body, "camera", **attrs, fovy=str(sensor.fovy))
                else:
                    ET.SubElement(body, "site", **attrs, size=".001", rgba="0 0 0 0")
                    if isinstance(sensor, Imu):
                        ET.SubElement(model.sensor, "gyro", name=name + "/gyro", site=name)
                        ET.SubElement(
                            model.sensor, "accelerometer", name=name + "/accel", site=name
                        )
                        ET.SubElement(
                            model.sensor,
                            "framequat",
                            name=name + "/quat",
                            objtype="site",
                            objname=name,
                        )

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
        option = self.model.root.find("option")
        if option is None:
            option = ET.SubElement(self.model.root, "option")
        option.set("timestep", str(self.config.timestep))
        option.set("integrator", "implicitfast")
        for geom in self.model.worldbody.iter("geom"):
            if int(geom.get("group", "0")) in self.description.hidden_geom_groups:
                rgba = geom.get("rgba", ".5 .5 .5 1").split()
                rgba[3] = "0"
                geom.set("rgba", " ".join(rgba))

    def initialize_time(self, control_freq: float) -> None:
        self.cur_time = 0.0
        # Do not use robosuite's process-global default 2 ms physics timestep.
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

    def reward(self, action: Any = None) -> float:
        return 0.0
