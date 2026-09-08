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

"""Experimental robosuite stepping/observables over a native DimOS robot model.

This deliberately does not claim RobotEnv/controller or task compatibility.
Model composition is the existing sim2 implementation, not a new importer.
"""

from collections import OrderedDict
from copy import deepcopy
from typing import Any, cast

import mujoco
import numpy as np
from numpy.typing import NDArray
from robosuite.environments.base import MujocoEnv
from robosuite.utils.binding_utils import MjSim
from robosuite.utils.camera_utils import get_real_depth_map
from robosuite.utils.observables import Observable, sensor
from scipy.spatial.transform import Rotation

from dimos.hardware.spec import JointLimits
from dimos.hardware.whole_body.spec import POS_STOP, VEL_STOP, IMUState, MotorCommand, MotorState
from dimos.sim2.scene import load_scene
from dimos.sim2.sensors.lidar.raycast import Raycaster
from dimos.sim2.sensors.spec import Camera, Imu, Lidar
from dimos.sim2.spec import ControlInterface, WorldConfig


class MotorEnvironment(MujocoEnv):
    """One floating effort-actuated robot using DimOS motor units and commands."""

    def __init__(self, world: WorldConfig, *, sense: bool = True) -> None:
        if len(world.robots) != 1:
            raise ValueError("this experiment covers one robot")
        self.world = world
        self.robot_id, instance = next(iter(world.robots.items()))
        self.definition = instance.config
        if self.definition.control != ControlInterface.WHOLE_BODY or any(
            j.mode != "effort" or j.scale != 1 or j.offset != 0 for j in self.definition.joints
        ):
            raise ValueError("the motor probe requires whole-body effort joints in native units")
        self.sense = sense
        self.captures: dict[str, int] = {}
        self.capture_times: dict[str, list[float]] = {}
        self._connected = True
        self._commands = np.array(
            [[j.home, 0, j.kp, j.kd, 0] for j in self.definition.joints], dtype=float
        )
        super().__init__(
            has_renderer=False,
            has_offscreen_renderer=sense
            and any(isinstance(s, Camera) for s in self.definition.sensors),
            renderer="mujoco",
            render_collision_mesh=True,
            control_freq=50,
            lite_physics=True,
            ignore_done=True,
            hard_reset=False,
            seed=0,
        )

    def _load_model(self) -> None:
        self.model = load_scene(self.world)

    def _initialize_sim(self, xml_string: str | None = None) -> None:
        if xml_string is not None:
            raise ValueError("reload the WorldConfig; XML replay is outside this experiment")
        self.sim = MjSim(self.model)
        self.sim.forward()
        self.initialize_time(self.control_freq)

    def initialize_time(self, control_freq: float) -> None:
        super().initialize_time(control_freq)
        # Upstream uses a global timestep here, not the loaded model's timestep.
        self.model_timestep = self.world.timestep
        substeps = self.control_timestep / self.model_timestep
        if not np.isclose(substeps, round(substeps)):
            raise ValueError("control period must contain a whole number of physics steps")

    def _setup_references(self) -> None:
        prefix = self.robot_id + "/"
        joints = [self.model.joint(prefix + j.model_name).id for j in self.definition.joints]
        self.qpos_ids = self.model.jnt_qposadr[joints]
        self.dof_ids = self.model.jnt_dofadr[joints]
        self.actuator_ids = np.array(
            [self.model.actuator(prefix + j.actuator).id for j in self.definition.joints]
        )
        self.root_id = self.model.body(prefix + self.definition.root_body).id
        imu = next(s for s in self.definition.sensors if isinstance(s, Imu))
        self.imu_addresses = tuple(
            int(self.model.sensor(f"{prefix}sensor/{imu.name}/{suffix}").adr[0])
            for suffix in ("gyro", "accel", "quat")
        )

    def _reset_internal(self) -> None:
        super()._reset_internal()
        self.sim.data.qpos[self.qpos_ids] = [j.home for j in self.definition.joints]
        self._commands[:, 0] = [j.home for j in self.definition.joints]
        self._commands[:, 1] = 0
        self._commands[:, 2] = [j.kp for j in self.definition.joints]
        self._commands[:, 3] = [j.kd for j in self.definition.joints]
        self._commands[:, 4] = 0
        self.sim.forward()
        self.captures.clear()
        self.capture_times.clear()

    def visualize(self, vis_settings: dict[str, bool]) -> None:
        # No robosuite Task object wrappers exist around this compiled scene.
        self.sim.model.site_rgba[:, 3] = 0

    @property
    def action_dim(self) -> int:
        return 5 * len(self.definition.joints)

    @property
    def action_spec(self) -> tuple[NDArray, NDArray]:
        return np.full(self.action_dim, -np.inf), np.full(self.action_dim, np.inf)

    def _pre_action(self, action: NDArray, policy_step: bool = False) -> None:
        q, dq, kp, kd, tau = np.asarray(action).reshape(-1, 5).T
        if len(q) != len(self.definition.joints):
            raise ValueError("wrong motor count")
        torque = kp * (q - self.sim.data.qpos[self.qpos_ids])
        torque += kd * (dq - self.sim.data.qvel[self.dof_ids]) + tau
        limits = self.model.actuator_ctrlrange[self.actuator_ids]
        limited = self.model.actuator_ctrllimited[self.actuator_ids].astype(bool)
        self.sim.data.ctrl[self.actuator_ids] = np.where(
            limited, np.clip(torque, limits[:, 0], limits[:, 1]), torque
        )

    def reward(self, action: NDArray | None = None) -> float:
        return 0.0

    def advance(self) -> OrderedDict[str, Any]:
        observations, _, _, _ = self.step(self._commands.ravel())
        return cast("OrderedDict[str, Any]", observations)

    def connect(self) -> bool:
        return self._connected

    def disconnect(self) -> None:
        self._connected = False

    def is_connected(self) -> bool:
        return self._connected

    def has_motor_states(self) -> bool:
        return self._connected

    def get_limits(self) -> JointLimits | None:
        return None

    def read_motor_states(self) -> list[MotorState]:
        return [
            MotorState(*values)
            for values in zip(
                self.sim.data.qpos[self.qpos_ids],
                self.sim.data.qvel[self.dof_ids],
                self.sim.data.qfrc_actuator[self.dof_ids],
                strict=True,
            )
        ]

    def read_imu(self) -> IMUState:
        g, a, r = self.imu_addresses
        values = self.sim.data.sensordata
        quat = tuple(values[r : r + 4])
        return IMUState(
            quaternion=quat,
            gyroscope=tuple(values[g : g + 3]),
            accelerometer=tuple(values[a : a + 3]),
            rpy=tuple(Rotation.from_quat(quat, scalar_first=True).as_euler("xyz")),
        )

    def write_motor_commands(self, commands: list[MotorCommand]) -> bool:
        values = np.array([[c.q, c.dq, c.kp, c.kd, c.tau] for c in commands])
        if values.shape != self._commands.shape or not np.isfinite(values).all():
            raise ValueError("expected one finite command per motor")
        values[values[:, 0] == POS_STOP, 2] = 0
        values[values[:, 0] == POS_STOP, 0] = 0
        values[values[:, 1] == VEL_STOP, 3] = 0
        values[values[:, 1] == VEL_STOP, 1] = 0
        self._commands[:] = values
        return self._connected

    def _captured(self, name: str) -> None:
        self.captures[name] = self.captures.get(name, 0) + 1
        self.capture_times.setdefault(name, []).append(float(self.sim.data.time))

    def _setup_observables(self) -> OrderedDict[str, Observable]:
        observables: OrderedDict[str, Observable] = super()._setup_observables()
        if self.sense:
            for device in self.definition.sensors:
                if isinstance(device, Camera):
                    observables.update(self._camera(device))
                elif isinstance(device, Lidar):
                    name = device.name + "_points"
                    observables[name] = Observable(
                        name, self._lidar(device), sampling_rate=device.rate_hz
                    )
        return observables

    def _camera(self, device: Camera) -> dict[str, Observable]:
        camera_name = f"{self.robot_id}/sensor/{device.name}"
        depth_name = device.name + "_depth"

        @sensor(modality="image")  # type: ignore[untyped-decorator]
        def rgb(cache: dict[str, Any]) -> NDArray:
            self._captured(device.name)
            result = self.sim.render(
                width=device.width,
                height=device.height,
                camera_name=camera_name,
                depth=device.depth,
            )
            if device.depth:
                image, depth = result
                cache[depth_name] = get_real_depth_map(self.sim, depth)[::-1].copy()
                return image[::-1].copy()
            return result[::-1].copy()

        @sensor(modality="image")  # type: ignore[untyped-decorator]
        def depth(cache: dict[str, Any]) -> NDArray:
            return cache.get(depth_name, np.zeros((device.height, device.width)))

        name = device.name + "_image"
        result = {name: Observable(name, rgb, sampling_rate=device.rate_hz)}
        if device.depth:
            result[depth_name] = Observable(depth_name, depth, sampling_rate=device.rate_hz)
        return result

    def _lidar(self, device: Lidar) -> Any:
        site = self.model.site(f"{self.robot_id}/sensor/{device.name}").id
        # Existing raycaster changes geometry groups to exclude the whole robot.
        # Preserve its private-model ownership; never change live render groups.
        ray_model = deepcopy(self.model)
        ray_data = mujoco.MjData(ray_model)
        state_kind = mujoco.mjtState.mjSTATE_INTEGRATION
        state = np.empty(mujoco.mj_stateSize(self.model, state_kind))
        caster = Raycaster(ray_model, self.root_id)
        directions = device.model.directions()

        @sensor(modality="lidar")  # type: ignore[untyped-decorator]
        def points(cache: dict[str, Any]) -> NDArray:
            self._captured(device.name)
            data = self.sim.data._data
            mujoco.mj_getState(self.model, data, state, state_kind)
            mujoco.mj_setState(ray_model, ray_data, state, state_kind)
            mujoco.mj_forward(ray_model, ray_data)
            origin = data.site_xpos[site]
            rotation = data.site_xmat[site].reshape(3, 3)
            rays = directions @ rotation.T
            if device.maximum_world_elevation is not None:
                limit = np.sin(np.deg2rad(device.maximum_world_elevation))
                rays = rays[rays[:, 2] <= limit + 1e-12]
            cloud = caster.cast(
                ray_data, origin, rays, device.model.min_range, device.model.max_range
            )
            return (cloud - origin) @ rotation if device.output_frame == "sensor" else cloud

        return points
