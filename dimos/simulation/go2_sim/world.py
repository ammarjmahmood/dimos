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

"""The simulated world: a generated scene, the legged Go2 and its Mid-360, stepped at the policy rate.

Physics alone decides motion, contacts, falls and getting stuck. The module stands in
for the Go2 driver and PointLio: it takes cmd_vel and publishes PointLio's output
contract from ground truth, a 10 Hz lidar frame in the sensor frame deskewed to the
frame-end pose, odometry and tf for odom -> mid360_link. It runs paced to the wall
clock so the real modules around it run unmodified.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from queue import Empty, Queue
from threading import Event, Thread
import time

import mujoco
import mujoco.viewer
import numpy as np
from numpy.typing import NDArray
from pydantic import Field
from reactivex.disposable import Disposable
from scipy.spatial.transform import Rotation

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.LineSegments3D import LineSegments3D
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.sim_msgs.Contacts import Contact, Contacts
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.navigation.sim_eval.scenes import FAMILIES, Scene, generate
from dimos.robot.unitree.go2.go2_mid360_static_transforms import (
    CAMERA_XYZ,
    MID360_PITCH_DOWN,
    MID360_XYZ,
)
from dimos.simulation.go2_legged.policy import Go2Policy, OnnxGo2Policy
from dimos.simulation.go2_legged.robot import CONTROL_DT, LeggedGo2, apply_fitted_physics, go2_spec
from dimos.simulation.sensors.mid360.lidar import SimMid360
from dimos.simulation.sensors.mid360.pattern import POINT_RATE
from dimos.simulation.sensors.mujoco_raycaster import MujocoRaycaster
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

FRAME_DT = 0.1
TICKS_PER_FRAME = round(FRAME_DT / CONTROL_DT)
MOUNT_XYZ = np.add(CAMERA_XYZ, MID360_XYZ)
MOUNT_R = Rotation.from_euler("y", MID360_PITCH_DOWN).as_matrix()
LIDAR_HALF = (0.0325, 0.0325, 0.03)
SCENE_PUBLISH_DT = 2.0
COMMAND_TIMEOUT = 0.2
STILL = np.zeros(3)

BOX_EDGES = np.array(
    [
        [[-1, -1, -1], [1, -1, -1]],
        [[-1, 1, -1], [1, 1, -1]],
        [[-1, -1, 1], [1, -1, 1]],
        [[-1, 1, 1], [1, 1, 1]],
        [[-1, -1, -1], [-1, 1, -1]],
        [[1, -1, -1], [1, 1, -1]],
        [[-1, -1, 1], [-1, 1, 1]],
        [[1, -1, 1], [1, 1, 1]],
        [[-1, -1, -1], [-1, -1, 1]],
        [[1, -1, -1], [1, -1, 1]],
        [[-1, 1, -1], [-1, 1, 1]],
        [[1, 1, -1], [1, 1, 1]],
    ],
    dtype=np.float64,
)


@dataclass
class LidarFrame:
    """One 10 Hz frame: sensor-frame points at the frame-end pose, like PointLio's output."""

    t: float
    points: NDArray[np.float32]
    position: NDArray[np.float64]
    rotation: NDArray[np.float64]


def build_model(scene: Scene) -> mujoco.MjModel:
    """The scene's boxes, the Go2 and the Mid-360 housing as a contact box, in one model."""
    spec = go2_spec()
    for i, b in enumerate(scene.boxes):
        g = spec.worldbody.add_geom()
        g.type = mujoco.mjtGeom.mjGEOM_BOX
        g.name = f"{b.kind}_{i}"
        g.pos = b.center
        g.size = b.half
    lidar = spec.body("base").add_geom()
    lidar.name = "mid360"
    lidar.type = mujoco.mjtGeom.mjGEOM_BOX
    lidar.size = LIDAR_HALF
    lidar.pos = MOUNT_XYZ
    q = Rotation.from_matrix(MOUNT_R).as_quat()
    lidar.quat = (q[3], q[0], q[1], q[2])
    lidar.group = 3
    model = spec.compile()
    apply_fitted_physics(model)
    return model


def scene_edges(scene: Scene) -> NDArray[np.float64]:
    """The 12 edges of every box, as (N, 2, 3) segments."""
    centers = np.array([b.center for b in scene.boxes])[:, None, None, :]
    halves = np.array([b.half for b in scene.boxes])[:, None, None, :]
    edges: NDArray[np.float64] = (centers + BOX_EDGES[None] * halves).reshape(-1, 2, 3)
    return edges


class SimWorld:
    """The compiled scene with the Go2 and its Mid-360, ticked from a velocity command."""

    def __init__(self, scene: Scene, seed: int, policy: Go2Policy) -> None:
        self.scene = scene
        self.model = build_model(scene)
        self.data = mujoco.MjData(self.model)
        self.robot = LeggedGo2(self.model, self.data, policy)
        self.lidar = SimMid360.go2(MujocoRaycaster(self.model, self.data), seed)
        self.t = 0.0
        self._tick = 0
        self._frame_world: list[NDArray[np.float64]] = []
        self._points_per_step = int(POINT_RATE * self.model.opt.timestep)
        geom = mujoco.mjtObj.mjOBJ_GEOM
        self._kind = {
            mujoco.mj_name2id(self.model, geom, f"{b.kind}_{i}"): b.kind
            for i, b in enumerate(scene.boxes)
        }
        self._lidar_geom = mujoco.mj_name2id(self.model, geom, "mid360")

    def reset(self, x: float, y: float, z_feet: float, yaw: float) -> None:
        self.robot.reset(x, y, z_feet, yaw)
        self.t = 0.0
        self._tick = 0
        self._frame_world = []

    def sensor_pose(self) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        p, rot = self.robot.base_pose()
        return p + rot @ MOUNT_XYZ, rot @ MOUNT_R

    def base_pose(self) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        return self.robot.base_pose()

    def contacts(self) -> list[Contact]:
        """Every (robot part, scene kind) pair in contact right now, sorted."""
        m, d = self.model, self.data
        found: set[Contact] = set()
        for i in range(d.ncon):
            c = d.contact[i]
            for scene_geom, robot_geom in ((c.geom1, c.geom2), (c.geom2, c.geom1)):
                kind = self._kind.get(scene_geom)
                if kind is not None and m.geom_bodyid[robot_geom] != 0:
                    found.add(Contact(self._part(robot_geom), kind))
        return sorted(found)

    def _part(self, geom: int) -> str:
        if geom == self._lidar_geom:
            return "lidar"
        if geom in self.robot.feet:
            return "foot"
        if self.model.geom_bodyid[geom] == self.robot.trunk:
            return "trunk"
        return "leg"

    def tick(self, command: NDArray[np.float64]) -> LidarFrame | None:
        """Advance one policy tick. Returns the lidar frame that completed on this tick, if any."""
        dt = self.model.opt.timestep

        def cast(i: int) -> None:
            pos, rot = self.sensor_pose()
            scan = self.lidar.cast(pos, rot, self._points_per_step, self.t + (i + 1) * dt)
            self._frame_world.append(pos + scan.points.astype(np.float64) @ rot.T)

        self.robot.tick(command, cast)
        self._tick += 1
        self.t = self._tick * CONTROL_DT
        if self._tick % TICKS_PER_FRAME:
            return None
        pos, rot = self.sensor_pose()
        world = np.concatenate(self._frame_world) if self._frame_world else np.zeros((0, 3))
        self._frame_world = []
        return LidarFrame(self.t, ((world - pos) @ rot).astype(np.float32), pos, rot)


class SimGo2WorldConfig(ModuleConfig):
    family: str = "office"
    seed: int = 1
    frame_id: str = "odom"
    base_frame_id: str = "base_link"
    sensor_frame_id: str = "mid360_link"
    real_time_factor: float = Field(default=1.0, gt=0.0)
    mujoco_viewer: bool = False


@dataclass(frozen=True)
class _LoadScene:
    family: str
    seed: int


@dataclass(frozen=True)
class _Reset:
    pass


@dataclass(frozen=True)
class _SetPose:
    x: float
    y: float
    yaw: float


_Request = _LoadScene | _Reset | _SetPose


class SimGo2World(Module):
    """A generated scene with the legged Go2 and its simulated Mid-360, paced to the wall clock."""

    config: SimGo2WorldConfig

    cmd_vel: In[Twist]

    lidar: Out[PointCloud2]
    odometry: Out[Odometry]
    tf: Out[TFMessage]
    ground_truth: Out[PoseStamped]
    contacts: Out[Contacts]
    scene: Out[LineSegments3D]

    _thread: Thread | None = None
    _command: tuple[NDArray[np.float64], float] = (STILL, 0.0)

    @rpc
    def start(self) -> None:
        super().start()
        self._policy = OnnxGo2Policy.load()
        self._command = (STILL, 0.0)
        self._requests: Queue[_Request] = Queue()
        self._stop_event = Event()
        self.register_disposable(Disposable(self.cmd_vel.subscribe(self._on_cmd_vel)))
        self._thread = Thread(target=self._run, daemon=True)
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        super().stop()

    @rpc
    def load_scene(self, family: str, seed: int) -> None:
        """Replace the scene and put the robot at its start."""
        if family not in FAMILIES:
            raise ValueError(f"unknown scene family {family!r}, choose from {sorted(FAMILIES)}")
        self._requests.put(_LoadScene(family, seed))

    @rpc
    def reset(self) -> None:
        """Put the robot back at the scene's start, at rest."""
        self._requests.put(_Reset())

    @rpc
    def set_pose(self, x: float, y: float, yaw: float) -> None:
        """Place the robot standing on the floor at (x, y) facing yaw."""
        self._requests.put(_SetPose(x, y, yaw))

    def _on_cmd_vel(self, msg: Twist) -> None:
        command = np.array([msg.linear.x, msg.linear.y, msg.angular.z])
        if not all(math.isfinite(v) for v in command):
            logger.warning("Ignored non-finite cmd_vel", command=command.tolist())
            return
        self._command = (command, time.monotonic())

    def _current_command(self) -> NDArray[np.float64]:
        """The latest command, or still once it is older than the driver's timeout."""
        command, received = self._command
        return command if time.monotonic() - received < COMMAND_TIMEOUT else STILL

    def _load(self, family: str, seed: int) -> SimWorld:
        scene = generate(family, seed)
        world = SimWorld(scene, seed, self._policy)
        world.reset(*scene.start, 0.0)
        logger.info(
            "Sim world ready", scene=scene.name, goals={g.name: g.position for g in scene.goals}
        )
        return world

    def _open_viewer(self, world: SimWorld) -> mujoco.viewer.Handle | None:
        if not self.config.mujoco_viewer:
            return None
        viewer = mujoco.viewer.launch_passive(
            world.model, world.data, show_left_ui=False, show_right_ui=False
        )
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        viewer.cam.trackbodyid = world.robot.trunk
        viewer.cam.distance = 3.0
        viewer.cam.elevation = -25
        viewer.cam.azimuth = 135
        return viewer

    def _run(self) -> None:
        world = self._load(self.config.family, self.config.seed)
        # open3d loads on the first cloud, about a second
        PointCloud2.from_numpy(np.zeros((0, 3), np.float32), frame_id=self.config.sensor_frame_id)
        viewer = self._open_viewer(world)
        t0 = time.time()
        last_contacts: list[Contact] | None = None
        next_scene_publish = 0.0
        while not self._stop_event.is_set():
            try:
                request = self._requests.get_nowait()
            except Empty:
                pass
            else:
                if isinstance(request, _LoadScene):
                    try:
                        world = self._load(request.family, request.seed)
                    except Exception:
                        logger.exception("Scene load failed, keeping the current scene")
                        continue
                    if viewer is not None:
                        viewer.close()
                    viewer = self._open_viewer(world)
                elif isinstance(request, _Reset):
                    world.reset(*world.scene.start, 0.0)
                else:
                    world.reset(request.x, request.y, world.scene.start[2], request.yaw)
                self._command = (STILL, 0.0)
                t0 = time.time()
                last_contacts = None
                next_scene_publish = 0.0
            frame = world.tick(self._current_command())
            if viewer is not None:
                viewer.sync()
            stamp = t0 + world.t / self.config.real_time_factor
            self._publish_poses(world, stamp)
            contacts = world.contacts()
            if contacts != last_contacts:
                self.contacts.publish(Contacts(contacts, ts=stamp))
                last_contacts = contacts
            if frame is not None:
                self.lidar.publish(
                    PointCloud2.from_numpy(
                        frame.points, frame_id=self.config.sensor_frame_id, timestamp=stamp
                    )
                )
            if world.t >= next_scene_publish:
                self.scene.publish(
                    LineSegments3D(
                        ts=stamp, frame_id=self.config.frame_id, segments=scene_edges(world.scene)
                    )
                )
                next_scene_publish = world.t + SCENE_PUBLISH_DT
            delay = stamp - time.time()
            if delay > 0:
                self._stop_event.wait(delay)
            elif delay < -1.0:
                logger.warning("Sim running slower than real time", behind_s=round(-delay, 1))
                t0 -= delay
        if viewer is not None:
            viewer.close()

    def _publish_poses(self, world: SimWorld, stamp: float) -> None:
        pos, rot = world.sensor_pose()
        q = Rotation.from_matrix(rot).as_quat()
        sensor = Pose(*map(float, pos), *map(float, q))
        self.odometry.publish(
            Odometry(
                ts=stamp,
                frame_id=self.config.frame_id,
                child_frame_id=self.config.sensor_frame_id,
                pose=sensor,
            )
        )
        self.tf.publish(
            TFMessage(
                Transform(
                    translation=Vector3(sensor.position),
                    rotation=Quaternion(sensor.orientation),
                    frame_id=self.config.frame_id,
                    child_frame_id=self.config.sensor_frame_id,
                    ts=stamp,
                )
            )
        )
        base_pos, base_rot = world.base_pose()
        base_q = Rotation.from_matrix(base_rot).as_quat()
        self.ground_truth.publish(
            PoseStamped(
                *map(float, base_pos),
                *map(float, base_q),
                ts=stamp,
                frame_id=self.config.frame_id,
            )
        )
