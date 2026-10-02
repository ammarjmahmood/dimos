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

"""Robot-model point-cloud self exclusion and map-clear-mask generation."""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from functools import partial
from io import BytesIO
from threading import RLock
from typing import Any
import xml.etree.ElementTree as ET

import numpy as np
from numpy.typing import NDArray
from pydantic import Field
import trimesh
import yourdfpy  # type: ignore[import-untyped]

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.tf.tf import TF
from dimos.robot.assets.model import RobotModel
from dimos.types.timestamped import TimestampedBufferCollection
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import matrix_to_pose

logger = setup_logger()


@dataclass(frozen=True)
class _CollisionGeometry:
    link: str
    link_from_geometry: NDArray[np.float64]
    clear_samples: NDArray[np.float64]


class PointCloudSelfFilterConfig(ModuleConfig):
    model: RobotModel
    padding_m: float = Field(default=0.01, ge=0.0)
    # Must match the mapper's voxel_size, or the mask names cells the map does
    # not hold and clears nothing.
    voxel_size: float = Field(default=0.05, gt=0.0)
    world_frame: str = "world"
    tf_tolerance_s: float = Field(default=0.02, ge=0.0)
    tf_forward_tolerance_s: float = Field(default=0.05, ge=0.0)
    state_tolerance_s: float = Field(default=0.02, ge=0.0)
    state_history_s: float = Field(default=5.0, gt=0.0)


class PointCloudSelfFilter(Module):
    """Remove the modeled robot from a cloud and emit its map clear mask.

    A wrist camera sees its own arm. Two things follow, and this module does
    both: the arm's returns must not enter the map, and the volume the arm
    occupies must be erased from it. Ray tracing cannot do the second - the arm
    occludes whatever is behind it, so no ray ever passes through that volume to
    clear it - so the mask says outright which cells are free.
    """

    config: PointCloudSelfFilterConfig

    pointcloud: In[PointCloud2]
    tf: In[TFMessage]
    coordinator_joint_state: In[JointState]
    filtered_pointcloud: Out[PointCloud2]
    voxel_clear_mask: Out[PointCloud2]

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._filter_lock = RLock()
        self._states = TimestampedBufferCollection[JointState](self.config.state_history_s)
        self._collision_geometry = self._load_collision_geometry()
        if not self._collision_geometry:
            raise ValueError("Robot model contains no collision geometry")
        self._previous_clear_keys: set[tuple[int, int, int]] = set()
        self._last_capture: float | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self._tf = TF(self.tf)

    @rpc
    def stop(self) -> None:
        super().stop()

    async def handle_pointcloud(self, cloud: PointCloud2) -> None:
        """Filter the latest capture without starving TF transport callbacks."""
        await asyncio.to_thread(self._on_pointcloud, cloud)

    def filter_cloud(self, cloud: PointCloud2) -> tuple[PointCloud2, PointCloud2] | None:
        """Filter one capture and build the matching world-frame clear mask."""
        # One native filter owns mutable scratch; history must commit in the
        # same critical section so callbacks cannot reorder the clear masks.
        with self._filter_lock:
            return self._filter_capture(cloud)

    async def handle_coordinator_joint_state(self, state: JointState) -> None:
        await asyncio.to_thread(self.add_joint_state, state)

    def add_joint_state(self, state: JointState) -> None:
        """Buffer full model state; out-of-order arrivals remain timestamped."""
        with self._filter_lock:
            if np.isfinite(state.ts):
                self._states.remove_by_timestamp(state.ts)
                self._states.add(JointState(state))
                latest = self._states.last()
                if latest is not None:
                    self._states.prune_old(latest.ts - self.config.state_history_s)

    def _filter_capture(self, cloud: PointCloud2) -> tuple[PointCloud2, PointCloud2] | None:
        config = self.config
        if not np.isfinite(cloud.ts) or (
            self._last_capture is not None and cloud.ts < self._last_capture
        ):
            logger.warning("Dropping cloud: invalid or out-of-order capture timestamp")
            return None
        base_from_sensor = self._lookup(self._base_link, cloud.frame_id, cloud.ts)
        world_from_base = self._lookup(config.world_frame, self._base_link, cloud.ts)
        q = self._capture_configuration(cloud.ts)
        if base_from_sensor is None or world_from_base is None or q is None:
            logger.warning("Dropping cloud: capture-time robot state or TF unavailable")
            return None
        points = cloud.points_f32()
        base_points = _transform_points(points, base_from_sensor.to_matrix())
        keep = ~np.asarray(self._body_filter.computeMask(q, base_points), dtype=bool)
        world_candidates = []
        for geometry in self._collision_geometry:
            base_from_link = np.asarray(self._context.forwardKinematics(q, geometry.link))
            world_from_geometry = (
                world_from_base.to_matrix() @ base_from_link @ geometry.link_from_geometry
            )
            world_candidates.append(_transform_points(geometry.clear_samples, world_from_geometry))
        # Keep volume sampling for map clearing: Coal mesh queries classify
        # proximity to triangle surfaces, not the interior of a closed mesh.
        keys = np.floor(np.concatenate(world_candidates) / config.voxel_size).astype(np.int64)
        current_clear_keys = set(map(tuple, keys.tolist()))

        # Where the arm was plus where it is: a link that moved between frames
        # leaves a ghost behind it that nothing else will ever clear.
        clear_keys = self._previous_clear_keys | current_clear_keys
        self._previous_clear_keys = current_clear_keys
        self._last_capture = cloud.ts
        clear_points = (
            np.asarray(sorted(clear_keys), dtype=np.float32).reshape((-1, 3)) + 0.5
        ) * config.voxel_size
        clear_mask = PointCloud2.from_numpy(
            clear_points,
            frame_id=config.world_frame,
            timestamp=cloud.ts,
        )

        intensities = cloud.intensities_f32()
        filtered = PointCloud2.from_numpy(
            points[keep],
            frame_id=cloud.frame_id,
            timestamp=cloud.ts,
            intensities=intensities[keep] if intensities is not None else None,
        )
        for name, values in cloud.pointcloud_tensor.point.items():
            if name not in ("positions", "intensities"):
                filtered.pointcloud_tensor.point[name] = values[keep]
        return filtered, clear_mask

    def _lookup(self, parent_frame: str, child_frame: str, stamp: float) -> Transform | None:
        config = self.config
        return self.tfbuffer.get(
            parent_frame,
            child_frame,
            time_point=stamp,
            time_tolerance=config.tf_tolerance_s,
            forward_tolerance=config.tf_forward_tolerance_s,
        )

    def _on_pointcloud(self, cloud: PointCloud2) -> None:
        with self._filter_lock:
            self._publish_capture(cloud)

    def _publish_capture(self, cloud: PointCloud2) -> None:
        result = self.filter_cloud(cloud)
        if result is None:
            return
        filtered, clear_mask = result
        # The mask is independent authoritative free-space evidence. Publishing
        # it first minimizes cleanup latency without making cloud processing
        # depend on cross-topic ordering.
        self.voxel_clear_mask.publish(clear_mask)
        self.filtered_pointcloud.publish(filtered)

    def _capture_configuration(self, stamp: float) -> NDArray[np.float64] | None:
        state = self._states.find_closest(stamp, self.config.state_tolerance_s)
        positions: dict[str, float] = {}
        if state is not None:
            if len(state.name) != len(state.position) or len(set(state.name)) != len(state.name):
                return None
            positions = dict(zip(state.name, state.position, strict=True))
            if not np.isfinite(list(positions.values())).all():
                return None
        # Start from the model's neutral configuration, never its latest state.
        q = self._neutral_q.copy()
        for name in self._scene.getJointNames():
            info = self._scene.getJointInfo(name)
            if info.num_velocity_dofs == 1:
                if name not in positions:
                    return None
                value = positions[name]
                values = [np.cos(value), np.sin(value)] if info.num_position_dofs == 2 else [value]
            else:
                # Multi-DOF joints have no scalar JointState representation.
                # Recover their configuration from capture-time relative TF.
                joint = self._joints[name]
                parent_from_child = self._lookup(joint.parent, joint.child, stamp)
                if parent_from_child is None:
                    return None
                origin = np.eye(4) if joint.origin is None else joint.origin
                motion = np.linalg.inv(origin) @ parent_from_child.to_matrix()
                pose = matrix_to_pose(motion)
                if info.num_position_dofs == 4:
                    if not np.allclose(motion[2], [0, 0, 1, 0], atol=1e-6):
                        return None
                    angle = np.arctan2(motion[1, 0], motion[0, 0])
                    values = [motion[0, 3], motion[1, 3], np.cos(angle), np.sin(angle)]
                elif info.num_position_dofs == 7:
                    values = [
                        *motion[:3, 3],
                        pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z,
                        pose.orientation.w,
                    ]
                else:
                    raise ValueError(f"Unsupported robot joint layout: {name}")
            q[self._scene.getJointPositionIndices([name])] = values
        return q

    def _load_collision_geometry(self) -> list[_CollisionGeometry]:
        # RoboPlan is optional for stacks that do not use this module.
        import roboplan.core as native

        description = self.config.model.load()
        root = ET.fromstring(description.xml)
        root.set("version", "1.0")
        self._scene = native.Scene(
            "dimos_self_filter",
            native.loadUrdfSceneDescriptionFromXml(
                ET.tostring(root, encoding="unicode"),
                [str(path) for path in description.package_paths.values()],
            ),
        )
        self._neutral_q = np.asarray(self._scene.getCurrentJointPositions()).copy()
        self._context = native.SceneContext(self._scene)
        self._body_filter = native.RobotBodyFilter(
            self._scene,
            native.RobotBodyFilterOptions(
                padding=self.config.padding_m,
                method=native.RobotBodyFilterMethod.Narrowphase,
                num_threads=1,
            ),
        )
        mesh_dir = str(description.source_path.parent)
        robot = yourdfpy.URDF.load(
            BytesIO(description.xml.encode()),
            build_scene_graph=False,
            build_collision_scene_graph=False,
            load_meshes=False,
            load_collision_meshes=False,
        )
        child_links = {joint.child for joint in robot.robot.joints}
        roots = [link.name for link in robot.robot.links if link.name not in child_links]
        if len(roots) != 1:
            raise ValueError("Robot model must have one root link")
        self._base_link = roots[0]
        self._joints = {joint.name: joint for joint in robot.robot.joints}
        resolve = partial(yourdfpy.filename_handler_magic, dir=mesh_dir)
        result: list[_CollisionGeometry] = []
        for link in robot.robot.links:
            for collision in link.collisions:
                shape = _geometry_mesh(collision.geometry, resolve)
                if shape is None:
                    continue
                mesh, shape_name, dimensions = shape
                result.append(
                    _CollisionGeometry(
                        link=link.name,
                        link_from_geometry=(
                            np.eye(4, dtype=np.float64)
                            if collision.origin is None
                            else np.asarray(collision.origin, dtype=np.float64)
                        ),
                        clear_samples=self._clear_samples(mesh, shape_name, dimensions),
                    )
                )
        return result

    def _clear_samples(
        self, mesh: trimesh.Trimesh, shape: str, dimensions: tuple[float, ...]
    ) -> NDArray[np.float64]:
        """Grid points covering the geometry, at map resolution.

        A cell whose center is outside the shape can still be occupied by it, so
        the margin reaches out by half a cell diagonal.
        """
        pitch = self.config.voxel_size
        margin = self.config.padding_m + (np.sqrt(3.0) * pitch / 2.0)
        lower = np.floor((mesh.bounds[0] - margin) / pitch).astype(int)
        upper = np.ceil((mesh.bounds[1] + margin) / pitch).astype(int)
        axes = [
            np.arange(lo, hi + 1, dtype=np.float64) * pitch
            for lo, hi in zip(lower, upper, strict=True)
        ]
        grid = np.stack(np.meshgrid(*axes, indexing="ij"), axis=-1).reshape((-1, 3))
        inside = _clear_volume_mask(grid, shape, dimensions, mesh, margin)
        return np.asarray(grid[inside], dtype=np.float64)


def _geometry_mesh(
    geometry: Any,
    resolve: Any,
) -> tuple[trimesh.Trimesh, str, tuple[float, ...]] | None:
    if geometry.box is not None:
        size = tuple(float(value) for value in geometry.box.size)
        return trimesh.creation.box(extents=size), "box", size
    if geometry.sphere is not None:
        radius = float(geometry.sphere.radius)
        return trimesh.creation.icosphere(radius=radius), "sphere", (radius,)
    if geometry.cylinder is not None:
        radius = float(geometry.cylinder.radius)
        length = float(geometry.cylinder.length)
        return (
            trimesh.creation.cylinder(radius=radius, height=length),
            "cylinder",
            (radius, length),
        )
    if geometry.mesh is None:
        return None
    filename = resolve(geometry.mesh.filename)
    loaded = trimesh.load_mesh(filename, force="mesh")
    if not isinstance(loaded, trimesh.Trimesh):
        raise ValueError(f"Collision mesh is not a single mesh: {filename}")
    mesh = loaded.copy()
    if geometry.mesh.scale is not None:
        mesh.apply_scale(  # type: ignore[no-untyped-call]
            np.asarray(geometry.mesh.scale, dtype=np.float64)
        )
    return mesh, "mesh", ()


def _clear_volume_mask(
    points: NDArray[np.float64],
    shape: str,
    dimensions: tuple[float, ...],
    mesh: trimesh.Trimesh,
    padding: float,
) -> NDArray[np.bool_]:
    """Precompute occupied volume samples for clearing, never classify sensor points."""
    if shape == "box":
        half_size = np.asarray(dimensions, dtype=np.float64) / 2.0
        return np.asarray(np.all(np.abs(points) <= half_size + padding, axis=1))
    if shape == "sphere":
        radius = dimensions[0] + padding
        return np.asarray(np.einsum("ij,ij->i", points, points) <= radius**2)
    if shape == "cylinder":
        radius, length = dimensions
        radial_sq = np.einsum("ij,ij->i", points[:, :2], points[:, :2])
        return np.asarray(
            (radial_sq <= (radius + padding) ** 2)
            & (np.abs(points[:, 2]) <= length / 2.0 + padding)
        )

    # Reject candidates outside the padded bounds before mesh volume sampling.
    padded_lower = mesh.bounds[0] - padding
    padded_upper = mesh.bounds[1] + padding
    candidates = np.all((points >= padded_lower) & (points <= padded_upper), axis=1)
    inside = np.zeros(len(points), dtype=bool)
    if np.any(candidates):
        signed_distance = trimesh.proximity.signed_distance(  # type: ignore[no-untyped-call]
            mesh, points[candidates]
        )
        inside[candidates] = signed_distance >= -padding
    return inside


def _transform_points(
    points: NDArray[np.float32] | NDArray[np.float64], transform: NDArray[np.float64]
) -> NDArray[np.float64]:
    if not len(points):
        return np.empty((0, 3), dtype=np.float64)
    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    return np.asarray(points @ rotation.T + translation, dtype=np.float64)
