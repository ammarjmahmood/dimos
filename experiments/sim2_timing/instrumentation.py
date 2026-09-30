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

"""Experiment-local probes. Production loops, policies and IPC remain unchanged."""

from __future__ import annotations

from collections import defaultdict
from collections.abc import Callable
import hashlib
import os
import time
from typing import Any

import mujoco
import numpy as np
from numpy.typing import NDArray
import psutil

from dimos.control.coordinator import ControlCoordinator
from dimos.core.core import rpc
from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.sim2.connections.whole_body import WholeBodyConnection
from dimos.sim2.module import SimulationModule
from dimos.sim2.sensors.camera.module import SimRGBDCameraModule
from dimos.sim2.sensors.lidar.module import LidarModule
from dimos.sim2.sensors.lidar.raycast import Raycaster
from dimos.simulation.engines import mujoco_sim_module as old_module
from dimos.simulation.engines.mujoco_engine import MujocoEngine, _camera_ray_directions
from dimos.simulation.engines.mujoco_shm import SEQ_POSITION_CMD, SEQ_POSITIONS
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule

EVENTS: dict[str, list[list[float]]] = defaultdict(list)
METADATA: dict[str, Any] = {}


def emit(event: str, *values: float) -> None:
    EVENTS[event].append([time.perf_counter(), *values])


def wrap(instance: Any, name: str, probe: Callable[..., Any]) -> None:
    original = getattr(instance, name)

    def measured(*args: Any, **kwargs: Any) -> Any:
        return probe(original, *args, **kwargs)

    setattr(instance, name, measured)


def model_metadata(model: mujoco.MjModel) -> dict[str, Any]:
    buffer = np.empty(mujoco.mj_sizeModel(model), dtype=np.uint8)
    mujoco.mj_saveModel(model, None, buffer)
    return {
        "mjb_sha256": hashlib.sha256(buffer).hexdigest(),
        "nq": model.nq,
        "ngeom": model.ngeom,
        "nmesh": model.nmesh,
        "ntex": model.ntex,
        "timestep": model.opt.timestep,
    }


class TraceRPC:
    @rpc
    def benchmark_trace(self) -> dict[str, Any]:
        memory = psutil.Process().memory_full_info()
        return {
            "pid": os.getpid(),
            "events": dict(EVENTS),
            "metadata": METADATA,
            "memory": {"uss": memory.uss, "rss": memory.rss},
        }


class MeasuredConnection(TraceRPC, WholeBodyConnection):
    pass


class CameraRays:
    """Match the old engine's actual ray pattern, not sim2's different default."""

    min_range = 0.1
    max_range = 20.0

    def __init__(self, width: int = 128, height: int = 64) -> None:
        self.width, self.height = width, height

    def directions(self) -> NDArray[np.float64]:
        return _camera_ray_directions(self.width, self.height, 60.0)


class MeasuredSimulation(TraceRPC, SimulationModule):
    @rpc
    def build(self) -> None:
        if self._runtime is not None:
            return
        super().build()
        runtime = self._require_runtime()
        METADATA["model"] = model_metadata(runtime.model)
        binding = runtime.robots["g1"]

        def step(original: Callable[..., Any]) -> Any:
            before = time.perf_counter()
            result = original()
            emit("physics", runtime.data.time, runtime.data.xpos[binding.root, 2], before)
            return result

        def read_action(original: Callable[..., Any], **kwargs: Any) -> Any:
            frame = original(**kwargs)
            if frame is not None:
                emit("applied", frame.metadata.sequence)
            return frame

        def publish(original: Callable[..., Any], *args: Any, **kwargs: Any) -> Any:
            sequence = original(*args, **kwargs)
            emit("feedback", sequence)
            return sequence

        wrap(runtime, "step", step)
        wrap(binding.channel, "read_action", read_action)
        wrap(binding.channel, "publish_observation", publish)


class MeasuredCamera(TraceRPC, SimRGBDCameraModule):
    def capture(self) -> None:
        before = time.perf_counter()
        super().capture()
        emit("camera_work", before, self.reader.timestamp)


class MeasuredLidar(TraceRPC, LidarModule):
    def capture(self) -> None:
        before = time.perf_counter()
        super().capture()
        emit("lidar_work", before, self.reader.timestamp)


class SharedCamera(MeasuredCamera):
    dedicated_worker = False


class SharedLidar(MeasuredLidar):
    dedicated_worker = False


class MeasuredCoordinator(TraceRPC, ControlCoordinator):
    g1_joints: Out[JointState]

    @rpc
    def start(self) -> None:
        super().start()
        assert self._tick_loop is not None

        def tick(original: Callable[..., Any]) -> Any:
            before = time.perf_counter()
            result = original()
            emit("control", before)
            return result

        wrap(self._tick_loop, "_tick", tick)
        adapter = self._hardware["g1"].adapter
        native = hasattr(adapter, "channel")

        def read(original: Callable[..., Any]) -> Any:
            before = time.perf_counter()
            result = original()
            if native:
                seq = adapter._sample.metadata.sequence
            else:
                seq = int(np.frombuffer(adapter._shm.shm.seq.buf, np.int64)[SEQ_POSITIONS])
            emit("feedback_read", seq, before)
            return result

        wrap(adapter, "read_motor_states", read)

        def write(original: Callable[..., Any], *args: Any, **kwargs: Any) -> Any:
            before = time.perf_counter()
            result = original(*args, **kwargs)
            seq = (
                result
                if native
                else int(np.frombuffer(adapter._shm.shm.seq.buf, np.int64)[SEQ_POSITION_CMD])
            )
            emit("command", seq, before)
            return result

        if native:
            wrap(adapter.channel, "publish_action", write)
        else:
            wrap(adapter._shm, "write_pd_tau_command", write)


class MeasuredEngine(MujocoEngine):
    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        METADATA["model"] = model_metadata(self._model)
        # Equal visible geometry and self-exclusion. This changes only visibility
        # groups, not physics, scheduling, rendering or the raycast algorithm.
        Raycaster(self._model, self._model.body("g1/pelvis").id)
        for camera in self._camera_configs:
            camera.geom_groups = tuple(range(6))

    def _render_cameras(self, now: float, states: Any) -> None:
        due = any(now - s.last_render_time >= s.interval for s in states.values())
        if due:
            emit("camera_source", now, EVENTS["physics"][-1][0])
        before = time.perf_counter()
        super()._render_cameras(now, states)
        if due:
            emit("camera_work", before, now)

    def _raycast_lidars(self, now: float, states: Any) -> None:
        due = any(now - s.last_cast_time >= s.interval for s in states.values())
        if due:
            emit("lidar_source", now, EVENTS["physics"][-1][0])
        before = time.perf_counter()
        super()._raycast_lidars(now, states)
        if due:
            emit("lidar_work", before, now)


class MeasuredOld(TraceRPC, MujocoSimModule):
    @rpc
    def start(self) -> None:
        original = old_module.MujocoEngine
        old_module.MujocoEngine = MeasuredEngine
        try:
            super().start()
        finally:
            old_module.MujocoEngine = original

    def _publish_shm_and_lcm(self, engine: MujocoEngine) -> None:
        super()._publish_shm_and_lcm(engine)
        assert self._shm is not None
        if not hasattr(self, "_benchmark_installed"):
            self._benchmark_installed = True

            def pre_step(original: Callable[..., Any], *args: Any) -> Any:
                self._benchmark_start = time.perf_counter()
                return original(*args)

            def read(original: Callable[..., Any], *args: Any) -> Any:
                result = original(*args)
                if result is not None:
                    emit("applied", self._shm._last_pos_cmd_seq)
                return result

            def publish(original: Callable[..., Any], *args: Any, **kwargs: Any) -> Any:
                result = original(*args, **kwargs)
                emit("feedback", self._shm._get_seq(SEQ_POSITIONS))
                return result

            wrap(engine, "_on_before_step", pre_step)
            wrap(self._shm, "read_position_command", read)
            wrap(self._shm, "write_joint_state", publish)
        emit(
            "physics",
            engine.data.time,
            engine.data.xpos[engine.model.body("g1/pelvis").id, 2],
            getattr(self, "_benchmark_start", time.perf_counter()),
        )
