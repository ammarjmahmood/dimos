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

"""Bounded headless probe, not a production backend or an end-to-end benchmark."""

import argparse
from dataclasses import replace
from importlib.metadata import version
import json
from pathlib import Path
import sys
import time
from typing import Any
from uuid import uuid4

import numpy as np
from numpy.typing import NDArray
from PIL import Image
import robosuite
from robosuite.utils.observables import Observable, sensor

from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.control.tasks.g1_groot_wbc_task.g1_groot_wbc_task import (
    G1GrootWBCTask,
    G1GrootWBCTaskConfig,
    g1_joints,
    g1_legs_waist,
)
from dimos.hardware.whole_body.spec import MotorCommand, WholeBodyAdapter as Adapter
from dimos.robot.unitree.g1.sim2 import G1_GROOT
from dimos.sim2.control.adapters import WholeBodyAdapter
from dimos.sim2.runtime import SimulationRuntime
from dimos.sim2.sensors.spec import Camera, Mount
from dimos.sim2.spec import RobotInstance, WorldConfig
from experiments.robosuite_emulator.bridge import MotorEnvironment


def world(root: Path, extra_camera: bool = False) -> WorldConfig:
    robot = replace(G1_GROOT, meshdir=root / "data/g1_urdf/meshes")
    if extra_camera:
        robot = robot.with_sensor(
            Camera(
                "pelvis_camera",
                Mount("pelvis", xyz=(0.1, 0, 0.05), rpy=(np.pi / 2, 0, -np.pi / 2)),
                width=320,
                height=240,
                rate_hz=15,
                depth=False,
            )
        )
    return WorldConfig(
        scene=root / "data/sim2/scenes/logistics.xml",
        robots={"g1": RobotInstance(robot, xyz=(0, 0, robot.spawn_height))},
        timestep=0.005,
    )


def policy(root: Path, adapter: Adapter) -> G1GrootWBCTask:
    task = G1GrootWBCTask(
        "spike-groot",
        G1GrootWBCTaskConfig(
            balance_onnx=root / "data/groot/balance.onnx",
            walk_onnx=root / "data/groot/walk.onnx",
            joint_names=g1_legs_waist,
            all_joint_names=g1_joints,
            decimation=1,
            auto_arm=True,
            auto_dry_run=False,
            default_ramp_seconds=0,
        ),
        adapter,
    )
    task.start()
    return task


def command(
    task: G1GrootWBCTask, adapter: Adapter, config: WorldConfig, tick: int, speed: float
) -> list[MotorCommand]:
    motors = adapter.read_motor_states()
    state = CoordinatorState(
        joints=JointStateSnapshot(
            joint_positions={name: m.q for name, m in zip(g1_joints, motors, strict=True)},
            joint_velocities={name: m.dq for name, m in zip(g1_joints, motors, strict=True)},
        ),
        imu={"g1": adapter.read_imu()},
        t_now=1 + tick * 0.02,
        dt=0.02,
    )
    # Stand for two seconds, walk for six, then stop; refresh the normal command timeout.
    vx = speed if 100 <= tick < 400 else 0.0
    task.set_velocity_command(vx, 0, 0, state.t_now)
    output = task.compute(state)
    if output is None:
        raise RuntimeError("the actual GR00T task did not produce a motor target")
    targets = dict(zip(output.joint_names, output.positions, strict=True))
    return [
        MotorCommand(q=targets.get(j.name, j.home), kp=j.kp, kd=j.kd)
        for j in config.robots["g1"].config.joints
    ]


def run_g1(args: argparse.Namespace) -> dict[str, Any]:
    config = world(args.assets_root, args.extra_camera)
    definition = config.robots["g1"].config
    started = time.perf_counter()
    if args.backend == "robosuite":
        runtime = MotorEnvironment(config, sense=args.sensors)
        adapter = runtime
        model, data = runtime.model, runtime.sim.data._data
        advance = runtime.advance
        runtime.reset()
    else:
        if args.sensors or args.extra_camera:
            raise ValueError(
                "native probe measures control/SHM only; no sensor throughput comparison"
            )
        sim_id = "rs-spike-" + uuid4().hex
        runtime = SimulationRuntime(config, sim_id)
        adapter = WholeBodyAdapter(address=sim_id + "/g1", dof=29, definition=definition)
        adapter.connect()
        model, data = runtime.model, runtime.data

        def advance() -> dict[str, Any]:
            for _ in range(4):
                runtime.step()
            return {}

    setup_seconds = time.perf_counter() - started
    task = None
    try:
        started = time.perf_counter()
        task = policy(args.assets_root, adapter)
        policy_load_seconds = time.perf_counter() - started
        poses, states, commands, times = [], [], [], []
        observations: dict[str, Any] = {}
        failure = None
        root_id = model.body("g1/pelvis").id
        if args.backend == "robosuite":
            runtime.captures.clear()
            runtime.capture_times.clear()
        started = time.perf_counter()
        for tick in range(round(args.seconds * 50)):
            tick_started = time.perf_counter()
            targets = command(task, adapter, config, tick, args.walk_speed)
            if not adapter.write_motor_commands(targets):
                raise RuntimeError("motor target rejected")
            observations = advance()
            times.append(time.perf_counter() - tick_started)
            poses.append(data.xpos[root_id].copy())
            states.append(data.qpos.copy())
            commands.append([c.q for c in targets])
            expected_time = (tick + 1) * 0.02
            if not np.isclose(data.time, expected_time):
                failure = f"MuJoCo reset/diverged at expected time {expected_time:.3f}s"
                break
            if not np.isfinite(data.qpos).all() or data.xpos[root_id, 2] < 0.4:
                failure = f"G1 fell or diverged at {data.time:.3f}s"
                break
        elapsed = time.perf_counter() - started
        path = args.output / args.label
        path.mkdir(parents=True, exist_ok=True)
        np.savez(path / "trajectory.npz", qpos=states, commands=commands, root=poses)
        result = dict(
            backend=args.backend,
            mujoco=version("mujoco"),
            robosuite=robosuite.__version__,
            sensors=args.sensors,
            extra_camera=args.extra_camera,
            setup_seconds=setup_seconds,
            policy_load_seconds=policy_load_seconds,
            elapsed_seconds=elapsed,
            sim_seconds=float(data.time),
            realtime_factor=float(data.time / elapsed) if failure is None else None,
            tick_ms_p50=float(np.percentile(times, 50) * 1000),
            tick_ms_p95=float(np.percentile(times, 95) * 1000),
            nq=model.nq,
            nv=model.nv,
            nu=model.nu,
            nbody=model.nbody,
            ngeom=model.ngeom,
            root_start=poses[0].tolist(),
            root_end=poses[-1].tolist(),
            minimum_root_height=float(np.array(poses)[:, 2].min()),
            failure=failure,
            walk_speed=args.walk_speed,
            note="Synchronous headless policy probe; not deployed ControlCoordinator/transport throughput.",
        )
        if args.backend == "robosuite":
            result["captures"] = runtime.captures
            result["capture_times"] = runtime.capture_times
            result["sensors_evidence"] = {}
            for name, values in observations.items():
                if name.endswith("_image"):
                    image = np.asarray(values)
                    Image.fromarray(image).save(path / (name + ".png"))
                    result["sensors_evidence"][name] = dict(
                        shape=list(image.shape),
                        std=float(image.std()),
                        unique_colors=len(np.unique(image.reshape(-1, 3), axis=0)),
                    )
                elif name.endswith("_depth") or name.endswith("_points"):
                    values = np.asarray(values)
                    result["sensors_evidence"][name] = dict(
                        shape=list(values.shape),
                        finite=bool(np.isfinite(values).all()),
                        minimum=float(values.min()),
                        maximum=float(values.max()),
                    )
        (path / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        return result
    finally:
        if task is not None:
            task.stop()
        adapter.disconnect()
        runtime.close()


def run_stock(args: argparse.Namespace) -> dict[str, Any]:
    env = robosuite.make(
        "Lift",
        robots="XArm7",
        has_renderer=False,
        has_offscreen_renderer=True,
        renderer="mujoco",
        camera_names=["robot0_eye_in_hand"],
        camera_widths=320,
        camera_heights=240,
        camera_depths=True,
        hard_reset=False,
        ignore_done=True,
        seed=17,
    )
    try:
        observations = env.reset()

        @sensor(modality="range")  # type: ignore[untyped-decorator]
        def joint_norm(cache: dict[str, Any]) -> NDArray:
            return np.array([np.linalg.norm(env.robots[0]._joint_positions)])

        name = "custom_measurement"
        env.add_observable(Observable(name, joint_norm, sampling_rate=10))
        env.modify_observable(name, "corrupter", lambda value: value + 0.25)
        for _ in range(10):
            observations, reward, done, _ = env.step(np.zeros(env.action_dim))
        result = dict(
            backend="stock-robosuite",
            mujoco=version("mujoco"),
            action_dim=env.action_dim,
            robot=type(env.robots[0]).__name__,
            controller=type(env.robots[0].composite_controller).__name__,
            rgb_shape=list(observations["robot0_eye_in_hand_image"].shape),
            depth_shape=list(observations["robot0_eye_in_hand_depth"].shape),
            custom_measurement=float(np.asarray(observations[name]).reshape(-1)[0]),
            reward=float(reward),
            done=done,
        )
        args.output.mkdir(parents=True, exist_ok=True)
        (args.output / "stock.json").write_text(json.dumps(result, indent=2) + "\n")
        return result
    finally:
        env.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", choices=("native", "robosuite", "stock"), required=True)
    parser.add_argument("--assets-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--label", default="run")
    parser.add_argument("--seconds", type=float, default=10)
    parser.add_argument("--walk-speed", type=float, default=0.35)
    parser.add_argument("--sensors", action="store_true")
    parser.add_argument("--extra-camera", action="store_true")
    args = parser.parse_args()
    result = run_stock(args) if args.backend == "stock" else run_g1(args)
    result.pop("capture_times", None)
    print(json.dumps(result, indent=2), flush=True)
    if result.get("failure"):
        sys.exit(1)


if __name__ == "__main__":
    main()
