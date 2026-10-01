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

"""Run the SAME device workload against either worktree, selected by PYTHONPATH.

This is a motor/runtime comparison, not a deployed locomotion-policy benchmark.
No alternate physics implementation is embedded in the measurement harness.
"""

import argparse
from dataclasses import replace
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import time
from uuid import uuid4

import mujoco
import numpy as np

import dimos
from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.control.tasks.g1_groot_wbc_task.g1_groot_wbc_task import (
    G1GrootWBCTask,
    G1GrootWBCTaskConfig,
    g1_joints,
    g1_legs_waist,
)
from dimos.hardware.whole_body.spec import MotorCommand
from dimos.robot.manipulators.xarm.sim2 import XARM7
from dimos.robot.unitree.g1.sim2 import G1_GROOT
from dimos.sim2.control.adapters import ManipulatorAdapter, WholeBodyAdapter
from dimos.sim2.runtime import SimulationRuntime
from dimos.sim2.scene import scene_path, scene_robot
from dimos.sim2.sensors.camera.renderers.mujoco import MujocoCamera
from dimos.sim2.sensors.lidar.raycast import Raycaster
from dimos.sim2.sensors.reader import WorldReader
from dimos.sim2.sensors.spec import Camera, Lidar
from dimos.sim2.spec import ControlInterface, RobotInstance, WorldConfig
from dimos.utils.data import LfsPath


def standing_policy(adapter: WholeBodyAdapter) -> G1GrootWBCTask:
    models = LfsPath("groot")
    task = G1GrootWBCTask(
        "comparison-groot",
        G1GrootWBCTaskConfig(
            balance_onnx=models / "balance.onnx",
            walk_onnx=models / "walk.onnx",
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


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", choices=("g1", "xarm"), required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seconds", type=float, default=2)
    parser.add_argument("--scene")
    parser.add_argument("--sensors", action="store_true")
    parser.add_argument(
        "--groot", action="store_true", help="Run G1's real standing policy at 50 Hz"
    )
    args = parser.parse_args()
    if args.seconds <= 0 or (args.groot and args.robot != "g1"):
        parser.error("seconds must be positive; --groot requires --robot g1")
    definition, dt, xyz, scene = {
        "g1": (G1_GROOT, 0.005, (0, 0, 0.793), "logistics.xml"),
        "xarm": (XARM7, 0.005, (0, 0, 0.12), "workbench.xml"),
    }[args.robot]
    if not args.sensors:
        definition = replace(
            definition,
            sensors=tuple(s for s in definition.sensors if not isinstance(s, (Camera, Lidar))),
        )
    source = scene_path(args.scene, scene)
    instance = (
        scene_robot(source, definition, default=(0, 0, 0))
        if args.scene
        else RobotInstance(definition, xyz=xyz)
    )
    config = WorldConfig(source, {args.robot: instance}, timestep=dt)
    started = time.perf_counter()
    world = SimulationRuntime(config, "comparison-" + uuid4().hex)
    setup_seconds = time.perf_counter() - started
    adapter_type = (
        WholeBodyAdapter
        if definition.control == ControlInterface.WHOLE_BODY
        else ManipulatorAdapter
    )
    adapter = adapter_type(
        address=world.snapshot_descriptor.sim_id + "/" + args.robot,
        dof=len(definition.joints),
        definition=definition,
    )
    reader = renderer = None
    task = None
    policy_load_seconds = 0.0
    capture_seconds = 0.0
    camera_frames = lidar_frames = 0
    rgb_std = depth_valid = point_count = 0
    arrays = {}
    paths = args.output
    paths.parent.mkdir(parents=True, exist_ok=True)
    try:
        adapter.connect()
        if args.groot:
            started = time.perf_counter()
            task = standing_policy(adapter)
            policy_load_seconds = time.perf_counter() - started
        for field in (
            "body_mass",
            "body_inertia",
            "dof_damping",
            "dof_armature",
            "dof_frictionloss",
            "jnt_range",
            "geom_rgba",
            "actuator_gainprm",
            "actuator_biasprm",
            "actuator_ctrlrange",
        ):
            arrays["model_" + field] = getattr(world.model, field).copy()
        with TemporaryDirectory(prefix="sim2-comparison-") as directory:
            if args.sensors:
                model_path = Path(directory) / "world.mjb"
                mujoco.mj_saveModel(world.model, str(model_path), None)
                reader = WorldReader(
                    {"model": str(model_path), "snapshot": world.snapshot_descriptor.to_dict()}
                )
                camera = next(s for s in definition.sensors if isinstance(s, Camera))
                renderer = MujocoCamera(reader.model, camera.width, camera.height)
                camera_id = reader.model.camera(f"{args.robot}/{camera.model_name}").id
                lidar = next((s for s in definition.sensors if isinstance(s, Lidar)), None)
                # A lidar owns a different model from the camera, like the real workers.
                if lidar is not None:
                    ray_model = mujoco.MjModel.from_binary_path(str(model_path))
                    ray_data = mujoco.MjData(ray_model)
                    raycaster = Raycaster(
                        ray_model, ray_model.body(f"{args.robot}/{definition.root_body}").id
                    )
                    site = ray_model.site(f"{args.robot}/{lidar.model_name}").id
                    pattern = lidar.model(**lidar.model_kwargs)
                    directions = pattern.directions()
            qpos, qvel, controls, step_times, root_heights = [], [], [], [], []
            root_id = world.model.body(f"{args.robot}/{definition.root_body}").id
            targets = np.array([j.home for j in definition.joints])
            steps = round(args.seconds / dt)
            run_started = time.perf_counter()
            for tick in range(steps):
                if task is not None:
                    if tick % round(0.02 / dt) == 0:
                        motors = adapter.read_motor_states()
                        state = CoordinatorState(
                            joints=JointStateSnapshot(
                                joint_positions={
                                    n: m.q for n, m in zip(g1_joints, motors, strict=True)
                                },
                                joint_velocities={
                                    n: m.dq for n, m in zip(g1_joints, motors, strict=True)
                                },
                            ),
                            imu={"g1": adapter.read_imu()},
                            t_now=1 + tick * dt,
                            dt=0.02,
                        )
                        task.set_velocity_command(0, 0, 0, state.t_now)
                        output = task.compute(state)
                        if output is None:
                            raise RuntimeError("GR00T did not produce a motor target")
                        commanded = dict(zip(output.joint_names, output.positions, strict=True))
                        targets = np.array(
                            [commanded.get(j.name, j.home) for j in definition.joints]
                        )
                else:
                    targets = np.array([j.home for j in definition.joints])
                    targets[0] += 0.03 * np.sin(tick * dt * 3)
                if definition.control == ControlInterface.WHOLE_BODY:
                    adapter.write_motor_commands(
                        [
                            MotorCommand(q=q, kp=j.kp, kd=j.kd)
                            for q, j in zip(targets, definition.joints, strict=True)
                        ]
                    )
                else:
                    targets[-1] = 425 + 425 * np.cos(tick * dt * 3)
                    adapter.write_joint_positions(targets)
                before_step = time.perf_counter()
                world.step()
                step_times.append(time.perf_counter() - before_step)
                qpos.append(world.data.qpos.copy())
                qvel.append(world.data.qvel.copy())
                controls.append(world.data.ctrl.copy())
                root_heights.append(world.data.xpos[root_id, 2])
                if args.sensors and tick % round(0.1 / dt) == 0:
                    before_capture = time.perf_counter()
                    reader.update()
                    rgb, depth = renderer.capture(reader.data, camera_id, True)
                    rgb_std = float(rgb.std())
                    depth_valid = int(
                        np.count_nonzero(np.isfinite(depth) & (depth > 0.2) & (depth < 20))
                    )
                    camera_frames += 1
                    if lidar is not None:
                        ray_data.qpos[:] = reader.data.qpos
                        ray_data.qvel[:] = reader.data.qvel
                        ray_data.mocap_pos[:] = reader.data.mocap_pos
                        ray_data.mocap_quat[:] = reader.data.mocap_quat
                        mujoco.mj_forward(ray_model, ray_data)
                        points = raycaster.cast(
                            ray_data,
                            ray_data.site_xpos[site],
                            directions @ ray_data.site_xmat[site].reshape(3, 3).T,
                            pattern.min_range,
                            pattern.max_range,
                        )
                        point_count = len(points)
                        lidar_frames += 1
                    capture_seconds += time.perf_counter() - before_capture
            run_seconds = time.perf_counter() - run_started
            arrays.update(qpos=np.array(qpos), qvel=np.array(qvel), ctrl=np.array(controls))
            if args.sensors:
                arrays.update(rgb=rgb, depth=depth)
            np.savez_compressed(paths.with_suffix(".npz"), **arrays)
            metadata = {
                "source_root": str(Path(dimos.__file__).parents[1]),
                "groot": args.groot,
                "policy_load_seconds": policy_load_seconds,
                "robot": args.robot,
                "mujoco": mujoco.__version__,
                "scene": str(source),
                "setup_seconds": setup_seconds,
                "run_seconds": run_seconds,
                "step_p50_ms": float(np.percentile(step_times, 50) * 1000),
                "step_p99_ms": float(np.percentile(step_times, 99) * 1000),
                "steps": steps,
                "sim_seconds": float(world.data.time),
                "real_time_factor": float(world.data.time / run_seconds),
                "capture_seconds": capture_seconds,
                "camera_frames": camera_frames,
                "lidar_frames": lidar_frames,
                "last_point_count": point_count,
                "last_rgb_std": rgb_std,
                "last_valid_depth_pixels": depth_valid,
                "finite": bool(np.isfinite(arrays["qpos"]).all()),
                "minimum_root_height": float(min(root_heights)),
            }
            paths.with_suffix(".json").write_text(json.dumps(metadata, indent=2) + "\n")
            print(json.dumps(metadata, indent=2))
    finally:
        if task is not None:
            task.stop()
        if renderer is not None:
            renderer.close()
        if reader is not None:
            reader.close()
        adapter.disconnect()
        world.close()


if __name__ == "__main__":
    main()
