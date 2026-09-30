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

"""Matched real-worker G1/GR00T benchmark, not a shipped robot blueprint.

Run from the repository root with `python -m experiments.sim2_timing.demo_benchmark`.
Each invocation owns its worker tree and private Zenoh router; it never attaches
to or stops a user's running blueprint. Raw probes are retained alongside results.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
from dataclasses import replace
import gc
import importlib.metadata
import json
import math
import os
from pathlib import Path
import platform
import signal
import socket
import subprocess
import sys
import time
from typing import Any

import mujoco
import numpy as np
import psutil

from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.global_config import global_config
from dimos.core.transport_factory import make_transport
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.protocol.service.zenohservice import ZenohService
from dimos.robot.unitree.g1.control_config import G1_GROOT_HOME, g1_arms, g1_legs_waist
from dimos.robot.unitree.g1.sim2 import G1_GROOT
from dimos.sim2.blueprint import simulation
from dimos.sim2.connections.whole_body import WholeBodyConnection
from dimos.sim2.module import SimulationModule
from dimos.sim2.scene import load_scene, scene_path, scene_robot
from dimos.sim2.sensors.camera.module import SimRGBDCameraModule
from dimos.sim2.sensors.lidar.module import LidarModule
from dimos.sim2.sensors.spec import Camera, Imu, Lidar, Mount
from dimos.sim2.spec import WorldConfig
from dimos.simulation.engines.robot_sim_binding import RobotSimSpec
from dimos.utils.data import LfsPath
from experiments.sim2_timing.instrumentation import (
    CameraRays,
    MeasuredCamera,
    MeasuredConnection,
    MeasuredCoordinator,
    MeasuredLidar,
    MeasuredOld,
    MeasuredSimulation,
    SharedCamera,
    SharedLidar,
    model_metadata,
)


def build_case(args: argparse.Namespace) -> tuple[Blueprint, dict[str, Any], list[str]]:
    sensing = args.case != "control"
    heavy = args.case == "heavy"
    width, height, hz = (1280, 960, 20) if heavy else (640, 480, 10)
    ray_width, ray_height = (256, 128) if heavy else (128, 64)
    sensors: list[Any] = [Imu("imu", site="control_imu")]
    if sensing:
        sensors += [
            Camera("camera", "front_camera", width=width, height=height, rate_hz=hz),
            Lidar(
                "lidar",
                Mount("torso_link", (0.07, 0, 0.42), (math.pi / 2, 0, -math.pi / 2)),
                CameraRays,
                {"width": ray_width, "height": ray_height},
                rate_hz=hz,
            ),
        ]
    robot = replace(G1_GROOT, sensors=tuple(sensors))
    scene = scene_path(args.scene, "logistics.xml")
    robots = {"g1": scene_robot(scene, robot, default=(0, 0, 0))}
    world = WorldConfig(scene=scene, robots=robots)
    model = load_scene(world)
    model_info = model_metadata(model)
    # A camera-shaped ray fan at the same named camera pose on both paths.
    if sensing:
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        camera, site = model.camera("g1/front_camera").id, model.site("g1/sensor/lidar").id
        np.testing.assert_allclose(data.cam_xpos[camera], data.site_xpos[site], atol=1e-12)
        np.testing.assert_allclose(data.cam_xmat[camera], data.site_xmat[site], atol=1e-12)
        del data
    mjb = args.output / "world.mjb"
    mujoco.mj_saveModel(model, str(mjb), None)
    del model
    gc.collect()
    sim = simulation(scene=scene, robots=robots, sim_id=f"benchmark-{os.getpid()}", viewer=False)
    hardware = sim.hardware["g1"]
    traced = ["physics", "ControlCoordinator"]
    if args.mode == "old":
        hardware = replace(
            hardware, adapter_type="sim_mujoco_g1", address=str(mjb), adapter_kwargs={}
        )
        backend = MeasuredOld.blueprint(
            instance_name="physics",
            address=str(mjb),
            dof=29,
            headless=True,
            reset_joint_positions=list(G1_GROOT_HOME),
            robot_sim_spec=RobotSimSpec(
                robot_id="g1",
                hardware_joints=tuple(j.name for j in robot.joints),
                root_body_names=("g1/pelvis",),
                root_joint_names=("g1/floating_base_joint",),
                require_floating_base=True,
                model_joint_names=tuple(f"g1/{j.model_name}" for j in robot.joints),
                model_actuator_names=tuple(f"g1/{j.actuator}" for j in robot.joints),
                imu_quat_names=("g1/sensor/imu/quat",),
                imu_gyro_names=("g1/sensor/imu/gyro",),
                imu_accel_names=("g1/sensor/imu/accel",),
                require_imu=True,
            ),
            camera_name="g1/front_camera",
            base_frame_id="g1/pelvis",
            width=width,
            height=height,
            fps=hz,
            enable_color=sensing,
            enable_depth=sensing,
            enable_pointcloud=sensing,
            enable_mujoco_lidar=sensing,
            pointcloud_fps=hz,
            mujoco_lidar_camera_names=["g1/front_camera"],
            mujoco_lidar_raycast_width=ray_width,
            mujoco_lidar_raycast_height=ray_height,
            mujoco_lidar_geom_groups=[0, 1, 2, 3, 4],
            mujoco_lidar_min_range=CameraRays.min_range,
            mujoco_lidar_max_range=CameraRays.max_range,
            mujoco_lidar_max_height=1000.0,
            mujoco_lidar_voxel_size=0.001,
        )
    else:
        replacements = {
            SimulationModule: MeasuredSimulation,
            WholeBodyConnection: MeasuredConnection,
            SimRGBDCameraModule: SharedCamera if args.mode == "compact" else MeasuredCamera,
            LidarModule: SharedLidar if args.mode == "compact" else MeasuredLidar,
        }
        atoms = []
        for atom in sim.blueprint.blueprints:
            kwargs = dict(atom.kwargs)
            atoms.append(
                replace(
                    atom,
                    module=replacements.get(atom.module, atom.module),
                    kwargs=kwargs,
                    instance_name="physics"
                    if atom.module is SimulationModule
                    else atom.instance_name,
                )
            )
        backend = replace(sim.blueprint, blueprints=tuple(atoms))
        traced.append("g1_connection")
        if sensing:
            traced += ["g1_camera", "g1_lidar"]
    coordinator = MeasuredCoordinator.blueprint(
        instance_name="ControlCoordinator",
        tick_rate=50.0,
        publish_robot_joint_states=True,
        hardware=[hardware],
        tasks=[
            TaskConfig(
                name="groot_wbc",
                type="g1_groot_wbc",
                joint_names=g1_legs_waist,
                priority=50,
                auto_start=True,
                params={
                    "model_path": LfsPath("groot"),
                    "hardware_id": "g1",
                    "auto_arm": True,
                    "auto_dry_run": False,
                    "default_ramp_seconds": 0.0,
                    "decimation": 1,
                },
            ),
            joint_trajectory_task(g1_arms, priority=10, velocity_limits={j: 1.0 for j in g1_arms}),
        ],
    )
    blueprint = autoconnect(backend, coordinator).global_config(
        n_workers=2, robot_model="unitree_g1"
    )
    return (
        blueprint,
        {
            "model": model_info,
            "scene": str(scene),
            "camera": [width, height, hz] if sensing else None,
            "lidar": [ray_width * ray_height, hz] if sensing else None,
        },
        traced,
    )


def percentiles(values: Any) -> dict[str, Any]:
    values = np.asarray(values, dtype=float)
    if not len(values):
        return {"count": 0}
    return {
        "count": len(values),
        "mean": float(np.mean(values)),
        **{
            key: float(value)
            for key, value in zip(
                ("p50", "p95", "p99", "max"), np.percentile(values, [50, 95, 99, 100]), strict=True
            )
        },
    }


def summarize(raw: dict[str, Any]) -> dict[str, Any]:
    begin, end = raw["window"]
    events: dict[str, list[list[float]]] = defaultdict(list)
    for trace in raw["traces"].values():
        for key, rows in trace["events"].items():
            events[key].extend(rows)
    for rows in events.values():
        rows.sort()

    def select(key: str) -> list[list[float]]:
        return [r for r in events[key] if begin <= r[0] < end]

    physics = select("physics")
    control = select("control")
    assert len(physics) > 10 and len(control) > 10, "missing loop probes"
    output: dict[str, Any] = {}
    for name, rows, period in (("physics", physics, 0.005), ("control", control, 0.020)):
        # Physics completions vs control tick entry, not RPC polling intervals.
        stamps = np.array([r[0] if name == "physics" else r[1] for r in rows])
        gaps = np.diff(stamps)
        output[name] = {
            "hz": len(stamps) / (end - begin),
            "interval_ms": percentiles(gaps * 1000),
            "gaps_over_1_5_period": int(np.sum(gaps > period * 1.5)),
            "work_ms": percentiles([(r[0] - r[-1]) * 1000 for r in rows]),
        }
    # Count the entire wall window, including a stall at its beginning or end.
    output["rtf"] = len(physics) * raw["workload"]["model"]["timestep"] / (end - begin)
    output["minimum_pelvis_z"] = min(r[2] for r in physics)
    first_applied: dict[int, float] = {}
    for row in events["applied"]:
        first_applied.setdefault(int(row[1]), row[0])
    commands = select("command")
    # Start-of-write to action read includes publication cost; completed writes
    # can race a reader, so avoid inventing negative end-of-write latencies.
    latencies = [
        (first_applied[int(r[1])] - r[2]) * 1000 for r in commands if int(r[1]) in first_applied
    ]
    output["command_to_read_ms"] = percentiles(latencies)
    output["commands_not_observed"] = sum(int(r[1]) not in first_applied for r in commands)
    feedback = {int(r[1]): r[0] for r in events["feedback"]}
    ages = [
        (r[0] - feedback[int(r[1])]) * 1000
        for r in select("feedback_read")
        if int(r[1]) in feedback
    ]
    output["feedback_age_ms"] = percentiles(ages)
    output["sensors"] = {}
    for channel, messages in raw["received"].items():
        messages = [r for r in messages if begin <= r[0] < end]
        # The old publisher may resend a frame; count fresh snapshots, not callbacks.
        fresh = list({r[1]: r for r in reversed(messages)}.values())
        fresh.sort()
        kind = "lidar" if channel == "pointcloud" else "camera"
        source = {r[1]: r[2] for r in events[f"{kind}_source"]}
        if raw["mode"] == "old":
            ages = [(r[0] - source[r[1]]) * 1000 for r in fresh]
        else:
            ages = [(r[2] - r[1]) * 1000 for r in fresh]
        output["sensors"][channel] = {
            "fresh_hz": len(fresh) / (end - begin),
            "callbacks": len(messages),
            "fresh_frames": len(fresh),
            "source_to_subscriber_ms": percentiles(ages),
            "capture_work_ms": percentiles([(r[0] - r[1]) * 1000 for r in select(f"{kind}_work")]),
        }
    memory = raw["memory"]
    output["process_count"] = max(len(r["processes"]) for r in memory)
    private = {p["pid"]: p["uss"] for p in memory[-1]["processes"]}
    private.update({t["pid"]: t["memory"]["uss"] for t in raw["traces"].values()})
    output["known_steady_uss_mib"] = sum(v for v in private.values() if v is not None) / 2**20
    output["uss_unavailable_pids"] = [p for p, v in private.items() if v is None]
    output["peak_sampled_rss_sum_mib"] = (
        max(sum(p["rss"] for p in r["processes"]) for r in memory) / 2**20
    )
    first = {p["pid"]: p["cpu"] for p in memory[0]["processes"]}
    last = {p["pid"]: p["cpu"] for p in memory[-1]["processes"]}
    output["cpu_cores_mean"] = sum(v - first[p] for p, v in last.items() if p in first) / (
        memory[-1]["t"] - memory[0]["t"]
    )
    return output


def memory_sample() -> dict[str, Any]:
    parent = psutil.Process()
    processes = []
    for process in [parent, *parent.children(recursive=True)]:
        try:
            memory, cpu = process.memory_info(), process.cpu_times()
            try:
                uss = process.memory_full_info().uss
            except psutil.AccessDenied:
                uss = None
            processes.append(
                {
                    "pid": process.pid,
                    "rss": memory.rss,
                    "uss": uss,
                    "cpu": cpu.user + cpu.system,
                    "name": process.name(),
                }
            )
        except psutil.NoSuchProcess:
            pass
    return {"t": time.perf_counter(), "processes": processes}


def run_suite(args: argparse.Namespace) -> None:
    args.output.mkdir(parents=True, exist_ok=False)
    workloads = [
        ("control", "logistics.xml"),
        ("normal", "logistics.xml"),
        ("heavy", "logistics.xml"),
        ("heavy", "robocasa-kitchen-1"),
    ]
    results = []
    for repeat in range(2):
        modes = args.modes if repeat == 0 else list(reversed(args.modes))
        for case, scene in workloads:
            for mode in modes:
                name = f"{scene.removesuffix('.xml')}-{case}-{mode}-{repeat + 1}"
                destination = args.output / name
                print(f"START {name}", flush=True)
                command = [
                    sys.executable,
                    "-m",
                    "experiments.sim2_timing.demo_benchmark",
                    mode,
                    "--case",
                    case,
                    "--scene",
                    scene,
                    "--seconds",
                    str(args.seconds),
                    "--warmup",
                    str(args.warmup),
                    "--output",
                    str(destination),
                ]
                with (args.output / f"{name}.log").open("w") as log:
                    process = subprocess.Popen(
                        command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True
                    )
                    try:
                        code = process.wait(timeout=args.seconds + args.warmup + 100)
                    except BaseException:
                        os.killpg(process.pid, signal.SIGTERM)
                        try:
                            process.wait(timeout=5)
                        except subprocess.TimeoutExpired:
                            os.killpg(process.pid, signal.SIGKILL)
                            process.wait()
                        raise
                if code:
                    raise RuntimeError(f"{name} failed ({code}); see its log")
                raw = json.loads((destination / "result.json").read_text())
                results.append(
                    {
                        "name": name,
                        **{
                            k: v
                            for k, v in raw.items()
                            if k not in ("traces", "received", "memory")
                        },
                    }
                )
                (args.output / "summary.json").write_text(json.dumps(results, indent=2))
                s = raw["summary"]
                print(
                    f"DONE {name}: RTF={s['rtf']:.3f} physics-p99={s['physics']['interval_ms']['p99']:.2f}ms "
                    f"command-p99={s['command_to_read_ms']['p99']:.2f}ms "
                    f"known-USS={s['known_steady_uss_mib']:.0f}MiB z={s['minimum_pelvis_z']:.3f}",
                    flush=True,
                )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=["old", "split", "compact", "suite"])
    parser.add_argument("--case", choices=["control", "normal", "heavy"], default="normal")
    parser.add_argument("--seconds", type=float, default=25.0)
    parser.add_argument("--warmup", type=float, default=5.0)
    parser.add_argument("--scene", default="logistics.xml")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--modes",
        nargs="+",
        choices=["old", "split", "compact"],
        default=["old", "split", "compact"],
        help="Suite arrangements",
    )
    args = parser.parse_args()
    if args.mode == "suite":
        run_suite(args)
        return
    args.output.mkdir(parents=True, exist_ok=False)
    global_config.update(simulation="mujoco", viewer="none", transport="zenoh", n_workers=2)
    with socket.socket() as reservation:
        reservation.bind(("127.0.0.1", 0))
        endpoint = f"tcp/127.0.0.1:{reservation.getsockname()[1]}"
    router = ZenohService(mode="router", listen=[endpoint], multicast=False)
    router.start()
    global_config.update(zenoh_mode="client", zenoh_connect=endpoint, zenoh_multicast=False)
    coordinator = None
    subscriptions: list[Any] = []
    transports: list[Any] = []
    received: dict[str, list[list[float]]] = defaultdict(list)
    raw: dict[str, Any] = {
        "mode": args.mode,
        "case": args.case,
        "received": received,
        "traces": {},
        "frame_samples": {},
        "memory": [],
        "environment": {
            "platform": platform.platform(),
            "cpu_count": os.cpu_count(),
            "ram_bytes": psutil.virtual_memory().total,
            "mujoco": mujoco.__version__,
            "zenoh": importlib.metadata.version("eclipse-zenoh"),
            "thread_environment": {
                name: os.environ.get(name)
                for name in (
                    "OMP_WAIT_POLICY",
                    "KMP_BLOCKTIME",
                    "OMP_NUM_THREADS",
                    "VECLIB_MAXIMUM_THREADS",
                )
            },
            "commit": subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip(),
        },
    }
    try:
        blueprint, raw["workload"], traced = build_case(args)
        before = time.perf_counter()
        parsed = BlueprintConfigParser(blueprint).parse(
            global_overrides=global_config.model_dump(mode="python")
        )
        coordinator = ModuleCoordinator.build(blueprint, parsed_config=parsed)
        raw["startup_seconds"] = time.perf_counter() - before
        if args.case != "control":
            for channel, msg_type in (
                ("color_image", Image),
                ("depth_image", Image),
                ("pointcloud", PointCloud2),
            ):
                transport = make_transport(channel, msg_type)
                transport.start()

                def receive(msg: Any, key: str = channel) -> None:
                    received[key].append([time.perf_counter(), float(msg.ts), time.time()])
                    if key != "pointcloud" and key not in raw["frame_samples"]:
                        raw["frame_samples"][key] = {
                            "shape": list(msg.data.shape),
                            "dtype": str(msg.data.dtype),
                            "min": float(np.min(msg.data)),
                            "max": float(np.max(msg.data)),
                            "std": float(np.std(msg.data)),
                        }

                subscriptions.append(transport.subscribe(receive))
                transports.append(transport)
        time.sleep(args.warmup)
        begin = time.perf_counter()
        while time.perf_counter() - begin < args.seconds:
            raw["memory"].append(memory_sample())
            time.sleep(0.5)
        raw["window"] = [begin, time.perf_counter()]
        for name in traced:
            trace = coordinator.get_instance(name).benchmark_trace()
            raw["traces"][str(trace["pid"])] = trace
        expected = raw["workload"]["model"]["mjb_sha256"]
        actual = next(
            t["metadata"]["model"]["mjb_sha256"]
            for t in raw["traces"].values()
            if "model" in t["metadata"]
        )
        assert actual == expected, "different compiled physics model"
        if args.case != "control":
            assert all(received[c] for c in ("color_image", "depth_image", "pointcloud")), (
                "missing sensor stream"
            )
        raw["summary"] = summarize(raw)
    finally:
        before = time.perf_counter()
        for unsubscribe in subscriptions:
            unsubscribe()
        for transport in transports:
            transport.stop()
        try:
            if coordinator is not None:
                coordinator.stop()
        finally:
            router.stop()
            raw["shutdown_seconds"] = time.perf_counter() - before
            (args.output / "result.json").write_text(json.dumps(raw, indent=2))
            (args.output / "world.mjb").unlink(missing_ok=True)
    print(
        json.dumps(
            {k: v for k, v in raw.items() if k not in ("traces", "received", "memory")}, indent=2
        )
    )


if __name__ == "__main__":
    main()
