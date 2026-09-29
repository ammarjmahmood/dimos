# M20 Simulation Assets

Experimental integration, kept on `pim/feat/sim2-m20-experiment` separately
from the G1/xArm emulator core. The public policy is not accepted for stair
climbing or vendor gait parity.

Source: [DeepRoboticsLab/sdk_deploy](https://github.com/DeepRoboticsLab/sdk_deploy),
commit `2367375f922f2a739f7b578849ae2affd4d8660d` (BSD-3-Clause).
The retained license is `LICENSE.sdk_deploy`.

`m20.xml` retains the source robot's inertias, joints, actuators and collisions.
Its world floor, world light and floor material are removed; the selected
sim2 scene owns those. Collision material alpha is zero so the visual meshes
remain visible without drawing duplicate collision shapes. `stairs.xml`
uses the source's 10 cm risers and 30 cm treads with a 60 cm top platform.

`data/.lfs/m20_sdk.tar.gz` contains only the 17 referenced STL meshes, the
unmodified M20 `policy.onnx`, and upstream license. `LfsPath("m20_sdk/...")`
uses ordinary DimOS data loading. The SDK checkout and its ROS2/DDS executable
are not runtime dependencies.

The public actor runs at 50 Hz inside ControlCoordinator. Its 57 inputs are
body gyro, projected gravity, velocity command, joint offsets, joint velocities
and previous actions. Twelve outputs control leg positions; four control wheel
velocities. The sim2 motor loop applies the source PD gains at 1 kHz.
Sensors are ideal simulation devices, not calibrated M20 camera/lidar replicas.
This does not replace or claim parity with the real robot's vendor Agile gait.

## Run

```bash
uv run dimos --simulation mujoco --transport zenoh --viewer rerun \
  run deeprobotics-m20-kronknav-control
```

The existing blueprint switches its hardware connection to sim2 plus the
ControlCoordinator actor. Its real-hardware branch remains unchanged.
The default world contains the SDK-sized stairs; `--scene-package` selects
another native world. G1 and M20 share the scene's `default` support pose;
M20 adds its configured 0.6 m root height. No M20-specific scene entry is needed.
The published actor is automatically active in simulation. Ordinary
`cmd_vel: Twist` controls forward/lateral velocity and yaw rate; the limits are
1.5 m/s, 0.5 m/s and 0.7 rad/s in this blueprint. Commands expire after 0.5 s without updates.
`MovementManager` supplies this stream from teleoperation and navigation.

The first KronkNav launch needs the existing Rust mapper/planner binaries.
They can be built beforehand with:

```bash
cargo build --release --locked -p dimos-voxel-ray-tracing -p dimos-mls-planner --bins
```

Verification on the initial integration: the real ONNX actor balanced and
moved forward through coordinator arbitration and SHM. The 40-second stair
trial remained upright but stopped at the first 10 cm riser. Stair climbing
is not accepted. The sensor camera produced native RGB/depth, and a separate
test checked sensor-frame lidar points with matching world transforms.

A headless run of the complete existing KronkNav blueprint on the development
Mac started in 1.83 s with assets and Rust binaries already available. Over
roughly five seconds, typed teleoperation commands moved the robot 0.75 m;
the stack published 49 RGB/depth pairs and 41 lidar scans/local maps. This
checks the live command/sensor/mapping path, not autonomous stair navigation
or sustained throughput. The run used an isolated Zenoh network and disabled
the viewer/web server; it did not replace a running robot session.

On the September 29 split onto current main, 86 focused coordinator, policy
and MuJoCo integration tests passed, including the actual ONNX balance/drive
test. The complete simulation blueprint passed configuration parsing. The
full KronkNav stack and stair trial were not rerun for this branch split.
