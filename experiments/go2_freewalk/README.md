# Go2 FREE Policy Through The SONIC Boundary

Measured 2026-10-06 on Apple M4 Max, macOS 15.6, Python 3.12.13,
MuJoCo 3.10.0, native Zenoh 1.10.1. Base: `pim/feat/sim2-core` at
`65cd8b8e1`. Work: `pim/feat/sim2-go2-freewalk`.

## Ownership

```text
keyboard / planner / ordinary cmd_vel publisher
    -> existing ControlCoordinator velocity task (50 Hz, priority + timeout)
    -> Twist over the existing transport adapter
    -> Go2FreewalkConnection (50 Hz inference, arming, feedback watchdog)
    -> MotorCommandArray(q, dq, kp, kd, tau)
    -> existing sim2 WholeBodyConnection in stream mode
    -> existing SHM motor channel
    -> MuJoCo physics owner (400 Hz)

physics -> coherent JointState + Imu (200 Hz) -> Go2FreewalkConnection
physics -> independent RGB-D / lidar workers
Mid360 raw timed returns + IMU -> existing native Point-LIO -> voxel mapper
```

This follows the current SONIC stack, not its earlier motor-level coordinator
task design. ControlCoordinator owns velocity arbitration; the policy connection
owns all twelve motor commands. There is one new DimOS module class. There are
no changes to ControlCoordinator, its task types, transports, mapping algorithms,
sim2's physics/runtime/IPC, or existing robot blueprints in this addition.

Source review:

- SONIC foundation [#4304](https://github.com/dimensionalOS/dimos/pull/4304),
  inference [#4305](https://github.com/dimensionalOS/dimos/pull/4305),
  connection [#3557](https://github.com/dimensionalOS/dimos/pull/3557),
  with the latter inspected at `d9ff201e1`.
- Numerical port adapted from Andrew's [#4441](https://github.com/dimensionalOS/dimos/pull/4441)
  `FreePolicy` at `4e524f40d`; his integrated physics loop and alternate ONNX
  policy are not imported. Coordinate extraction of this common inference code
  with Andrew rather than maintaining two ports after those branches land.
- Private go2web `policy/src/policies/unitree/himloco.rs`, revision
  `1ffe3ee6ab17f387c4fc6d6abf84b99d9748e6a7`. Its absent sprint expert
  selects the fast expert; this upstream selection rule is preserved.

## Run

From this worktree, authenticate `gh` with access to `dimensionalOS/go2web`:

```bash
uv run python -m dimos.control.go2_freewalk.models --install
uv run dimos --simulation mujoco --transport zenoh --viewer rerun run unitree-go2-freewalk
```

For Mid360 and actual Point-LIO, use the same command with
`unitree-go2-freewalk-pointlio`. On a fresh checkout, build the existing native
Point-LIO and voxel-mapper binaries using the repository's native build setup
(`--build-native` enables their configured build commands). This worktree reused
the already-built native-core binaries from identical source, not stubs.

`--scene-package /path/to/scene.xml` selects another ordinary MJCF scene.
Both blueprints are simulation-only and never connect to physical motors.
They intentionally coexist with the older Go2 WebRTC simulation while that
separate stack is being migrated; this does not silently change its policy.

Weights are 2,919,640 bytes, SHA-256
`406e735c32a8e5501c9122ef83fa61d67c70aa9bfc9a4dae266b321db1458330`.
The explicit installer uses authenticated `gh` and verifies the pinned blob.
They live under the user's cache, never in this public repository. Missing or
different weights are errors, not a request to run another policy.

The 4.4 MiB robot archive contains only the Menagerie Go2 MJCF, meshes and
upstream BSD-3-Clause license/README/changelog, extracted from Andrew's retained
asset. It contains neither policy weights nor another Mid360 model. Joint gains
and starting pose match the pinned FREE metadata; physical inertias, contacts,
damping and actuator limits retain the source MJCF. The camera is an ideal RGB-D
device, not a calibrated Go2 front camera. Mid360 uses the existing measured SF
mount: camera offset plus (-0.032, 0, 0.12) m and 60-degree downward pitch.

## Interface

- `base_command: In[Twist]`: bounded forward/lateral/yaw references.
- `motor_states: In[JointState]`, `imu: In[Imu]`: same-timestamp low-level samples.
- `motor_command: Out[MotorCommandArray]`: full PD frame in configured joint order.
- RPCs: `arm() -> bool`, `disarm()`, `status()` plus normal module lifecycle.
- Defaults: unarmed; these simulation blueprints explicitly enable auto-arm.
  A fresh coherent sample is required, followed by a one-second standing ramp.
- Expired velocity commands return to zero while the policy continues balancing.
  Stale/invalid feedback, excessive tilt or inference failure latch damping;
  fresh data alone does not restart a faulted policy. Explicit `arm()` is required.
- Physical scene reset does not reset controller history automatically. Disarm,
  reset the simulation, wait for fresh feedback, then arm again.

## Evidence

`motion.json` and `pointlio.json` retain measured results, not target rates.

| Check | Result |
| --- | --- |
| Upstream MNN comparison | All 18 vectors; maximum raw output error 0.00017543, upstream tolerance 0.003 |
| Policy + device tests | 93 passed, including sim2 MuJoCo tests and registry generation |
| Scoped production mypy | 7 source files passed |
| Motion run | 14.12 s, real-time factor 1.00019, policy 49.98 Hz |
| Inference p95 | 0.224 ms |
| Forward | 1.114 m in 4 s, commanded 0.3 m/s |
| Lateral | 0.549 m in 3 s, commanded 0.2 m/s |
| Turn | 0.540 rad in 3 s, commanded 0.3 rad/s |
| Stop interval | 0.0215 m displacement over 2 s |
| Point-LIO run | 15.05 s, real-time factor 1.00000 |
| Mid360 / IMU | 9.967 scans/s / 200.000 samples/s; zero recorded drops |
| Point-LIO pose comparison | 7.01 mm RMS, 24.82 mm maximum position error over 0.572 m net motion |
| Shutdown | Both runs exited normally; native processes returned zero |

The first loop used relative waits and ran below 50 Hz due to macOS oversleep.
Absolute deadlines corrected this without catch-up inference bursts or changes
to the shared coordinator scheduler. Do not confuse this with changing physics
speed. The recorded motion test overlapped the focused test run; it is not an
isolated-machine performance benchmark. The Point-LIO run did not overlap tests.

Repeat the actual blueprint probes:

```bash
uv run python -m dimos.robot.unitree.go2.demo_freewalk
uv run python -m dimos.sim2.demo_pointlio --robot go2 --move --seconds 15
```

For numerical comparison, retrieve go2web's private
`policy/tests/freewalk_mcf.validation.json` at the revision above. For every
case compare `FreePolicy.forward_for_kind(kind, np.array(p_obs))` with `act`,
as upstream `freewalk_validation.rs` does. Reference vectors and weights remain
private; unit tests use synthetic networks and do not need repository access.

The estimator consumes raw timed lidar/IMU, not truth poses or truth orientation.
Whole-body and camera truth TF are routed away from navigation in the Point-LIO
blueprint. The probe alone reads truth for a single initial alignment and scoring.
Estimated camera/body transforms for manipulation are not supplied by this probe.

## Limits

This proves inference, ordinary blueprint control and simulated estimator
integration. It is not a hardware safety qualification, calibrated motor model,
stair-climbing test, or real-world SLAM benchmark. Velocity tracking is not exact:
the lateral phase also drifted 0.387 m forward and the turn under-rotated. Do not
hide that by tuning the simulator to the test. Closed-loop navigation goal
completion and sensor mounting calibration are separate acceptance checks.
The Mid360 material/IMU fidelity limits in `../mid360/README.md` still apply.
