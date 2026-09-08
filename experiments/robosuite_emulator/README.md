# Robosuite Hardware-Emulator Spike

Isolated experiment on `test/robosuite-emulator-spike`, based on `a82725e85`.
No production simulation, blueprint, policy, dependency-lock or notebook changes.

## Questions

1. Can robosuite step the exact DimOS G1 model under the unchanged GR00T task?
2. Can mounted RGB-D and ideal lidar use its Observable interface?
3. What is reused, and what must remain a DimOS-specific bridge?
4. What changes when using the ordinary robosuite RobotEnv/controller path?

This is not a replacement backend or a complete deployed DimOS blueprint. The
hardware probe deliberately uses robosuite's lower-level MujocoEnv, retaining
the existing sim2 model composition. Its standard RobotEnv path is probed
separately; succeeding at the former does not prove the latter is drop-in.

## Environment

The separate venv installs robosuite from the local upstream checkout
`5ce6643f3092639d08f7b0f90ed1c6a84f50552c` (also upstream HEAD when checked),
and MuJoCo 3.9. The current DimOS `sim` extra requires MuJoCo >=3.10, whereas
this robosuite revision declares <3.10. This is a real packaging conflict.

Other DimOS dependencies are read from the original venv through the local
`dimos_spike_dependencies.pth`; original packages are not modified. Invoke the
experiment from this worktree with `PYTHONPATH=.`. This is an isolated runtime
probe, not evidence of a clean dependency-resolved DimOS installation.

## Findings

**Robosuite can host this hardware-emulation direction. It is not inherently
incompatible with arbitrary robot models or custom sensors. This experiment
does not establish that replacing sim2 with it reduces the required code.**

Two deliberately separate paths were exercised:

```text
Existing G1 RobotConfig + scene
  -> existing sim2 model composition
  -> MotorEnvironment (robosuite MujocoEnv subclass)
       physics stepping, resets, camera rendering, Observables
       DimOS motor adapter + PD/effort bindings
       existing sim2 lidar raycaster
  <- unchanged DimOS G1GrootWBCTask, called by synchronous test driver

Stock robosuite Lift + XArm7
  -> FixedBaseRobot + CompositeController
  -> wrist RGB-D + added custom Observable
```

The first path bypasses robosuite's RobotEnv, RobotModel and composite-controller
ownership. No upstream code was patched. The second path actually uses those
standard robot/controller facilities, but does not drive them from DimOS.
Neither path is a deployed replacement blueprint.

### Measured Results

Captured locally on 2026-09-08. Full generated JSON, trajectories and rendered
images remain at `/tmp/robosuite-emulator-evidence`; this table records the
important evidence persistently. The small logistics world has 84 geoms with
G1, not a large furnished office.

| Probe | Result |
| --- | --- |
| G1 balance-command run, two cameras and lidar | 10 simulated seconds in 3.307 wall seconds, 3.02x real time; setup 0.764 s |
| Main RGB-D camera | 640x480 at 10 Hz, 100 captures; finite metric depth, nonblank image visually inspected |
| Additional pelvis camera | 320x240 RGB at 15 Hz, 149 captures due to sampling phase; configuration-only addition |
| Ideal lidar | 15,000-ray pattern at 10 Hz, 100 captures, 12,444 valid points in final scan |
| Combined policy/physics/sensor tick | p50 1.296 ms; p95 14.814 ms |
| G1 native versus robosuite control | All 271 recorded qpos, motor-target and root samples bit-identical on MuJoCo 3.9 |
| G1 sustained walking | Both control-only probes numerically failed at the same point, around 5.4 s; not a walking success |
| Stock XArm7, MuJoCo 3.9 | Environment, composite controller, wrist RGB-D and custom Observable with corruption all worked |
| Stock XArm7, MuJoCo 3.10 | Environment construction failed at the upstream `mj_fullM` call, matching its dependency cap |
| Focused bridge tests | Four passed: native step equivalence, reset/model reuse, motor stop sentinels, mounts/lidar geometry isolation |

The 10-second G1 run stayed upright, but drifted about 0.98 m horizontally with
zero commanded walking velocity. Minimum root height was 0.728 m. It therefore
proves sustained sensor/physics operation, not correct position holding. A
separate three-second sensor run (two seconds balance, one second walking)
completed, but does not establish sustained locomotion either.

The identical native/robosuite walking failure shows that this particular
failure is not introduced by swapping the stepping backend in this driver. Its
cause has not been diagnosed. The driver calls the real policy task but does
not reproduce ControlCoordinator arbitration, safety or scheduling; it holds
non-policy arm joints at their configured home positions. Do not infer from
this that the user's running G1 blueprint is broken.

The native comparison includes sim2 shared-memory publication while the
robosuite probe is in-process. Timing is not a transport-equivalent backend
speed comparison. No GUI, typed streams, network transport, multiplayer,
large-scene throughput or real-time scheduling was tested.

The stock xArm probe took ten zero-action steps. It did not solve Lift and did
not run the existing DimOS xArm manipulation blueprint. Its custom measurement
is a simple joint-norm Observable, not a physically realistic new sensor.

### Extension Workflow

For another compatible whole-body effort-actuated robot, the experiment takes
the existing `RobotConfig`: model, root, joint/actuator mapping, initial state,
gains and mounted sensors. It currently requires one robot, exactly the native
joint units, an IMU, and the existing sim2 composition conventions. It does
not prove arbitrary actuation modes, multiple robots, or a new locomotion
policy integration.

Adding a second already-supported camera used `RobotConfig.with_sensor(Camera(
..., Mount("pelvis", ...)))`; no environment subclass or renderer change was
needed. Existing lidar patterns likewise remain configuration. A genuinely
new sensor physics model still needs an implementation, mount/state lookup,
sample-rate wiring and a DimOS stream publisher. Robosuite's Observable wraps
that measurement and provides sampling/noise/delay machinery; it does not
provide realistic MID360 physics or the DimOS publisher automatically.

For the standard robosuite robot path, existing XArm7 was selected by name.
A new robot family would additionally need the appropriate upstream robot
model/runtime/controller declarations. The motor bridge deliberately avoids
those manipulation-oriented declarations; that is an architectural choice,
not proof they are impossible to extend.

### What This Would Replace

- Robosuite can own environment stepping, resets, camera access and Observable
  scheduling/noise/delay. Its standard arm environments also supply existing
  controllers and task machinery.
- DimOS still needs hardware-facing joint and sensor interfaces, policy
  integration, transport/lifecycle ownership, robot configurations, model/mount
  composition, and custom lidar/splat implementations.
- This bridge retains existing sim2 scene composition and lidar. It adds an
  adapter rather than demonstrating deletion of those systems.
- The raw-model override does not implement upstream task/XML replay contracts.
  Hosting G1 here does not automatically make RoboCasa tasks robot-independent.
- Current robosuite's MuJoCo version cap must be reconciled before ordinary
  installation into current DimOS. This is a localized compatibility problem,
  not an argument that the two architectures can never coexist.

**Recommendation:** keep this as a viable bridge experiment, not a migration
decision. A production decision needs an actual deployed G1 ControlCoordinator
run, a stock-arm DimOS control bridge, and a supported dependency combination.
Those are the remaining gates; this experiment deliberately does not broaden
into implementing them. There is not yet evidence that replacing the minimal
hardware emulator with robosuite would make adding sensors/robots simpler.

## Reproduce In This Worktree

These use the existing isolated venv and original read-only robot assets.
They are headless probes; no Rerun or native viewer is launched.

```bash
cd ~/Desktop/dimos-robosuite-emulator-spike

PYTHONPATH=. .venv/bin/python -m experiments.robosuite_emulator.demo_compare \
  --backend robosuite --assets-root "$HOME/Desktop/dimos" \
  --output /tmp/robosuite-emulator-evidence --label g1-standing \
  --seconds 10 --walk-speed 0 --sensors --extra-camera

PYTHONPATH=. .venv/bin/python -m experiments.robosuite_emulator.demo_compare \
  --backend stock --assets-root "$HOME/Desktop/dimos" \
  --output /tmp/robosuite-emulator-evidence

PYTHONPATH=. .venv/bin/python -m pytest \
  experiments/robosuite_emulator/test_bridge.py -q --noconftest -o addopts=''
```

For control equivalence, run the same demo with `--backend native` and then
`--backend robosuite`, no sensors, default walking speed, and distinct labels.
Compare `qpos`, `commands` and `root` arrays in each `trajectory.npz`. Both
probes are expected to exit nonzero on the recorded walking failure. Failed
runs are not performance results.

The additional ignored `.venv-mj310` is an intentionally unsupported diagnostic
environment for reproducing the stock controller failure. It is not a proposed
installation workaround. No original venv package, lockfile or upstream source
was modified.
