# Robosuite-Backed sim2 Comparison

2026-09-09. Branch `test/robosuite-emulator-spike`, separate worktree
`~/Desktop/dimos-robosuite-emulator-spike`. Native baseline: `a82725e85`.
This is a working alternative for comparison, not an accepted migration.
The original worktree, user changes, running simulator and upstream source
were not modified.

## G1 GR00T: Full Blueprint Run (2026-09-30)

The existing G1 integration at `fe48ee062` now has a real deployed-blueprint
check, not just the earlier standing-policy harness. **No new robot definition,
blueprint, runtime changes, policy changes or upstream patches were needed.**

For a manual session in this worktree:

```bash
cd ~/Desktop/dimos-robosuite-emulator-spike
uv run --no-sync python -m dimos.sim2.demo_smoke g1 \
  --local-router --viewer --rerun --seconds 3600
```

This loads the actual `unitree-g1-groot-wbc` blueprint and its logistics scene.
Use the existing Rerun drive controls; the MuJoCo window displays the world.
No `--move` means the launcher sends no automatic walking commands. The
duration is a one-hour inspection limit, not a sampling/reset interval.
Close the Rerun window before Ctrl-C; see the shutdown caveat below.

The existing smoke launcher creates a private local Zenoh router using normal
configuration, avoiding this branch's stock Zenoh 1.9/macOS discovery problem.
It is a direct Python blueprint launch, not a registered `dimos run` session.
`uv run --no-sync` uses the already provisioned worktree environment and puts
its viewer binaries on PATH. Run `uv sync --extra sim --inexact --frozen` first
on a fresh checkout. No transport implementation was patched.

The exercised control path is:

```text
Rerun twist message -> tele_cmd_vel -> MovementManager -> cmd_vel
  -> ControlCoordinator / G1GrootWBCTask at 50 Hz
  -> WholeBodyAdapter / existing motor SHM
  -> MotorLegged / MotorFirmware / EmulatorEnvironment.step at 200 Hz
  -> MuJoCo joint + IMU feedback -> the same GR00T policy

World snapshots -> independent RGB-D / lidar workers
                -> ordinary DimOS streams -> mapping / Rerun
```

Measured on the local Mac, with already downloaded assets:

| Run | Startup | Real-time factor | Result |
|---|---:|---:|---|
| Headless, 10 simulated seconds, `--move` | 7.45 s | 0.99975 | Walked; 640x480 RGB-D and lidar published; exited normally |
| Rerun + MuJoCo viewer, 45 simulated seconds | 5.46 s | 0.99989 | Rerun connected; control-endpoint messages moved G1 about 1.2 m; final pelvis height 0.745 m |

The visible run received eight seconds of forward commands, four seconds of
turn commands and then stop, through the same WebSocket endpoint used by the
Rerun controls. Translation and final standing height were measured; yaw
tracking accuracy was not. Mapping/planning modules deployed, but this was
not an autonomous-navigation or task-success test. These short runs do not
establish long-run stability, stairs or a speed advantage over native sim2.

Issues actually encountered:

- Running `.venv/bin/python` directly without activating the environment left
  `dimos-viewer` and `rerun` off PATH. Using `uv run --no-sync` fixed the viewer
  launch without a code change. Viewer connection is confirmed by the server;
  desktop capture did not expose the windows, so no visual-inspection claim.
- With Rerun left open, the simulation workers shut down, but the launcher
  waited during Python interpreter finalization in `waitpid`. Closing only
  that run's detached Rerun process let it exit with status zero. This is an
  open process-lifetime issue on the older comparison branch, not a stalled
  physics loop. It was not hidden with forced parent exit or a new cleanup
  abstraction.
- Optional robosuite-models/GR1-IK warnings, macOS GLFW/Open3D duplicate-class
  warnings and empty Rerun-root warnings remain. They did not prevent this
  G1 run. G1 uses DimOS's existing GR00T policy, not upstream GR1 IK.

The robot still has its authored rigid hands. The two massless wrist frames
are robosuite end-effector metadata, not working grippers. This proves G1
hardware emulation with robosuite's robot/environment lifecycle, not automatic
compatibility with arbitrary RoboCasa tasks. The native core worktree was
not changed; this prototype still predates its configuration/shutdown fixes.

## What Changed

The earlier raw-MjModel probe is retained in Git at `d92e83f0f`. It did not
demonstrate robot/model/controller adoption and is removed from the current
tree rather than kept as a second implementation. This version changes the
actual sim2 production path:

```text
Existing DimOS blueprint / ControlCoordinator / policy
  -> unchanged WholeBodyAdapter or ManipulatorAdapter / SHM
  -> SimulationRuntime: lifecycle, commands, snapshots, scene RPCs
  -> EmulatorEnvironment (ManipulationEnv -> RobotEnv -> MujocoEnv)
       Task composes SceneModel + robot models + optional upstream objects
       FixedBaseRobot / LeggedRobot assemble model + grippers and bind references
       Upstream robot reset, joint/EEF observables and action slicing
       MotorRobot mixin supplies the DimOS motor-controller extension
       MotorFirmware (registered CompositeController) applies motor commands
       MujocoEnv.step/reset owns simulation advancement
  -> unchanged independent camera/lidar workers and typed DimOS streams
```

No custom robot registry, second task system, persistent converted asset
format, physics fallback, new transport, or duplicate blueprint was added.
Robot definitions are classes registered by robosuite. Their existing joint
maps, gains, units, sensor mounts and SHM contracts remain DimOS declarations.
Motor firmware applies PD once to effort actuators, or forwards native
position-servo targets; policy/IK ownership stays outside the emulator.

The motor-only comparison is retained at `183d854f0`. This second pass adopts
the higher-level robot conventions: `MotorManipulator` extends `FixedBaseRobot`,
and `MotorLegged` extends `LeggedRobot`. Both use the same motor-command mixin;
they are robot families, not competing runtimes. The environment now inherits
upstream robot observations, resets, action slicing and manipulation helpers.

There is one definition per physical robot:

- G1 declares two seven-joint arms, torso and legs. Its existing rigid hands
  remain rigid. Massless wrist frames provide EEF metadata, not fingers or a
  grasping capability. Upstream NullGripper would add 0.3 kg and is not used.
- M20 declares legs and wheel/base actuators, with no invented manipulator.
- xArm declares one arm and a real `GripperModel` with finger-pad groups,
  grip site and actuator bindings. Both components come from the **same existing
  xArm MJCF**, with no second asset copy or task-specific robot variant.

Source assets, control policies, blueprints and sensor workers are unchanged.
Only `DIMOS_MOTORS` is supported by these hardware adapters; swapping in stock
OSC/IK is not established by this experiment.

```text
dimos/sim2/
  models.py                 # native-input preservation and model base classes
  robot.py                  # upstream fixed/legged robot extensions
  environment.py            # upstream composition and robot/task lifecycle
  runtime.py                # existing continuous SHM/scene/snapshot ownership
  control/firmware.py        # motor semantics, once
  assets/end_effector_frame.xml  # massless metadata, no robot geometry
dimos/robot/
  unitree/g1/sim2.py         # G1 model, body parts and hardware configuration
  deeprobotics/m20/sim2.py    # M20 model, body parts and hardware configuration
  manipulators/xarm/sim2.py  # xArm model, gripper component and hardware configuration
```

### API And Extension Example

An existing robot's definition changes from a model path to a small model
class; asset paths live in that robot-local class. Examples are the three
robot-local `sim2.py` files. G1, M20 and xArm blueprints need no new copies.
An upstream model can instead be subclassed directly, as the Panda example
does, with our instance namespace and joint/actuator mappings.

```python
from robosuite.models.objects import BoxObject
from dimos.sim2.blueprint import simulation_blueprint
from dimos.sim2.scene import scene_path
from dimos.sim2.sensors.spec import Camera, Mount
from dimos.sim2.spec import ObjectInstance, RobotInstance
from dimos.robot.manipulators.xarm.sim2 import XARM7

arm = XARM7.with_sensor(Camera("overview", Mount("link_base"), depth=False))
blueprint = simulation_blueprint(
    scene=scene_path(None, "workbench.xml"),
    robots={"arm": RobotInstance(arm, xyz=(0, 0, 0.12))},
    objects=(
        ObjectInstance(BoxObject, "box", (0.4, 0, 0.8),
                       {"size": [0.03, 0.03, 0.03], "rgba": [1, 0.1, 0.1, 1]}),
    ),
    viewer=False,
)
```

The object becomes an ordinary named sim2 scene entity: inspect, relocate
and reset it using the existing scene API. It is not a generated task or a
success predicate. Robot camera addition still changes configuration only;
new lidar physics still needs a sensor implementation.

`test_extensions.py` exercises an upstream Panda, BoxObject, CylinderObject,
mounted wrist camera, DimOS joint commands, robosuite Observable, object
editing/reset without recompilation, blueprint parsing, and construction
inside a real DimOS forkserver worker. The worker test uses the Actor pipe,
not cross-worker Zenoh RPC.

### Same Robot In An Original Upstream Task

No copied Lift implementation, custom placement sampler or replacement oracle:

```python
import robosuite
from dimos.robot.manipulators.xarm.sim2 import XARM7
from dimos.sim2.robot import motor_controller_config

env = robosuite.make(
    "Lift",
    robots=XARM7.model.__name__,
    controller_configs=motor_controller_config(XARM7),
    base_types="NullBase",
    has_renderer=False,
    has_offscreen_renderer=False,
    use_camera_obs=False,
    hard_reset=False,
    lite_physics=False,
    renderer="mujoco",
    seed=42,
)
try:
    observation = env.reset()  # Upstream samples the cube placement.
    motors = env.robots[0].composite_controller.home.copy()
    motors[0, 0] = 0.2  # First joint position; columns are q, dq, kp, kd, tau.
    for _ in range(5):
        observation, reward, done, info = env.step(motors.ravel())
    print(observation["cube_pos"], observation["0/eef_pos"])
finally:
    env.close()
```

`dimos/sim2/test_robot.py` proves different sampled placements without model
replacement, joint-command movement, standard joint/EEF observations, negative
and positive grasp-contact checks, and the original Lift success/reward check.
The positive contact and success states are deliberately placed by the test:
**this is task interoperability, not a policy solving Lift**. G1/M20/xArm also
exercise their upstream observations and reset lifecycle through normal sim2.
Two xArms retain independent names, cameras and motor actions in one world.

The Lift example runs the upstream environment directly. It does not deploy
ControlCoordinator or add task selection to the sim2 blueprint. Connecting a
chosen upstream task environment to that continuous device runtime remains a
separate integration step; arbitrary RoboCasa tasks are not claimed to work.

RobotConfig's only new field in this pass is `Joint.gripper`: `None` means the
robot model owns the joint; `"right"` names the model's composed gripper. It
resolves joint/actuator namespaces once. The public DimOS control and scene
interfaces remain unchanged. Native robosuite observations are available to
task code; they are not automatically new DimOS typed streams.

## Asset Input Boundary

Raw upstream RobotModel loading changes G1 damping/armature, recolors collision
geometry, and cannot resolve xArm's nested defaults. The three counterexamples
in `test_model_import.py` remain as evidence, not accepted behavior.

`models.py` uses MuJoCo's own XML serialization to resolve includes, angle
units, implicit names and actuator shortcuts. It expands inherited defaults
before robosuite composition, preserves authored appearance, and suppresses
unrequested upstream joint retuning. Meshes/textures remain shared files
referenced by absolute path; only temporary XML is produced.

Native `MjSpec.to_xml()` compiles internally. Consequently this path adds
cold model preparation, especially costly for a large scene. It also rounds
XML numbers to six significant digits; this is **not a universally lossless
MJCF importer**. A 90-degree test differs by approximately 3.7e-6 radians.
Real G1, M20 and xArm parameter tests cover masses/inertias, joint dynamics,
actuators, contacts and materials at 1e-6 tolerance. No arbitrary-model or
cross-MuJoCo-version equivalence is claimed.

## Measured Comparison

Same `demo_runtime.py`, same interpreter/MuJoCo 3.9 and input workloads;
`PYTHONPATH` selects native or experimental sim2. Both paths use the real
SHM adapters and independent sensor model snapshots. Sensors are captured
synchronously in this harness, not timed through deployed worker streams.
Setup times exclude Python imports and policy loading. These are short local
measurements, not platform throughput guarantees.

| Workload | Native setup | Robosuite setup | Native run | Robosuite run |
|---|---:|---:|---:|---:|
| G1 GR00T standing, logistics, 5 simulated seconds | 0.089 s | 0.206 s | 1.002 s | 1.122 s |
| M20 motor commands, logistics, 2 simulated seconds | 0.036 s | 0.134 s | 0.428 s | 0.438 s |
| xArm joint/gripper commands, workbench, 2 simulated seconds | 0.029 s | 0.120 s | 0.162 s | 0.170 s |
| G1 GR00T, populated RoboCasa kitchen, 1 simulated second | 0.348 s | 0.691 s | 0.567 s | 0.634 s |
| G1 GR00T, HSSD home, 1 simulated second | 4.645 s | 11.672 s | 2.593 s | 2.719 s |

All rows include 640x480 RGB-D at 10 simulated Hz; G1/M20 also use 15,000-ray
lidar at 10 simulated Hz. HSSD is slower than real time in **both** synchronous
runs, dominated by capture (2.53/2.59 s). Robosuite does not fix that bottleneck.
These are fresh sequential measurements for this second pass, not numbers
copied from the motor-only commit. No cache was introduced to hide preparation.
G1's median per-step time rises from 0.110 to 0.225 ms; total measured run time
rises about 12%. This includes the upstream lifecycle/observation bookkeeping.
It is a functionality tradeoff, not a performance improvement.

Behavior evidence:

- G1: all recorded qpos, qvel, actuator commands and final RGB/depth arrays
  match exactly in logistics, RoboCasa and HSSD. Minimum logistics root height
  is 0.728 m in both. This proves the bounded standing run, not position-hold
  quality, sustained locomotion, stairs or a full ControlCoordinator deployment.
- xArm: maximum qpos difference 2.9e-15, qvel 6.4e-14, actuator control 5.7e-14.
- M20: maximum qpos difference 4.7e-7, qvel 5.9e-5, actuator control 1.8e-6.
  Small numerical differences remain; no bit-identical M20 claim or locomotion
  policy acceptance is made.
- Final RGB/depth are identical for all three devices. The G1 comparison runs
  the standing policy; it does not use an uncontrolled falling robot as proof.
- G1 logistics produces 50 RGB-D frames and 50 lidar scans in five simulated
  seconds; final image standard deviation 77.24, 152,154 valid depth pixels,
  13,925 lidar returns. RoboCasa/HSSD images are nonblank; RoboCasa was visually
  inspected in the earlier comparison. New captures and traces are in
  `/tmp/sim2-robosuite-conventions`; no new screenshot inspection is claimed.

### Verification And Dependencies

- 41 focused tests passed in 5.91 s, covering the existing sim2 tests plus
  full model reassembly, upstream robot observations, two-robot action slicing,
  original Lift sampling/contact/oracle behavior and upstream counterexamples.
- Strict mypy passed on the nine production files affected by this pass with
  `--no-incremental --follow-imports=silent`; robosuite is an untyped dependency.
- Ruff checks pass. Dependency resolution and `uv sync --extra sim --inexact
  --frozen` were verified in the preceding comparison; this pass changes no
  dependencies and uses that normal installed venv, not a cross-venv `.pth`.
- robosuite is pinned to `5ce6643f3092639d08f7b0f90ed1c6a84f50552c` (1.5.2).
  Its `<3.10` cap changes this branch's sim requirement to MuJoCo 3.9;
  the resolver also lowers the dm-control and mujoco-mjx versions. No upstream
  patch is used. Moving all DimOS back to this version is a team decision.
- In the September 9 comparison, cross-worker RPC timed out on macOS with
  stock Zenoh 1.9 loopback discovery. Model/config deployment itself succeeded.
  The September 30 full G1 runs above supersede that network-test gap using
  the existing explicit local-router configuration. Default multicast
  discovery remains unverified; no transport code was patched.

## Code Size

Count physical Python lines, including headers/blanks, in `dimos/sim2` plus
the G1/M20/xArm robot-local definitions; exclude tests and `demo_*` scripts.

| | Native | Robosuite alternative |
|---|---:|---:|
| Production Python files | 28 | 32 |
| Production lines | 3,275 | 4,060 |

Net **+785 Python lines (24.0%)** compared with native sim2. The previous
motor-only experiment was 3,684 lines; adopting these conventions adds **376**
net production Python lines across nine existing files, plus one 19-line XML
metadata asset. No production Python file was added in this second pass.
The old composer is removed, not retained beside robosuite.

| Responsibility | Change |
|---|---:|
| Model input preservation and component metadata (`models.py`) | +294 |
| Environment composition/lifecycle (`environment.py`) | +177 |
| Upstream robot-family extensions (`robot.py`) | +135 |
| Registered motor firmware (`control/firmware.py`) | +104 |
| Existing runtime, scene resolver, spec and blueprint | -88 net |
| Three robot-local definitions | +163 net |

Tests, docs, comparison harness, dependency lock and XML are excluded from that
Python count. This pass adds `test_robot.py` and changes four existing test
files. The old 699-line raw-model probe was deleted in the previous comparison;
that deletion is **not** counted as an emulator code reduction.

## What It Buys And What It Does Not

**Demonstrated:** real upstream arm/gripper composition, reference binding,
robot observations, reset/action lifecycle, reusable Panda/object classes,
and original Lift placement/contact/success machinery with our existing xArm.
The same robot definition still serves the DimOS device and sensor interface.

**Still ours:** motor bindings/firmware, real-time process ownership, SHM,
sensor workers, lidar/splat physics, typed streams, scene semantics and RPCs.
The cameras still use sim2's existing renderer; Observable noise/delay is not
automatically applied to those independent sensor streams.

**Potential, not demonstrated:** other upstream tasks, stock OSC/IK controllers,
wrappers and datasets. RoboCasa tasks still
carry robot/gripper/controller assumptions. Importing robosuite does not make
those tasks or their success logic work on every DimOS robot.

**Decision boundary:** the second pass now demonstrates task reuse, rather
than asking the team to accept an abstract ecosystem promise. It is still not
a smaller or faster hardware emulator. Native sim2 remains unchanged; adoption
is a team decision about this concrete tradeoff. Resolve cold-start cost,
asset-input guarantees and the dependency policy before choosing this path.

## Reproduce

```bash
cd ~/Desktop/dimos-robosuite-emulator-spike
uv sync --extra sim --inexact --frozen

PYTHONPATH=. .venv/bin/python experiments/robosuite_emulator/demo_runtime.py \
  --robot g1 --groot --sensors --seconds 5 --output /tmp/comparison/robosuite-g1

PYTHONPATH="$HOME/Desktop/dimos" .venv/bin/python \
  experiments/robosuite_emulator/demo_runtime.py \
  --robot g1 --groot --sensors --seconds 5 --output /tmp/comparison/native-g1

PYTHONPATH=. .venv/bin/python -m pytest dimos/sim2 \
  experiments/robosuite_emulator/test_extensions.py \
  experiments/robosuite_emulator/test_model_import.py \
  -q -o addopts='' -m '' --timeout=45
```

Use `--robot m20` or `--robot xarm` without `--groot` for motor comparisons;
add `--scene robocasa-kitchen-1` or `--scene hssd-home` to the G1 command.
The baseline worktree must remain at the compared implementation for matching
results. Scripts are headless and use unique SHM names; they do not restart a
user's blueprint. Each run writes JSON timings and NPZ model/state/RGB-D arrays.
