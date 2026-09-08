# Robosuite-Backed sim2 Comparison

2026-09-08. Branch `test/robosuite-emulator-spike`, separate worktree
`~/Desktop/dimos-robosuite-emulator-spike`. Native baseline: `a82725e85`.
This is a working alternative for comparison, not an accepted migration.
The original worktree, user changes, running simulator and upstream source
were not modified.

## What Changed

The earlier raw-MjModel probe is retained in Git at `d92e83f0f`. It did not
demonstrate robot/model/controller adoption and is removed from the current
tree rather than kept as a second implementation. This version changes the
actual sim2 production path:

```text
Existing DimOS blueprint / ControlCoordinator / policy
  -> unchanged WholeBodyAdapter or ManipulatorAdapter / SHM
  -> SimulationRuntime: lifecycle, commands, snapshots, scene RPCs
  -> EmulatorEnvironment (robosuite MujocoEnv)
       Task composes SceneModel + robot models + optional upstream objects
       MotorRobot (robosuite Robot) creates robot-local model via its factory
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

This deliberately subclasses the generic `Robot` and `MujocoEnv`, not
`FixedBaseRobot`/`RobotEnv`. Those higher-level manipulation assumptions do not
describe G1/M20 motor hardware. Our custom controller does not demonstrate
swapping in upstream OSC/IK or running RoboCasa task environments unchanged.

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
| G1 GR00T standing, logistics, 5 simulated seconds | 0.081 s | 0.103 s | 1.002 s | 1.058 s |
| M20 motor commands, logistics, 2 simulated seconds | 0.031 s | 0.047 s | 0.414 s | 0.412 s |
| xArm joint/gripper commands, workbench, 2 simulated seconds | 0.026 s | 0.035 s | 0.137 s | 0.151 s |
| G1 GR00T, populated RoboCasa kitchen, 1 simulated second | 0.376 s | 0.549 s | 0.571 s | 0.616 s |
| G1 GR00T, HSSD home, 1 simulated second, repeat | 4.642 s | 11.706 s | 2.668 s | 2.762 s |

All rows include 640x480 RGB-D at 10 simulated Hz; G1/M20 also use 15,000-ray
lidar at 10 simulated Hz. HSSD is slower than real time in **both** synchronous
runs, dominated by capture (2.60/2.65 s). Robosuite does not fix that bottleneck.
The first HSSD pair was 4.719/12.259 s setup; the repeat confirms the large
preparation penalty. No extra cache was introduced to conceal it.

Behavior evidence:

- G1: all recorded qpos, qvel, actuator commands and final RGB/depth arrays
  match exactly in logistics, RoboCasa and HSSD. Minimum logistics root height
  is 0.728 m in both. This proves the bounded standing run, not position-hold
  quality, sustained locomotion, stairs or a full ControlCoordinator deployment.
- xArm: maximum qpos difference 2.9e-15, qvel 6.4e-14, actuator control 5.7e-14.
- M20: maximum qpos difference 4.7e-7, qvel 5.9e-5, actuator control 1.8e-6.
  Small numerical differences remain; no bit-identical M20 claim or locomotion
  policy acceptance is made.
- Final RGB/depth are identical for all three devices. The uncontrolled G1
  motor-only run fell and produced a black final image in both versions; that
  is rejected as camera evidence and replaced by the standing-policy run.
- G1 logistics produces 50 RGB-D frames and 50 lidar scans in five simulated
  seconds; final image standard deviation 77.24, 152,154 valid depth pixels,
  13,925 lidar returns. RoboCasa/HSSD images are nonblank; RoboCasa was visually
  inspected. Saved captures and traces are in `/tmp/sim2-robosuite-comparison`.

### Verification And Dependencies

- 36 focused tests passed in 5.46 s, covering the existing sim2 tests plus
  real-model preservation, upstream extensions and upstream counterexamples.
- Strict mypy passed on the 11 affected production files with
  `--no-incremental --follow-imports=silent`; robosuite is an untyped dependency.
- Ruff checks pass. The lock resolves and `uv sync --extra sim --inexact
  --frozen` succeeds in the experimental venv. The earlier cross-venv `.pth`
  was removed; final tests use the normal installed dependency closure.
- robosuite is pinned to `5ce6643f3092639d08f7b0f90ed1c6a84f50552c` (1.5.2).
  Its `<3.10` cap changes this branch's sim requirement to MuJoCo 3.9;
  the resolver also lowers the dm-control and mujoco-mjx versions. No upstream
  patch is used. Moving all DimOS back to this version is a team decision.
- The ordinary cross-worker RPC attempt timed out on macOS with stock Zenoh
  1.9 loopback discovery. Model/config deployment itself succeeded. A separate
  forkserver/Actor test proves construction/reset; full network blueprint
  acceptance is **not** established. No transport workaround is included.

## Code Size

Count physical Python lines, including headers/blanks, in `dimos/sim2` plus
the G1/M20/xArm robot-local definitions; exclude tests and `demo_*` scripts.

| | Native | Robosuite alternative |
|---|---:|---:|
| Production Python files | 28 | 32 |
| Production lines | 3,275 | 3,684 |

Net **+409 lines (12.5%)**; 11 production files touched, including four new
ones, +569/-160. The old composer is removed, not retained beside robosuite.

| Responsibility | Change |
|---|---:|
| Model input preservation (`models.py`) | +147 |
| Environment composition/lifecycle (`environment.py`) | +165 |
| Generic robosuite robot (`robot.py`) | +75 |
| Registered motor firmware (`control/firmware.py`) | +92 |
| Existing runtime, scene resolver, spec and blueprint | -94 net |
| Three robot-local definitions | +24 net |

Separate from production: two affected colocated test files add 117 net lines;
the new extension test is 163 lines and the comparison harness 283. The old
699-line probe/test implementation is deleted; that deletion is **not** counted
as an emulator code reduction. Dependency lock, docs and historical import
counterexamples are also excluded from the production comparison.

## What It Buys And What It Does Not

**Demonstrated:** shared upstream robot/object model conventions, reusable
Panda/object classes, native robosuite composition and Observable extensions,
while keeping the same DimOS device interface and sensor configuration.

**Still ours:** motor bindings/firmware, real-time process ownership, SHM,
sensor workers, lidar/splat physics, typed streams, scene semantics and RPCs.
The cameras still use sim2's existing renderer; Observable noise/delay is not
automatically applied to those independent sensor streams.

**Potential, not demonstrated:** adopting more upstream placement samplers,
task environments, controllers, wrappers and datasets. RoboCasa tasks still
carry robot/gripper/controller assumptions. Importing robosuite does not make
those tasks or their success logic work on every DimOS robot.

**Decision:** viable if adopting that ecosystem is a product objective, but
not currently a smaller or faster hardware emulator. For the firm V1 goal,
native sim2 remains the stronger default on these measurements. Asset-library
reuse may be valuable independently of adopting the entire environment loop;
that narrower approach was not implemented here. Do not migrate main based
only on this experiment. Resolve the startup cost, asset-input guarantee and
dependency policy explicitly if choosing the full robosuite path.

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
  -q --noconftest -o addopts='' -m '' --timeout=45
```

Use `--robot m20` or `--robot xarm` without `--groot` for motor comparisons;
add `--scene robocasa-kitchen-1` or `--scene hssd-home` to the G1 command.
The baseline worktree must remain at the compared implementation for matching
results. Scripts are headless and use unique SHM names; they do not restart a
user's blueprint. Each run writes JSON timings and NPZ model/state/RGB-D arrays.
