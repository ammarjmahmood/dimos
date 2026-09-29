# sim2 Hardware Emulator

The existing G1 GR00T and xArm7 planner blueprints can use `sim2` in this branch.
Their controllers remain ordinary DimOS modules. MuJoCo owns physics; robot
control uses shared memory. Cameras and lidar run independently of physics.
PimSim, task catalogs and population preparation are not involved.

## Run

```bash
uv run dimos --simulation mujoco --transport zenoh --viewer rerun --scene-package kitchen run unitree-g1-groot-wbc
uv run dimos --simulation mujoco --transport zenoh --scene-package kitchen run xarm7-planner-coordinator
```

Run one stack at a time on the default transport bus. Both open the native
MuJoCo viewer. Disable it with the module override
`--simulationmodule.viewer=false` after the blueprint name. A scene name, an
absolute XML path, or a directory containing `scene.xml` uses the same loader.
Defaults without `--scene-package` remain the small logistics/workbench scenes.

The first download of existing robot meshes and GR00T policies is separate
from measured startup. Install the existing simulation and robot dependencies.
The simulation extra requires MuJoCo 3.10 or newer for batched raycasting.

This branch uses main's Zenoh dependency; no locally patched wheel is required.
Bounded device checks can use an explicit local router:

```bash
uv run python -m dimos.sim2.demo_smoke g1 --local-router --viewer --seconds 15 --move
uv run python -m dimos.sim2.demo_smoke xarm --local-router --viewer --seconds 15 --move
```

Add `--rerun` to include the Rerun bridge and viewer. These checks use the same
configuration parser as the CLI before deploying workers. Omit `--move` to
leave the robot holding its starting pose. The router is test setup, not a
second control path: joint commands still cross the same SHM device interface.

On the September 29 split onto main, the local-router headless smoke runs
reported G1 startup at 4.03 s and 0.999x real time, with GR00T walking and
640x480 RGB-D. xArm startup was 19.41 s and 0.999x real time, with RGB-D and
a completed joint command. These are short local measurements with assets
already downloaded, not cross-machine performance guarantees. The xArm run
also exposed stale-TF warnings during manipulation-module shutdown and a
worker stop timeout; that lifecycle issue remains open. G1 shut down cleanly.

## Included Scenes

The eight populated scenes ship together in the existing `data/.lfs/sim2.tar.gz`
data package. Shared mesh/texture files live once in `scenes/_assets`; no
PimSim install, bundle-path environment variable, or cooking step is needed
to run them. The existing DimOS LFS mechanism obtains/extracts the archive.

| Scene name | Named entities | Movable bodies | Fixture joints | Named spawn supports |
|---|---:|---:|---:|---|
| `kitchen` | 22 | 5 | 1 | `default`, `workbench` |
| `libero-kitchen-1` | 14 | 2 | 3 | `default` |
| `libero-kitchen-9` | 15 | 3 | 1 | `default`, `workbench` |
| `robocasa-kitchen-1` | 47 | 3 | 45 | `default` |
| `robocasa-kitchen-7` | 124 | 3 | 100 | `default` |
| `ithor-kitchen` | 84 | 28 | 25 | `default` |
| `procthor-house` | 90 | 51 | 15 | `default` |
| `hssd-home` | 232 | 0 | 0 | `default` |

HSSD is a furnished rigid navigation scene. Its furniture is not graspable.
The RoboCasa entries include three added movable mesh objects on an authored
counter. ProcTHOR is a multi-room house. These are scene imports, not claims
of passing the upstream benchmarks. Imported region labels are retained;
`kitchen` also contains explicitly authored regions for scene-control examples.

Scene spawns are support poses, independent of robot identity.
G1 selects `default` and adds its robot definition's `spawn_height` (0.793 m).
An arm selects `workbench` with zero offset. For example, a floor at Z=-1.5
places G1's root at -0.707.
Missing named supports fail clearly; an arm is never silently placed on a
floor. Raw scenes without spawn metadata use the blueprint's explicit support
default. Direct `RobotInstance` and live pose edits still use absolute root
poses. Scene metadata uses this single convention.

Old `office` is not an alias for one of these scenes: its
legacy collision wrapper still needs a separate visual/entity conversion.

```python
from dimos.sim2.scene import list_scenes

print(list_scenes())
```

The supplied scenes are finished native files. Edit their `scene.xml` and
`scene.json` directly; no preparation script or PimSim source library is
required. Source provenance is retained in each `scene.json`; source content
licensing remains subject to the original datasets' terms.

## Configure A Robot

Robot-local definitions live in:

- `dimos/robot/unitree/g1/sim2.py`: model, joint order, gains and named sensor bindings.
- `dimos/robot/manipulators/xarm/sim2.py`: native servos, gripper units and camera.

An existing blueprint selects simulated devices or real devices. It keeps its
controller, planner, perception and navigation modules:

```python
from pathlib import Path

from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.manipulators.common.blueprints import coordinator, trajectory_task
from dimos.robot.manipulators.xarm.sim2 import XARM7
from dimos.sim2.blueprint import simulation
from dimos.sim2.spec import RobotInstance

devices = simulation(
    scene=Path("/absolute/path/to/scene.xml"),
    sim_id="workbench",
    robots={"arm": RobotInstance(XARM7, xyz=(0, 0, 0.12))},
)
hardware = devices.hardware["arm"]
app = autoconnect(
    devices.blueprint,
    coordinator(hardware=[hardware], tasks=[trajectory_task(hardware)]),
)
```

Adding a robot with supported controls/sensors means adding its `sim2.py`
definition and changing its existing blueprint's device selection, plus a
robot contract test and assets. There is no central robot-name switch.

Stock sensors bind named cameras or sites in the native asset. Their positions
and orientations are not repeated in Python. A named camera's field of view
comes from its compiled asset. Select output size, rate and depth in Python:

```python
from dimos.sim2.sensors.spec import Camera

XARM7_SMALL_RGB = XARM7.with_sensor(
    Camera("wrist_camera", camera="wrist_camera", width=320, height=240, depth=False),
)
```

G1 uses `Imu("imu", site="control_imu")` and a lidar bound to `mid360_link`.
These sites are explicit in the MJCF;
G1's base-frame policy IMU is distinct from the torso hardware IMU.

An additional sensor can explicitly define a new mount on an existing body:

```python
from dimos.sim2.sensors.spec import Camera, Mount

XARM7_FRONT = XARM7.with_sensor(
    Camera("front", Mount("link_base", xyz=(0.1, 0, 0.3)), depth=False),
)
```

Names select sensor instances. A missing named camera/site/body fails during
composition, never triggering a replacement attachment. `Mount` means an
explicitly added device, not an alternate interpretation of a missing name.
Mount rotations use roll/pitch/yaw radians in the named body's local frame;
cameras use MuJoCo's -Z viewing direction and publish an optical-frame TF.
RGB-only and RGB-D modules have different declared ports. Repeated cameras
use `robot/sensor/port` names; multiple robots also namespace device ports.

The composition result is only data: `.blueprint` declares the world and
devices, while `.hardware` gives ControlCoordinator the matching adapters.
Neither `RobotConfig` nor `Simulation` is a running module. G1 uses four
emulator modules (physics, connection, camera, lidar); xArm uses three.
Physics and camera/lidar have dedicated workers. Each sensor worker holds a
local model/data copy, so isolation has a memory and state-reconstruction cost.

The existing GR00T, xArm7 planner and coordinator-xarm7 entrypoints
use this path. Old G1 vendor-action and other xArm perception/room/teleop,
xArm6 and Piper simulator paths are not yet all migrated. They are not a
fallback inside the migrated blueprints. The G1 vendor-action capability
requires an explicit retirement or preservation decision before replacing it.

Lidar configurations reference a concrete model factory, not an instance or
registered name. The worker constructs it once using its settings:

```python
from dimos.sim2.sensors.lidar.models.fibonacci import Fibonacci
from dimos.sim2.sensors.spec import Lidar

Lidar("lidar", "mid360_link", model=Fibonacci, model_kwargs={"ray_count": 15000})
```

New ideal ray patterns implement the `RayPattern` contract; they need no
model-name registry. World, robot and sensor settings are typed dataclasses.
The normal blueprint parser merges their settings as dictionaries, and module
configuration reconstructs the declared types. Python factory references
survive this process, just like custom IK solver classes elsewhere in DimOS.
The adapter reconstructs the robot definition at the generic `adapter_kwargs`
boundary. There is no separate simulator configuration parser.

G1 uses `models.fibonacci.Fibonacci`: the previous PimSim 15,000-ray pattern
at 10 Hz, mounted at the calibrated upside-down MID360 pose. Its explicit
`maximum_world_elevation=0.0` discards upward rays after the mount transform.
This is an ideal mapping scan, without MID360 scan timing or noise. G1's
simulation costmap no longer forces a disk around world origin to be free.

## Runtime Ownership

`SimulationModule` owns one continuously stepping `SimulationRuntime` and
disposable model snapshot. Each camera/lidar worker loads its own query model
and receives stamped integration-state frames, not a new scene per frame.
All scene geom groups are visible to camera rendering; lidar excludes only
its own robot subtree. The native viewer reads the same state snapshots.

The whole-body channel contains complete position, velocity, gains and
feed-forward torque. The adapter latches joint and IMU data together per
coordinator tick. Native xArm servos retain their original actuator model;
the gripper retains the hardware API's 0-850 units. No second PD is applied.

## Scene Interface

Use the existing `Dimos.connect()` interface (or the same module proxies in
`dimos shell`). No separate simulator client or session object is required.
All poses use metres and world coordinates; quaternion order is XYZW.
Joint angles use radians, slide-joint positions use metres.

```python
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.porcelain.dimos import Dimos
from dimos.sim2.interaction import reset_scene
from dimos.sim2.scene_types import SceneUpdate

app = Dimos.connect()
sim = app.get_module("SimulationModule")
description = sim.describe_scene()
print(description.entities.keys(), description.joints.keys(), description.regions.keys())
state = sim.scene_state()
print(state.entities["block"].pose, state.regions["tray/interior"])

sim.set_scene_state(SceneUpdate(
    poses={"block": Pose(0.30, -0.16, 0.926)},
    joints={"cabinet-1/door-hinge": 0.8},
))
reset_scene(app)  # Captured scene defaults plus robot/controller homes.
app.stop()        # Disconnect; does not stop the running blueprint.
```

Complete operator RPC surface (in addition to normal Module lifecycle):

```python
status() -> dict[str, Any]
describe_scene() -> SceneDescription
scene_state() -> SceneState
set_scene_state(update: SceneUpdate) -> SceneState
reset(initial: SceneUpdate | None = None) -> SceneState
set_spawn(robot_id: str, xyz: tuple[float, float, float],
          rpy: tuple[float, float, float] = (0, 0, 0)) -> None
set_paused(paused: bool) -> None
set_truth_enabled(enabled: bool) -> None
```

`build()` and `describe()` are internal model/snapshot bootstrap RPCs used
by sensor workers, not another scene API.

The typed records are defined in `dimos/sim2/scene_types.py`:

| Record | Fields |
|---|---|
| `SceneUpdate` | `poses: dict[entity_or_robot_id, Pose]`, `joints: dict[fixture_joint_id, float]` |
| `SceneDescription` | `format`, `id`, `entities`, `joints`, `regions`, `initial`, `spawns`, `hidden_geom_groups`, `provenance` |
| `SceneEntity` | `body`, `label`, `kind`, `movable` |
| `SceneJoint` | `joint`, `entity`, `closed`, `opened` |
| `SceneRegion` | `body`, `kind` (support/containment/navigation), local `pose`, full `size` |
| `SceneState` | `world_id`, `scene_id`, `generation`, `tick`, `sim_time`, wall `ts`, `entities`, `robots`, `joints`, `regions`, `contacts` |
| `EntityState` | world `pose`, linear `velocity`, `angular_velocity`, world `bounds_min`, `bounds_max` |
| `RegionState` | world `pose`, full `size` |

`SceneState.robots` maps instance IDs to poses; contacts are pairs of entity
IDs or robot body names. Stable scene IDs are not MuJoCo array indices.
Robot joints remain on the ordinary control interface.

### Reset And Edit Rules

`set_scene_state` validates the whole update before mutation. It changes only
existing free/mocap bodies and declared scalar fixture joints, zeroes affected
velocities, advances the command generation and publishes immediately. Model,
viewer and sensor workers remain resident. Adding assets or changing structural
scene geometry requires a new run.

`sim.reset()` restores the captured authored physics baseline, then applies
optional overrides. Overrides do not redefine the baseline. It does not cancel
application goals. Use this application-side helper for a running stack:

```python
reset_scene(
    app: Dimos,
    initial: SceneUpdate | None = None,
    *, simulation: str = "SimulationModule",
    coordinators: Sequence[str] = ("ControlCoordinator",),
    before_reset: Sequence[Callable[[Dimos], None]] = (),
    after_reset: Sequence[Callable[[Dimos], None]] = (),
) -> SceneState
```

The helper waits for controller startup, pauses physics, cancels trajectories,
deactivates controllers, runs explicit cancellation hooks, resets physics and
controller histories, runs explicit post-reset hooks, then reactivates. Failure
leaves physics paused. The starter cases supply navigation/manipulation goal
cancellation. Mapping/perception histories require hooks from their actual
owners; they are not automatically inferred or cleared. Reset affects every
robot in the world. Moving an arm's physical base does not reconfigure its
planner, so retain its authored spawn for manipulation.

## Streams And Actions

| Module | Inputs | Outputs |
|---|---|---|
| Whole-body connection | `motor_command: MotorCommandArray` | `motor_states: JointState`, `imu: Imu`, `odom: PoseStamped`, `tf: TFMessage` |
| Manipulator connection | `joint_command: JointState` | `joint_states: JointState` |
| RGB camera | none | `color_image: Image`, `camera_info: CameraInfo`, `tf: TFMessage` |
| RGB-D camera | none | RGB ports plus `depth_image: Image`, `depth_camera_info: CameraInfo` |
| Lidar | none | `pointcloud: PointCloud2` |
| SimulationModule | none | optional `sim_truth: SceneState` at 10 Hz |

Motor command input streams are consumed only in explicit
`command_source="stream"` mode. The shipped GR00T/xArm coordinators use the
direct SHM hardware adapters; sensor streams and RPCs use ordinary transport.
Robot actions remain existing navigation/manipulation RPCs such as `set_goal`,
`plan_to_poses`, `execute`, and `set_gripper_position`.

## Evaluation Boundary

This branch provides physical scene controls and optional `sim_truth`, not a
second evaluation runner. Truth is disabled by default and is not exposed as
an agent skill or wired into perception. Main replaced `InteractiveEval` with
the `EvalCase`/environment API. The earlier three sim2 cases are preserved on
`feat/sim2-emulator-g1-xarm`; they are not runnable examples on this branch.
Adapting them to main's evaluation API is a separate follow-up.

## Scope Of This Checkpoint

Verified: G1 balance/walking, xArm joint control and gripper mapping, native
RGB-D, ideal instantaneous lidar, reset-frame invalidation, fixed-base
relocation, and independent two-robot channels/mounts.

The initial whole-body family requires one coherent control IMU. Standalone
IMU modules, timed MID360/Point-LIO input, splat rendering, automatic planner
scene obstacles, arbitrary live scene switching, full task generation/DR, and
the ten-minute latency/30-Hz-camera acceptance benchmark remain outside this
checkpoint. M20 and its coordinator extension remain a separate experiment.
Other robot blueprints remain on their existing backends until migrated.
