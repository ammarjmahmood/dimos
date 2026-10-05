# Raw robot interface

Agents that run without dimOS (`--set no_dimos=true`) reach the robot through
`RawRobotBridge`: plain Zenoh topics carrying JSON, JPEG and float32 bytes, the
surface a vendor SDK would expose. One bridge serves every robot. It declares
every stream it understands as an optional input or output, so on each robot only
the streams that robot provides come alive.

Run the default-scene cylinder lift through it:

```bash skip
dimos evals run dimos.evals.suites.mujoco_xarm_raw \
  --agent dimos.evals.agents.pi --set no_dimos=true --set max_steps=120
```

The goal is to lift the cylinder at least 5 cm above its initial position, graded
with the existing recorded-body-pose scorer.

## Wiring

```text
Agent <-> isolated Zenoh endpoint <-> RawRobotBridge <-> robot streams (autoconnect)
```

| dimOS stream | Agent topic (under `robot/`) | Robots |
| --- | --- | --- |
| `color_image` in | `camera/jpeg` | any camera |
| `depth_image` in | `camera/depth_f32`, `camera/depth_info/json` | depth cameras |
| `camera_info` in | `camera_info/json` | any camera |
| `overview_image`, `overview_camera_info` in | `overview/jpeg`, `overview/camera_info/json` | a fixed workspace camera (MuJoCo `overview_camera_name`) |
| `lidar` in | `lidar/xyz_f32` | lidar robots |
| `odom` in | `odom/json` | mobile bases (the MuJoCo sim also publishes one) |
| `coordinator_joint_state` in | `arm/state/json` | ControlCoordinator arms |
| `tf` in, configured frames only | `camera_pose/json`, `overview/camera_pose/json`, `ee_pose` in `arm/state/json` | when configured |
| `cmd_vel` out | `cmd_vel/json` | mobile bases |
| `ee_twist_command` out | `arm/twist/json` | arms with an `eef_twist` task |
| `gripper_command` out | `arm/gripper/json` | arms with a gripper task |

Arm commands drive the coordinator's existing `eef_twist` and gripper tasks, the
same streams keyboard and hosted teleop publish. The bridge loads no robot model.

Velocity commands are held for their `t` seconds (capped) and republished, then a
single zero is sent: the same deadman for the base and the arm. Gripper openings
(0 closed, 1 open) pass straight through. Malformed commands are dropped.

Robot-specific settings default to off and are set per suite through `module_env`:
`RAWROBOTBRIDGE__CAMERA_FRAME`, `__OVERVIEW_FRAME`, `__EE_FRAME`, `__GRIPPER_JOINT` and
`__GRIPPER_RANGE`. The xArm sim blueprint has MuJoCo publish its `link_tcp` site on
TF (`tracked_sites`) so the bridge can report the measured TCP pose.

## Describing the interface to the agent

The suite owns the description. `mujoco_xarm_raw` puts the topics, units and
command formats in the case instruction and sets `raw_guide=False`; the harness
adds only the per-run endpoint. Suites that leave `raw_guide` on get the default
navigation `ROBOT.md`.

## Robot context

Suites can select a robot-only bundle with `robot_context=local_robot_context(...)`,
which resolves `$DIMOS_ROBOT_CONTEXT_DIR/<name>`; with the variable unset the case
runs without it. The bundle is kept outside the repository for now. Pi stages only
checksum-manifest-listed files (URDFs, meshes, licenses, `robot_info.json`, a short
README) into the case's `robot/` directory. Scene XML, object poses and grader state
are never included.
