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
| `lidar` in | `lidar/xyz_f32` | lidar robots |
| `odom` in | `odom/json` | mobile bases (the MuJoCo sim also publishes one) |
| `coordinator_joint_state` in | `arm/state/json` | ControlCoordinator arms |
| `tf` in, configured frames only | `camera_pose/json`, `ee_pose` in `arm/state/json` | when configured |
| `cmd_vel` out | `cmd_vel/json` | mobile bases |
| `ee_twist_command` out | `arm/twist/json` | arms with an `eef_twist` task |
| `gripper_command` out | `arm/gripper/json` | arms with a gripper task |

Arm commands drive the coordinator's existing `eef_twist` and gripper tasks, the
same streams keyboard and hosted teleop publish. The bridge loads no robot model.

Velocity commands are held for their `t` seconds (capped) and republished, then a
single zero is sent: the same deadman for the base and the arm. Gripper openings
(0 closed, 1 open) pass straight through. Malformed commands are dropped.

Robot-specific settings default to off and are set per suite through `module_env`:
`RAWROBOTBRIDGE__CAMERA_FRAME`, `__EE_FRAME`, `__GRIPPER_JOINT` and
`__GRIPPER_RANGE`. The measured TCP pose comes from the coordinator, which publishes
its IK task's `link_tcp` on TF (`publish_frame_poses`) by forward kinematics on the
measured joints, so it works the same in simulation and on hardware.

## Describing the interface to the agent

A no-dimOS agent learns the robot from `ROBOT.md`, which Pi writes into the run
directory from a template, filling in the per-run endpoint and limits. The template
comes from the suite's `raw_guide`; when unset it is the default navigation guide
(`RAW_README`). `mujoco_xarm_raw` uses `RAW_XARM7_README`, which also covers the
gripper geometry. The case instruction carries only the task.
