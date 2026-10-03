# Raw manipulation

Run the default-scene cylinder lift through plain robot topics:

```bash skip
dimos evals run dimos.evals.suites.mujoco_xarm_raw \
  --agent dimos.evals.agents.pi --set no_dimos=true
```

The goal is to lift the cylinder at least 5 cm above its initial position, graded
with the existing recorded-body-pose scorer. No robosuite assets are required.

## Architecture

```text
Agent <-> isolated Zenoh <-> RawManipulationBridge
                                  | pose / joint / native gripper targets
                           Existing ControlCoordinator tasks <-> robot adapter

Camera streams ----------> RawManipulationBridge --> JPEG / float32 depth
```

The bridge adapts inputs and encodes observations. A delta is resolved once from
measured EE feedback into an absolute target for the existing `CartesianIKTask`.
Joint targets use `JointTrajectoryTask`; native gripper positions use
`GripperControlTask.set_position()`. These tasks own the robot model, shared IK,
bounded updates and execution. The bridge loads no model or solver and does not
invoke high-level skills or obstacle-aware planning.
The simulated stack enables the Cartesian task's opt-in measured-feedback
correction to compensate servo tracking lag, within the same bounded IK update.

The robot-only `xarm-sim` blueprint configures control and cameras. A suite sets
`raw_bridge=True, raw_interface="manipulation"`; the eval launcher appends the
generic bridge and assigns an isolated endpoint. `mcp-server` supplies lifecycle
readiness, but Pi's `no_dimos` mode receives `ROBOT.md` rather than MCP access.
The existing navigation interface remains the default for `raw_bridge=True`.

## Commands and frames

Subscribe before commanding. Publish JSON to `robot/arm/command/json`:

```json
{"kind":"delta","xyz":[0,0,0.05],"rpy":[0,0,0],"frame":"base"}
```

- `delta`: XYZ in metres and RPY in radians, relative to the measured pose when
  accepted. Translation uses fixed robot-base axes. Rotation left-composes
  `Rz(yaw) Ry(pitch) Rx(roll)`. Omitted vectors mean zero change, not an absolute
  zero orientation. Translation and rotation can both be commanded.
- `gripper`: native `position` in the advertised range/units. In the simulator,
  0 is closed and 0.85 open in the adapter's radian opening coordinate, not jaw
  width. The URDF driver angle runs the other way: `q_driver = 0.85 - position`.
- `joints`: optional full ordered arm-joint `positions` in radians.
- `stop`: cancels arm motion and discards pending inputs, retaining the applied
  gripper target. Stops take priority over normal inputs and are serviced before
  FK/feedback processing, including when feedback is stale or missing.

The default scene's robot base is at world z=0.12 m. Agent poses are relative to
that base, with explicit `quaternion_xyzw` fields. Points below the base have
negative Z. Subscribe to `robot/arm/info/json` for joint order and limits, and
`robot/arm/state/json` for measured joint/EE/gripper feedback.

Commands are fire-and-forget. There are no IDs, acknowledgements or status replies.
The agent observes the continuous state/camera streams to decide whether to proceed.
The latest pending arm input wins and supersedes any active arm target; gripper
input is independent. Intermediate inputs may be replaced when sent rapidly.
Each new delta uses a fresh measured pose; resending is another input, not a retry.

Arm `timeout_s` is a bounded target lease (default 10 seconds, maximum 30). The
bridge renews the same absolute pose while the lease is live so the existing task
watchdog stays effective, without accumulating another delta or extending the
deadline. Lease expiry and stop cancel arm targets; gripper intent persists.
A blocked gripper keeps its target. Inspect measured position and object motion
to assess a grasp. Invalid inputs are logged and dropped, not acknowledged.

## Observations

| Topics (under `robot/`) | Contents |
| --- | --- |
| `camera/jpeg` | Wrist RGB, normally 15 Hz |
| `camera/depth_f32`, `camera/depth_info/json` | Aligned metric optical-Z depth and dimensions |
| `camera_info/json`, `camera_pose/json` | Wrist intrinsics and optical pose in robot-base frame |
| `overview/jpeg` | Fixed-camera RGB, normally 5 Hz |
| `overview/camera_info/json`, `overview/camera_pose/json` | Overview's own intrinsics and optical pose |
| `arm/info/json`, `arm/state/json` | Limits/units and continuous measured robot state |

Depth is little-endian float32, row-major `(height, width)`, decoded with
`np.frombuffer(payload, dtype="<f4").reshape(height, width)`. It is optical-axis Z
in metres, not distance along the pixel ray. Mask nonfinite/nonpositive depths.
RGB/depth binary messages have a `{"t": unix_seconds}` attachment; match pose and
image timestamps within each camera. The overview has no depth and uses different
calibration. Internal camera poses use TF; full TF and object truth are not
forwarded. Wrist depth is live only; the eval recorder does not JPEG-encode it.

Metadata repeats at 1 Hz; robot state is published at 20 Hz by default. Keep the
latest samples and save image/depth bytes rather than printing arrays. Camera TFs
must be in world or robot-base coordinates; unrecognized parents are not exported.

## Robot context

Suites can select a robot-only bundle with `robot_context=LfsPath(...)`.
`xarm7_agent_context` contains arm/gripper URDFs, portable meshes, licenses,
joint/frame information, TCP offsets and gripper geometry estimates. Pi and
dimcode stage only checksum-manifest-listed files into the case's `robot/`
directory. Scene XML, object poses and grader state are not included.

The gripper's analytic jaw-gap estimate is nonlinear (approximately 1.6-88.9 mm),
not a hardware specification or dynamic contact calibration. Commands remain
native position targets. Read the short context once and consult URDFs
selectively; use live observations for current state.
