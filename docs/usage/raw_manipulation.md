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
                                  | typed commands / feedback
                           ManipulationControl
                                  | bounded IK / joint and gripper tasks
                           ControlCoordinator <-> robot adapter

Camera streams ----------> RawManipulationBridge --> JPEG / float32 depth
```

The bridge only decodes commands and encodes observations. The robot-side
`ManipulationControl` owns the robot model, FK/IK, frame transforms, motion limits,
command IDs, timeouts and measured completion. It reuses `PinkPoseTargetSolver`
and the existing coordinator tasks; Cartesian commands do not invoke high-level
skills or obstacle-aware planning.

The robot-only `xarm-sim` blueprint configures control and cameras. A suite sets
`raw_bridge=True, raw_interface="manipulation"`; the eval launcher appends the
generic bridge and assigns an isolated endpoint. `mcp-server` supplies lifecycle
readiness, but Pi's `no_dimos` mode receives `ROBOT.md` rather than MCP access.
The existing navigation interface remains the default for `raw_bridge=True`.

## Commands and frames

Subscribe before commanding. Publish JSON to `robot/arm/command/json`:

```json
{"id":"lift-1","kind":"delta","xyz":[0,0,0.05],"rpy":[0,0,0],"frame":"base"}
```

- `delta`: XYZ in metres and RPY in radians, relative to the measured pose when
  accepted. Translation uses fixed robot-base axes. Rotation left-composes
  `Rz(yaw) Ry(pitch) Rx(roll)`. Omitted vectors mean zero change, not an absolute
  zero orientation. Translation and rotation can both be commanded.
- `gripper`: normalized `opening`, from 0 closed to 1 open, not metres.
- `joints`: optional full ordered arm-joint `positions` in radians.
- `stop`: cancels arm motion and discards pending motion commands, retaining the
  gripper target. Stops bypass the bounded motion queue and are serviced before
  FK/feedback processing, including when feedback is stale or missing.

The default scene's robot base is at world z=0.12 m. Agent poses are relative to
that base, with explicit `quaternion_xyzw` fields. Points below the base have
negative Z. Subscribe to `robot/arm/info/json` for joint order and limits, and
`robot/arm/state/json` for measured joint/EE/gripper feedback.

Command status is on `robot/arm/status/json`: `running`, `succeeded`, `rejected`,
`cancelled`, `timed_out`, or `error`. IDs are idempotent for the controller's
lifetime. Identical retries return the recorded status; conflicting reuse is
rejected. Only one command executes at a time. Completion uses measured feedback.
Arm/gripper timeout defaults are 10/5 seconds, with a maximum of 30 seconds.
A grasp can block full gripper closure and time out; inspect the object and
measured opening rather than equating command completion with grasp success.

## Observations

| Topics (under `robot/`) | Contents |
| --- | --- |
| `camera/jpeg` | Wrist RGB, normally 15 Hz |
| `camera/depth_f32`, `camera/depth_info/json` | Aligned metric optical-Z depth and dimensions |
| `camera_info/json`, `camera_pose/json` | Wrist intrinsics and optical pose in robot-base frame |
| `overview/jpeg` | Fixed-camera RGB, normally 5 Hz |
| `overview/camera_info/json`, `overview/camera_pose/json` | Overview's own intrinsics and optical pose |
| `arm/info/json`, `arm/state/json`, `arm/status/json` | Limits, robot state and command results |

Depth is little-endian float32, row-major `(height, width)`, decoded with
`np.frombuffer(payload, dtype="<f4").reshape(height, width)`. It is optical-axis Z
in metres, not distance along the pixel ray. Mask nonfinite/nonpositive depths.
RGB/depth binary messages have a `{"t": unix_seconds}` attachment; match pose and
image timestamps within each camera. The overview has no depth and uses different
calibration. Internal camera poses use TF; full TF and object truth are not
forwarded. Wrist depth is live only; the eval recorder does not JPEG-encode it.

Metadata and unchanged status repeat at 1 Hz; status transitions are immediate.
Keep the latest samples and save image/depth bytes rather than printing arrays.

## Robot context

Suites can select a robot-only bundle with `robot_context=LfsPath(...)`.
`xarm7_agent_context` contains arm/gripper URDFs, portable meshes, licenses,
joint/frame information, TCP offsets and gripper geometry estimates. Pi and
dimcode stage only checksum-manifest-listed files into the case's `robot/`
directory. Scene XML, object poses and grader state are not included.

The gripper's analytic jaw-gap estimate is nonlinear (approximately 1.6-88.9 mm),
not a hardware specification or dynamic contact calibration. Commands remain
normalized opening fractions. Read the short context once and consult URDFs
selectively; use live observations for current state.
