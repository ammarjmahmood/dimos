# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Default xArm cylinder lift through the raw robot interface.

dimos evals run dimos.evals.suites.mujoco_xarm_raw --agent dimos.evals.agents.pi \
    --set no_dimos=true --set max_steps=120

The instruction below is the agent's whole description of the robot; the harness
adds only the per-run endpoint. Pi's default 40-step budget is mostly spent on
client setup and observation, so give it room to re-observe after each move.
"""

from dimos.evals.environments.mujoco_sim import MujocoEnvironment
from dimos.evals.robot_context import local_robot_context
from dimos.evals.suites.mujoco_xarm import lifted
from dimos.evals.types import EvalCase, Suite
from dimos.utils.data import LfsPath

TASK = (
    "Pick up the cylinder from the table and hold it in the air, at least 5 cm above its "
    "initial position."
)

INTERFACE = """\
The robot is an xArm7 with a parallel gripper and a wrist RGB-D camera, reachable as a
Zenoh peer at the endpoint given to you. Use a plain Zenoh client (for example
`uv venv .clientenv && uv pip install --python .clientenv/bin/python eclipse-zenoh numpy pillow`):
peer mode, connect directly to the endpoint, multicast and gossip scouting disabled, and
close the session before your script exits. Subscribe before commanding.

Observations (binary payloads carry an attachment {"t": unix_seconds}):
  robot/arm/state/json     {"t", "joint_names", "positions" (rad), "velocities" (rad/s),
                           "ee_pose": {"frame", "xyz", "quaternion_xyzw"} or null,
                           "gripper_opening" (0 closed .. 1 open)}, 20 Hz.
                           ee_pose is the measured tool centre point (TCP) in world.
  robot/camera/jpeg        wrist RGB, 15 Hz
  robot/camera/depth_f32   wrist depth aligned to the RGB: float32 little-endian
                           (height, width), optical-axis Z in metres
  robot/camera/depth_info/json  {"t", "width", "height", "dtype", "unit", "frame_id"}
  robot/camera_info/json   {"width", "height", "K"}
  robot/camera_pose/json   {"t", "frame", "xyz", "quaternion_xyzw"}: wrist optical pose
                           in world; optical +Z forward, +X image right, +Y image down
  robot/overview/jpeg      fixed external RGB camera viewing the whole workspace, 5 Hz,
                           no depth; use it to check an object after a grasp or lift
  robot/overview/camera_info/json, robot/overview/camera_pose/json
                           the overview's own intrinsics and pose (different from the wrist)

Commands (fire-and-forget: no IDs, acknowledgements or status):
  robot/arm/twist/json     {"vx", "vy", "vz" (m/s), "wx", "wy", "wz" (rad/s), "t" (s)}
                           TCP velocity about fixed world axes, held for t seconds
                           (max 2 s) and then stopped. A new twist replaces the previous
                           one; a zero twist stops. Components clamp to 0.1 m/s and
                           0.5 rad/s; omitted fields are zero.
  robot/arm/gripper/json   {"opening": 0..1}, 0 closed and 1 open. Persists until changed.
Malformed or out-of-range commands are dropped silently.

Motion is local IK tracking without collision checking. Distance is velocity x time and
only approximate, so check ee_pose and the cameras after every move. At the start pose the
wrist camera looks straight down: image right = world -Y, image up = world +X. A gripper
blocked by an object holds its target; judge a grasp from object motion, not closure.
Depth pixel (u, v) back-projects as z = depth[v, u], x = (u - cx) z / fx,
y = (v - cy) z / fy, then camera_pose takes it to world.
"""

SUITE: Suite = [
    EvalCase(
        id="xarm_raw_pick_cylinder",
        inputs=f"{TASK}\n\n{INTERFACE}",
        environment=MujocoEnvironment(
            blueprint=["xarm-sim", "mcp-server"],
            raw_bridge=True,
            raw_guide=False,
            module_env={
                "RAWROBOTBRIDGE__CAMERA_FRAME": "wrist_camera_color_optical_frame",
                "RAWROBOTBRIDGE__OVERVIEW_FRAME": "env_camera_color_optical_frame",
                "RAWROBOTBRIDGE__EE_FRAME": "link_tcp",
                "RAWROBOTBRIDGE__GRIPPER_JOINT": "arm/gripper",
                "RAWROBOTBRIDGE__GRIPPER_RANGE": "[0.0, 0.85]",
            },
            robot_context=local_robot_context("xarm7"),
            ready_streams=("color_image", "overview_image", "coordinator_joint_state"),
            scene=LfsPath("xarm7/scene.xml"),
            tracked_bodies=("cup",),
        ),
        grade=lifted("cup", by_m=0.05),
        timeout_s=600.0,
        tags=frozenset({"mujoco", "manipulation", "raw", "pick"}),
    ),
]
