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

"""MuJoCo launch, readiness and settling for live manipulation evals."""

from __future__ import annotations

import json
import os
import time
from typing import TYPE_CHECKING, Any, cast

from pydantic import Field

from dimos.evals.environments.lib.body_poses import last_body_transform
from dimos.evals.environments.sim import Sim, SimConfig

if TYPE_CHECKING:
    from pathlib import Path

    from dimos.e2e_tests.dimos_cli_call import DimosCliCall
    from dimos.memory.store.base import Store
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped


_READY_STREAMS = ("color_image", "coordinator_joint_state")
# What graders and readiness read; depth_image is float32, which the JPEG recorder rejects.
_RECORDED_TOPICS = ("color_image", "camera_info", "coordinator_joint_state", "tf", "odom")


class MujocoEnvironmentConfig(SimConfig):
    # Free bodies whose world pose the simulator publishes on tf for the graders.
    tracked_bodies: tuple[str, ...] = ()
    # Joint speed below which the robot counts as at rest while settling.
    at_rest_rad_s: float = 0.02
    # Extra MODULE__FIELD overrides for the launched dimos; they beat blueprint values.
    module_env: dict[str, str] = Field(default_factory=dict)


class MujocoEnvironment(Sim):
    """Run agent evaluations in a MuJoCo scene, with ground-truth object poses recorded on tf."""

    config: MujocoEnvironmentConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._initial_body_positions: dict[str, list[float]] = {}

    def configure_launch(self, proc: DimosCliCall) -> None:
        proc.simulator = "mujoco"
        proc.global_args = ["--record-topics", ",".join(_RECORDED_TOPICS)]
        proc.extra_env.update(self.config.module_env)
        # Headless unless the shell asks for the viewer (MUJOCOSIMMODULE__HEADLESS=false on Linux).
        proc.extra_env.setdefault(
            "MUJOCOSIMMODULE__HEADLESS", os.environ.get("MUJOCOSIMMODULE__HEADLESS", "true")
        )
        if self.config.tracked_bodies:
            proc.extra_env["MUJOCOSIMMODULE__TRACKED_BODIES"] = json.dumps(
                list(self.config.tracked_bodies)
            )

    def prepare_recording(self, recording: Store, path: Path, deadline: float) -> dict[str, Path]:
        self.wait_ready(recording, deadline=deadline)
        metadata = path.parent / "mujoco_episode.json"
        metadata.write_text(json.dumps(self.episode_metadata(), indent=2))
        return {"episode": metadata}

    def wait_ready(self, recording: Store, *, deadline: float) -> None:
        """Wait for fresh samples on every ready stream and a pose for every tracked body."""
        # Message timestamps are Unix wall-clock seconds; the deadline is monotonic.
        while time.monotonic() < deadline:
            try:
                ages = [
                    time.time() - getattr(recording.streams, name).last().data.ts
                    for name in _READY_STREAMS
                    if name in recording.streams
                ]
                poses = {
                    body: last_body_transform(recording, body).translation
                    for body in self.config.tracked_bodies
                }
            except (LookupError, AttributeError):
                ages, poses = [], {}
            if len(ages) == len(_READY_STREAMS) and all(age < 10.0 for age in ages):
                self._initial_body_positions = {body: [t.x, t.y, t.z] for body, t in poses.items()}
                return
            time.sleep(0.1)
        raise TimeoutError(
            f"MuJoCo did not publish fresh {', '.join(_READY_STREAMS)}"
            + (
                f" and poses for {', '.join(self.config.tracked_bodies)}"
                if self.config.tracked_bodies
                else ""
            )
        )

    def latest_pose(self, recording: Store) -> PoseStamped:
        """Base pose from ``odom`` for floating-base robots; a fixed-base arm has none."""
        if "odom" not in recording.streams:
            raise LookupError("No MuJoCo odometry recorded")
        return cast("PoseStamped", recording.streams.odom.last().data)

    def settle(self, budget_s: float) -> None:
        """At rest once every joint is slower than ``at_rest_rad_s`` for ``at_rest_s``."""
        recording = self._recording
        if recording is None or "coordinator_joint_state" not in recording.streams:
            super().settle(budget_s)
            return
        rest_since: float | None = None
        deadline = time.monotonic() + budget_s
        while time.monotonic() < deadline:
            try:
                state = recording.streams.coordinator_joint_state.last().data
            except LookupError:
                return
            now = time.monotonic()
            if any(abs(v) > self.config.at_rest_rad_s for v in state.velocity):
                rest_since = None
            elif rest_since is None:
                rest_since = now
            elif now - rest_since >= self.config.at_rest_s:
                return
            time.sleep(self.config.settle_poll_s)

    def episode_metadata(self) -> dict[str, object]:
        return {
            "backend": "mujoco",
            "blueprint": list(self.config.blueprint),
            "tracked_bodies": list(self.config.tracked_bodies),
            "module_env": dict(self.config.module_env),
            "initial_body_positions": self._initial_body_positions,
        }
