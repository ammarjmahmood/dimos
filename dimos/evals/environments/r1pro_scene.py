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

"""The R1 Pro classical MuJoCo scenes, graded from the simulator's own object state."""

from __future__ import annotations

import json
from pathlib import Path
import threading
import time
from typing import TYPE_CHECKING, Any

from pydantic import Field

from dimos.agents.mcp.mcp_adapter import McpAdapter
from dimos.evals.environments.lib.launch import default_mcp_url
from dimos.evals.environments.lib.r1pro_actions import action_running, call_json
from dimos.evals.environments.mujoco_sim import MujocoEnvironment, MujocoEnvironmentConfig
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.e2e_tests.dimos_cli_call import DimosCliCall
    from dimos.memory.store.base import Store

logger = setup_logger()


class R1ProSceneConfig(MujocoEnvironmentConfig):
    blueprint: list[str] = Field(default_factory=lambda: ["r1pro-classical-open-space-sim"])
    sim_module: str = "R1ProOpenSpaceSim"
    # Picks the layout: which object, color and size starts on which platform.
    seed: int = 5000
    # Object ID -> {"kind", "color", "on"} the case was written for. A live scene
    # that differs fails the case before the agent starts.
    expected_objects: dict[str, dict[str, str]] = Field(default_factory=dict)
    # Skill calls that do the case, saved as reference_plan.json. Only the
    # scripted-plan agent reads it.
    reference_plan: list[dict[str, Any]] = Field(default_factory=list)
    # Seconds between ground-truth samples in scene_truth.jsonl.
    truth_period_s: float = 1.0
    # Readiness is get_scene answering; no camera stream is required.
    ready_streams: tuple[str, ...] = ()
    # Streams the run records (``--record-topics`` globs). Grading reads
    # scene_truth.jsonl, so the recording is only for replaying what happened.
    record_topics: str = "odom,color_image,camera_info"
    launch_timeout_s: float = 1800.0


class R1ProScene(MujocoEnvironment):
    """An R1 Pro classical scene launched at a fixed seed.

    While the agent works, the simulator's ``get_scene`` view (which hand holds
    what, what each object rests on, the tray's cargo) is sampled into
    ``scene_truth.jsonl``; graders read that artifact, never the agent's words.
    """

    config: R1ProSceneConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._mcp: McpAdapter | None = None
        self._truth: Path | None = None
        self._write_lock = threading.Lock()
        self._stop_sampling = threading.Event()
        self._sampler: threading.Thread | None = None

    def configure_launch(self, proc: DimosCliCall) -> None:
        url = default_mcp_url()
        if McpAdapter(url).wait_for_ready(timeout=0.5):
            # Every launch serves MCP on the same port: this case would drive that robot.
            raise RuntimeError(f"a dimos is already serving MCP at {url}; stop it first")
        super().configure_launch(proc)
        proc.extra_env[f"{self.config.sim_module.upper()}__SEED"] = str(self.config.seed)
        proc.global_args += ["--record-topics", self.config.record_topics]

    def prepare_recording(self, recording: Store, path: Path, deadline: float) -> dict[str, Path]:
        self._mcp = McpAdapter(default_mcp_url())
        scene = self._wait_scene(deadline)
        self._check_layout(scene)
        folder = path.parent
        plan = folder / "reference_plan.json"
        plan.write_text(json.dumps(self.config.reference_plan, indent=2))
        episode = folder / "r1pro_episode.json"
        episode.write_text(
            json.dumps(
                {
                    "blueprint": list(self.config.blueprint),
                    "seed": self.config.seed,
                    "headless": self.config.headless,
                    "expected_objects": self.config.expected_objects,
                    "initial_scene": scene,
                },
                indent=2,
            )
        )
        self._truth = folder / "scene_truth.jsonl"
        self._record(scene)
        self._stop_sampling.clear()
        self._sampler = threading.Thread(target=self._sample, name="r1pro-truth", daemon=True)
        self._sampler.start()
        return {"episode": episode, "scene_truth": self._truth, "reference_plan": plan}

    def settle(self, budget_s: float) -> None:
        """Let a skill the agent left running finish, then take a last sample."""
        if self._mcp is None:
            return
        deadline = time.monotonic() + budget_s
        scene: dict[str, Any] | None = None
        while True:
            try:
                scene = call_json(self._mcp, "get_scene")
            except Exception as e:
                logger.warning("get_scene failed while settling", error=str(e))
                return
            if not action_running(scene) or time.monotonic() >= deadline:
                break
            time.sleep(2.0)
        self._record(scene)

    def stop(self) -> None:
        self._stop_sampling.set()
        if self._sampler is not None:
            self._sampler.join(timeout=10.0)
            self._sampler = None
        self._mcp = None
        super().stop()

    def _wait_scene(self, deadline: float) -> dict[str, Any]:
        assert self._mcp is not None
        error = "no reply"
        while time.monotonic() < deadline:
            try:
                scene = call_json(self._mcp, "get_scene")
                if scene.get("objects") and not scene.get("error"):
                    return scene
                error = str(scene.get("error") or "no objects yet")
            except Exception as e:
                error = str(e)
            time.sleep(1.0)
        raise TimeoutError(f"R1 Pro scene not ready: {error}")

    def _check_layout(self, scene: dict[str, Any]) -> None:
        rows = {row["id"]: row for row in scene["objects"]}
        wrong = [
            f"{oid}: expected {want}, got "
            + str({k: rows[oid].get(k) for k in want} if oid in rows else "nothing")
            for oid, want in self.config.expected_objects.items()
            if oid not in rows or any(rows[oid].get(k) != v for k, v in want.items())
        ]
        if wrong:
            raise RuntimeError(
                f"seed {self.config.seed} no longer gives the layout this case was written for; "
                "regenerate the suite's layouts. " + "; ".join(wrong)
            )

    def _sample(self) -> None:
        assert self._mcp is not None
        mcp = self._mcp
        while not self._stop_sampling.wait(self.config.truth_period_s):
            try:
                self._record(call_json(mcp, "get_scene"))
            except Exception as e:
                # Samples are evidence, not control: a missed one only coarsens the timeline.
                logger.debug("ground-truth sample failed", error=str(e))

    def _record(self, scene: dict[str, Any] | None) -> None:
        if scene is None or self._truth is None or "objects" not in scene:
            return
        tray = scene.get("tray", {})
        action = scene.get("action", {})
        line = {
            "t": time.time(),
            "sim_time": scene.get("sim_time"),
            "objects": [
                {
                    k: row.get(k)
                    for k in ("id", "kind", "color", "position", "on", "upright", "inside")
                }
                for row in scene["objects"]
            ],
            "held_objects": scene.get("held_objects", {}),
            "tray": {
                k: tray.get(k) for k in ("station", "held", "cargo", "position", "tilt_radians")
            },
            "base_pose": scene.get("base_pose"),
            "action": {k: action.get(k) for k in ("id", "name", "state", "success", "error")},
            "error": scene.get("error"),
        }
        with self._write_lock, self._truth.open("a") as stream:
            stream.write(json.dumps(line) + "\n")
