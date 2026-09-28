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

"""The xArm graders over a synthetic recording, and the suites' compositions."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from dimos.evals.agents.lib.trajectory_builder import TrajectoryBuilder
from dimos.evals.environments.mujoco_sim import MujocoEnvironmentConfig
from dimos.evals.suites.mujoco_xarm import (
    PERCEPTION_MODULES,
    arm_only_environment,
    ended_near,
    lifted,
    sensor_score,
)
from dimos.evals.suites.mujoco_xarm_pick_cylinder import SUITE as PICK_CYLINDER
from dimos.evals.types import Outcome
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.tf2_msgs.TFMessage import TFMessage

Point = tuple[float, float, float]
REST = {"apple": (0.40, 0.08, 0.17), "orange": (0.45, -0.08, 0.175), "cup": (0.50, 0.0, 0.19)}


def _outcome(
    tmp_path: Path, *, moves: dict[str, Point] | None = None, sensors: bool = True
) -> Outcome:
    """A recording of the tracked bodies at rest, then (optionally) moved."""
    db = tmp_path / "memory.db"
    store = SqliteStore(path=str(db))
    tf = store.stream("tf", TFMessage)
    for i, poses in enumerate((REST, {**REST, **(moves or {})})):
        tf.append(
            TFMessage(
                *(
                    Transform(
                        translation=Vector3(*xyz),
                        frame_id="world",
                        child_frame_id=body,
                        ts=1000.0 + i,
                    )
                    for body, xyz in poses.items()
                )
            ),
            ts=1000.0 + i,
        )
    if sensors:
        store.stream("color_image", Image).append(
            Image(data=np.zeros((1, 1, 3), np.uint8), format=ImageFormat.RGB, ts=1000.0), ts=1000.0
        )
        store.stream("coordinator_joint_state", JointState).append(
            JointState(ts=1000.0, name=["j1"], position=[0.0], velocity=[0.0]), ts=1000.0
        )
    store.stop()
    return Outcome(
        trajectory=TrajectoryBuilder("pick", name="test").build("answer"),
        artifacts={"recording": db},
    )


def test_sensor_score_needs_camera_joint_state_and_every_tracked_body(tmp_path: Path) -> None:
    assert sensor_score(_outcome(tmp_path / "ok")) == 1.0
    assert sensor_score(_outcome(tmp_path / "blind", sensors=False)) == 0.0


def test_lifted_scores_height_gained_up_to_the_target(tmp_path: Path) -> None:
    grade = lifted("cup", by_m=0.05)
    assert grade(_outcome(tmp_path / "still")) == 0.0
    assert grade(_outcome(tmp_path / "half", moves={"cup": (0.50, 0.0, 0.215)})) == pytest.approx(
        0.5
    )
    assert grade(_outcome(tmp_path / "high", moves={"cup": (0.50, 0.0, 0.40)})) == 1.0
    assert grade(_outcome(tmp_path / "dropped", moves={"cup": (0.50, 0.0, 0.05)})) == 0.0


def test_ended_near_scores_planar_distance_to_the_target(tmp_path: Path) -> None:
    grade = ended_near("orange", 0.45, 0.12, band_m=0.10)
    assert grade(_outcome(tmp_path / "there", moves={"orange": (0.45, 0.12, 0.175)})) == 1.0
    assert grade(
        _outcome(tmp_path / "close", moves={"orange": (0.45, 0.07, 0.175)})
    ) == pytest.approx(0.5)
    assert grade(_outcome(tmp_path / "untouched")) == 0.0  # 20 cm away, outside the band


def test_graders_score_zero_without_the_body(tmp_path: Path) -> None:
    grade = lifted("ghost", by_m=0.05)
    assert grade(_outcome(tmp_path)) == 0.0
    assert ended_near("ghost", 0.0, 0.0, band_m=1.0)(_outcome(tmp_path)) == 0.0


def test_rerun_flag_keeps_the_bridge() -> None:
    assert "rerun-bridge-module" in arm_only_environment().config.disable
    assert "rerun-bridge-module" not in arm_only_environment(rerun=True).config.disable


def test_pick_cylinder_is_one_arm_only_case_with_the_viewer() -> None:
    (case,) = PICK_CYLINDER
    assert case.id == "xarm_pick_cylinder"
    config = case.environment.config
    assert isinstance(config, MujocoEnvironmentConfig)
    assert set(PERCEPTION_MODULES) <= set(config.disable)
    assert "rerun-bridge-module" not in config.disable
    assert config.headless is False
