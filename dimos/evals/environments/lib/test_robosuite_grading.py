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

from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dimos.evals.environments.lib.robosuite_grading import (
    door,
    lift,
    nut_assembly,
    pick_place,
    stack,
    sustained,
    tool_hang,
)
from dimos.evals.types import AgentInfo, FinalMetrics, Outcome, RunExtra, Trajectory
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.simulation.engines.mujoco_evaluation import MujocoEvaluationState as State


def state(
    positions: dict[str, list[float]],
    *,
    sites: dict[str, list[float]] | None = None,
    geoms: dict[str, list[float]] | None = None,
    contacts: tuple[tuple[str, str], ...] = (),
    held: tuple[str, ...] = (),
) -> State:
    return State(
        ts=10.0,
        positions={name: np.array(value, dtype=float) for name, value in positions.items()},
        rotations={name: np.eye(3) for name in positions},
        sites={name: np.array(value, dtype=float) for name, value in (sites or {}).items()},
        geoms={name: np.array(value, dtype=float) for name, value in (geoms or {}).items()},
        contacts=frozenset(contacts),
        robot_contacts=frozenset(held),
    )


def test_lift_requires_height_and_robot_support() -> None:
    s = state({"cube_main": [0, 0, 0.88]}, sites={"table_top": [0, 0, 0.8]}, held=("cube_main",))
    assert lift(s)
    assert not lift(replace(s, robot_contacts=frozenset()))
    s.positions["cube_main"][2] = 0.84
    assert not lift(s)


def test_door_measures_panel_relative_to_rotated_frame() -> None:
    s = state({"Door_door": [0, 0, 1], "Door_frame": [0, 0, 1]})
    frame = Rotation.from_euler("z", 1.8).as_matrix()
    s.rotations["Door_frame"] = frame
    for angle, expected in ((0.35, True), (0.1, False), (-0.35, False)):
        s.rotations["Door_door"] = frame @ Rotation.from_euler("z", angle).as_matrix()
        assert door(s) == expected
    s.rotations["Door_door"] = frame @ Rotation.from_euler("z", 0.35).as_matrix()
    assert not door(replace(s, robot_contacts=frozenset({"Door_door"})))


def test_can_rejects_wrong_compartment_hover_and_grasp() -> None:
    s = state({"Can_main": [0.70, 0.50, 0.86], "VisualCan_main": [0.6975, 0.5025, 0.86]})
    assert pick_place(s)
    assert not pick_place(replace(s, robot_contacts=frozenset({"Can_main"})))
    for p in ([0.50, 0.50, 0.86], [0.70, 0.50, 0.91]):
        s.positions["Can_main"] = np.array(p)
        assert not pick_place(s)


def test_stack_requires_support_alignment_release_and_table() -> None:
    s = state(
        {"cubeA_main": [0, 0, 0.87], "cubeB_main": [0, 0, 0.825]},
        sites={"table_top": [0, 0, 0.8]},
        contacts=(("cubeB_g0", "cubeA_g0"),),
    )
    assert stack(s)
    assert not stack(replace(s, contacts=frozenset()))
    assert not stack(replace(s, robot_contacts=frozenset({"cubeA_main"})))
    s.positions["cubeA_main"][0] = 0.04
    assert not stack(s)
    s.positions["cubeA_main"][0] = 0
    s.positions["cubeA_main"][2] += 0.1
    s.positions["cubeB_main"][2] += 0.1
    assert not stack(s)


def test_square_nut_rejects_wrong_peg_hover_and_offset_hole() -> None:
    s = state(
        {"SquareNut_main": [0.79, 0.1, 0.83], "peg1": [0.79, 0.1, 0.85]},
        sites={"table_top": [0, 0, 0.82]},
        contacts=(("SquareNut_g0", "table_collision"),),
    )
    assert nut_assembly(s)
    assert not nut_assembly(replace(s, contacts=frozenset()))
    assert not nut_assembly(replace(s, robot_contacts=frozenset({"SquareNut_main"})))
    for p in ([0.79, -0.1, 0.83], [0.79, 0.1, 0.97], [0.81, 0.1, 0.83]):
        s.positions["SquareNut_main"] = np.array(p)
        assert not nut_assembly(s)


def hanging() -> State:
    return state(
        {"stand_root": [0, 0, 0.08], "frame_root": [0, 0, 0.18], "tool_root": [0.05, 0, 0.2]},
        sites={
            "stand_mount_site": [0, 0, 0.15],
            "frame_tip_site": [0, 0, 0.01],
            "frame_mount_site": [0, 0, 0.05],
            "frame_intersection_site": [0, 0, 0.2],
            "frame_hang_site": [0.1, 0, 0.2],
            "tool_hole1_center": [0.05, 0, 0.2],
        },
        geoms={
            "stand_base": [0, 0, 0],
            "stand_wall0": [-0.006, 0, 0.1],
            "stand_wall1": [0, -0.006, 0.1],
            "stand_wall2": [0.006, 0, 0.1],
            "stand_wall3": [0, 0.006, 0.1],
            "tool_hole1_hc_0": [0.05, -0.013, 0.2],
            "tool_hole1_hc_4": [0.05, 0.013, 0.2],
        },
        contacts=(("tool_hole1_hc_0", "frame_horizontal_frame"),),
    )


def test_tool_hang_needs_assembly_threading_contact_and_release() -> None:
    s = hanging()
    assert tool_hang(s)
    assert not tool_hang(replace(s, contacts=frozenset()))
    for body in ("stand_root", "frame_root", "tool_root"):
        assert not tool_hang(replace(s, robot_contacts=frozenset({body})))
    for name, bad in (
        ("stand_mount_site", [0.15, 0, 0]),
        ("frame_tip_site", [0, 0, 0.1]),
        ("frame_mount_site", [0.1, 0.1, 0.05]),
        ("tool_hole1_center", [0.05, 0.02, 0.2]),
        ("tool_hole1_center", [0.11, 0, 0.2]),
    ):
        candidate = hanging()
        candidate.sites[name] = np.array(bad)
        assert not tool_hang(candidate), name
    s.geoms["tool_hole1_hc_4"] = s.geoms["tool_hole1_hc_0"].copy()
    assert not tool_hang(s)


def outcome(path: Path) -> Outcome:
    return Outcome(
        trajectory=Trajectory(
            agent=AgentInfo(name="test", version="1", model_name="none"),
            steps=(),
            final_metrics=FinalMetrics(
                total_prompt_tokens=0,
                total_completion_tokens=0,
                total_cached_tokens=0,
                total_cost_usd=0,
                total_steps=0,
            ),
            extra=RunExtra(ended_by="answer"),
        ),
        artifacts={"recording": path},
    )


@pytest.mark.parametrize(
    "failure", [None, "brief", "sparse", "missing", "nonfinite", "moving", "stale", "released"]
)
def test_terminal_window_grading(tmp_path: Path, failure: str | None) -> None:
    path = tmp_path / "recording.db"
    with SqliteStore(path=str(path)) as store:
        stream = store.stream("evaluation_state", State)
        for i in range(15):
            s = state(
                {"cube_main": [0, 0, 0.88]}, sites={"table_top": [0, 0, 0.8]}, held=("cube_main",)
            )
            s = replace(s, ts=10 + i * (0.4 if failure == "sparse" else 0.1))
            if failure == "brief" and i < 10:
                s.positions["cube_main"][2] = 0.82
            if failure == "missing":
                s.positions.clear()
            if failure == "nonfinite":
                s.positions["cube_main"][0] = np.nan
            if failure == "moving":
                s.positions["cube_main"][0] = i * 0.01
            if failure == "released":
                s = replace(s, robot_contacts=frozenset())
            stream.append(s)
        if failure == "stale":
            store.stream("coordinator_joint_state", JointState).append(JointState(ts=20))
    assert sustained(lift, ("cube_main",))(outcome(path)) == (1.0 if failure is None else 0.0)


def test_missing_recording_stream_fails(tmp_path: Path) -> None:
    path = tmp_path / "empty.db"
    with SqliteStore(path=str(path)):
        pass
    assert sustained(lift, ("cube_main",))(outcome(path)) == 0.0
