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

"""Static MJCF/goal checks: compile and forward kinematics only, never step or render."""

import json
import math
from pathlib import Path

import mujoco
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dimos.e2e_tests.dimos_cli_call import DimosCliCall
from dimos.evals.environments.lib import robosuite_grading as grading
from dimos.evals.suites.robosuite_xarm import SUITE
from dimos.simulation.engines.mujoco_evaluation import MujocoEvaluationRecorder


@pytest.mark.mujoco
@pytest.mark.parametrize("case", SUITE, ids=lambda case: case.id)
def test_exported_scene_names_initial_failure_and_goal_geometry(case) -> None:
    config = case.environment.config
    proc = DimosCliCall()
    case.environment.configure_launch(proc)
    assert proc.global_args[-2:] == ["--xarm7-sim-base-height", "0.912"]
    assert "evaluation_state" in config.ready_streams
    assert "evaluation_state" in config.recorded_topics
    assert not config.raw_bridge
    scene = Path(config.scene.resolve())
    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)
    recorder = MujocoEvaluationRecorder(
        model,
        bodies=list(config.tracked_bodies),
        sites=json.loads(config.module_env["MUJOCOSIMMODULE__EVALUATION_SITES"]),
        geoms=json.loads(config.module_env["MUJOCOSIMMODULE__EVALUATION_GEOMS"]),
        robot_body="link_base",
    )
    predicate = getattr(grading, scene.parent.name)
    mujoco.mj_forward(model, data)
    assert not predicate(recorder.capture(data, 0))

    def place(body: str, xyz, rotation=None) -> None:
        joint = model.body_jntadr[model.body(body).id]
        assert model.jnt_type[joint] == mujoco.mjtJoint.mjJNT_FREE
        address = model.jnt_qposadr[joint]
        data.qpos[address : address + 3] = xyz
        data.qpos[address + 3 : address + 7] = (
            [1, 0, 0, 0] if rotation is None else rotation.as_quat(scalar_first=True)
        )

    if scene.parent.name == "lift":
        return  # Robot support is checked separately; don't fabricate a grasp here.
    if scene.parent.name == "door":
        data.joint("Door_hinge").qpos[0] = 0.35
    elif scene.parent.name == "pick_place":
        place("Can_main", data.body("VisualCan_main").xpos)
    elif scene.parent.name == "stack":
        xy = data.body("cubeB_main").xpos[:2]
        place("cubeB_main", [*xy, 0.82499])
        place("cubeA_main", [*xy, 0.86998])
    elif scene.parent.name == "nut_assembly":
        place("SquareNut_main", [*data.body("peg1").xpos[:2], 0.82999])
    else:
        # Seat the upright stand on the table and align the frame post with its slot.
        place("stand_root", [0.48, 0, 0.88])
        frame = np.array([0.48 - 0.04375, 0.045, 0.80999 + 0.13445])
        place("frame_root", frame)
        hole = frame + np.array([-0.02, 0, 0.08625 - 0.00596])
        rotation = Rotation.from_euler("y", math.pi / 2)
        center = hole - rotation.apply([-0.093532843272420993, 0, 0])
        place("tool_root", center, rotation)
    mujoco.mj_forward(model, data)
    assert predicate(recorder.capture(data, 1))
