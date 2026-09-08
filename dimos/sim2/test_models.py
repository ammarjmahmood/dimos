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

from pathlib import Path

import mujoco
import numpy as np
import pytest

from dimos.robot.deeprobotics.m20.sim2 import M20Model
from dimos.robot.manipulators.xarm.sim2 import XArm7Model
from dimos.robot.unitree.g1.sim2 import G1Model
from dimos.sim2.models import SceneModel
from dimos.utils.data import LfsPath

pytestmark = pytest.mark.mujoco


@pytest.mark.parametrize(
    ("factory", "path", "meshdir"),
    [
        (
            G1Model,
            Path(__file__).parents[1] / "robot/unitree/g1/assets/g1_29dof.xml",
            LfsPath("g1_urdf/meshes"),
        ),
        (
            M20Model,
            Path(__file__).parents[1] / "robot/deeprobotics/m20/assets/m20.xml",
            LfsPath("m20_sdk/meshes"),
        ),
        (XArm7Model, LfsPath("xarm7/xarm7.xml"), None),
    ],
)
def test_robot_physics_and_appearance_match_native_mjcf(factory, path, meshdir):
    spec = mujoco.MjSpec.from_file(str(path))
    if meshdir is not None:
        spec.meshdir = str(meshdir)
    for key in list(spec.keys):
        spec.delete(key)
    native = spec.compile()
    imported = factory("comparison").get_model()
    for field in (
        "body_mass",
        "body_inertia",
        "body_ipos",
        "body_iquat",
        "body_pos",
        "body_quat",
        "dof_damping",
        "dof_armature",
        "dof_frictionloss",
        "jnt_stiffness",
        "jnt_range",
        "jnt_limited",
        "qpos0",
        "geom_pos",
        "geom_quat",
        "geom_size",
        "geom_rgba",
        "geom_friction",
        "geom_contype",
        "geom_conaffinity",
        "geom_solref",
        "geom_solimp",
        "actuator_gainprm",
        "actuator_biasprm",
        "actuator_dynprm",
        "actuator_gear",
        "actuator_ctrlrange",
        "actuator_forcerange",
        "actuator_ctrllimited",
        "actuator_forcelimited",
    ):
        np.testing.assert_allclose(
            getattr(imported, field), getattr(native, field), atol=1e-6, rtol=1e-6, err_msg=field
        )


def test_native_angle_and_include_semantics_are_preserved(tmp_path):
    (tmp_path / "body.xml").write_text("""<mujocoinclude><body name="body" euler="0 0 90">
      <joint name="joint" range="-90 90"/><geom size=".1"/>
    </body></mujocoinclude>""")
    path = tmp_path / "scene.xml"
    path.write_text(
        '<mujoco><compiler angle="degree"/><worldbody><include file="body.xml"/></worldbody></mujoco>'
    )
    native = mujoco.MjModel.from_xml_path(str(path))
    imported = SceneModel(str(path)).get_model()
    # MuJoCo's XML writer rounds to six significant digits. This experiment
    # preserves units/structure, but does not claim bit-exact MJCF round trips.
    np.testing.assert_allclose(imported.jnt_range, native.jnt_range, atol=5e-6, rtol=0)
    np.testing.assert_allclose(imported.body_quat, native.body_quat, atol=5e-6, rtol=0)
