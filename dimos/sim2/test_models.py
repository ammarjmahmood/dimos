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

from dimos.robot.deeprobotics.m20.sim2 import M20
from dimos.robot.manipulators.xarm.sim2 import XARM7
from dimos.robot.unitree.g1.sim2 import G1_GROOT
from dimos.sim2.models import SceneModel
from dimos.sim2.robot import MotorLegged, MotorManipulator, motor_controller_config
from dimos.utils.data import LfsPath

pytestmark = pytest.mark.mujoco


@pytest.mark.parametrize(
    ("definition", "runtime", "path", "meshdir"),
    [
        (
            G1_GROOT,
            MotorLegged,
            Path(__file__).parents[1] / "robot/unitree/g1/assets/g1_29dof.xml",
            LfsPath("g1_urdf/meshes"),
        ),
        (
            M20,
            MotorLegged,
            Path(__file__).parents[1] / "robot/deeprobotics/m20/assets/m20.xml",
            LfsPath("m20_sdk/meshes"),
        ),
        (XARM7, MotorManipulator, LfsPath("xarm7/xarm7.xml"), None),
    ],
)
def test_robot_physics_and_appearance_match_native_mjcf(definition, runtime, path, meshdir):
    spec = mujoco.MjSpec.from_file(str(path))
    if meshdir is not None:
        spec.meshdir = str(meshdir)
    for key in list(spec.keys):
        spec.delete(key)
    native = spec.compile()
    robot = runtime(
        robot_type=definition.model.__name__,
        idn="comparison",
        base_type="NullBase",
        composite_controller_config=motor_controller_config(definition),
    )
    robot.load_model()
    imported = robot.robot_model.mujoco_model
    owners = [robot.robot_model, *robot.gripper.values()]
    body_ids = [0]
    for index in range(1, native.nbody):
        name = native.body(index).name
        candidates = [
            mujoco.mj_name2id(imported, mujoco.mjtObj.mjOBJ_BODY, owner.correct_naming(name))
            for owner in owners
        ]
        matches = [value for value in candidates if value >= 0]
        assert len(matches) == 1, name
        body_ids.extend(matches)
    extra_bodies = sorted(set(range(imported.nbody)) - set(body_ids))
    np.testing.assert_array_equal(imported.body_mass[extra_bodies], 0)
    assert (imported.nq, imported.nv, imported.nu, imported.ngeom) == (
        native.nq,
        native.nv,
        native.nu,
        native.ngeom,
    )
    for field in (
        "body_mass",
        "body_inertia",
        "body_ipos",
        "body_iquat",
        "body_pos",
        "body_quat",
    ):
        np.testing.assert_allclose(
            getattr(imported, field)[body_ids],
            getattr(native, field),
            atol=1e-6,
            rtol=1e-6,
            err_msg=field,
        )
    for field in (
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
