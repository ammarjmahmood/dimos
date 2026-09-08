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

"""Counterexamples to lossless import through upstream's standard model loader.

These tests document the integration boundary, not an accepted sim2 behavior.
No patched loader or alternative conversion path is installed by this experiment.
"""

import mujoco
import numpy as np
import pytest
from robosuite.models.robots.robot_model import RobotModel


def test_top_level_joint_defaults_are_replaced(tmp_path):
    # These are the defaults in DimOS's g1_29dof.xml.
    path = tmp_path / "robot.xml"
    path.write_text("""<mujoco>
      <default><joint damping="0.001" armature="0.01" frictionloss="0.1"/></default>
      <worldbody><body name="base"><joint name="hinge"/>
        <geom type="box" size=".1 .1 .1" mass="1" group="1"/>
      </body></worldbody>
      <actuator><motor name="motor" joint="hinge"/></actuator>
    </mujoco>""")
    native = mujoco.MjModel.from_xml_path(str(path))
    imported = RobotModel(str(path)).get_model()

    assert native.dof_damping[0] == pytest.approx(0.001)
    assert native.dof_armature[0] == pytest.approx(0.01)
    assert imported.dof_damping[0] == pytest.approx(0.1)
    assert imported.dof_armature[0] == pytest.approx(5.0)


def test_nested_default_class_is_not_importable(tmp_path):
    path = tmp_path / "robot.xml"
    path.write_text("""<mujoco>
      <default><default class="robot"><joint damping=".2"/>
        <default class="finger"><joint damping=".3"/></default>
      </default></default>
      <worldbody><body name="base"><joint name="hinge" class="finger"/>
        <geom type="box" size=".1 .1 .1" mass="1" group="1"/>
      </body></worldbody>
      <actuator><motor name="motor" joint="hinge"/></actuator>
    </mujoco>""")
    native = mujoco.MjModel.from_xml_path(str(path))
    assert native.dof_damping[0] == pytest.approx(0.3)
    with pytest.raises(KeyError, match="finger"):
        RobotModel(str(path))


def test_collision_geom_appearance_is_rewritten(tmp_path):
    path = tmp_path / "robot.xml"
    path.write_text("""<mujoco>
      <worldbody><body name="base"><joint name="hinge"/>
        <geom name="shell" type="box" size=".1 .1 .1" mass="1" rgba=".2 .3 .4 1"/>
      </body></worldbody>
      <actuator><motor name="motor" joint="hinge"/></actuator>
    </mujoco>""")
    native = mujoco.MjModel.from_xml_path(str(path))
    imported = RobotModel(str(path)).get_model()
    np.testing.assert_allclose(native.geom_rgba[0], [0.2, 0.3, 0.4, 1])
    assert not np.allclose(imported.geom_rgba[0], native.geom_rgba[0])
