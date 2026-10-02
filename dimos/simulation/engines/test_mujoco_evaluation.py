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

from types import SimpleNamespace

import mujoco
import numpy as np
import pytest

from dimos.simulation.engines.mujoco_evaluation import MujocoEvaluationRecorder
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule

XML = """<mujoco><worldbody>
<body name="robot" pos="0.15 0 1"><geom size="0.05"/></body>
<body name="assembly" pos="0 0 1"><freejoint/>
  <body name="object"><body name="tip" pos="0.1 0 0">
    <geom name="tip_geom" size="0.05"/><site name="tip_site"/>
  </body></body>
</body>
</worldbody></mujoco>"""


def test_contacts_include_descendants_and_use_nearest_tracked_owner() -> None:
    model = mujoco.MjModel.from_xml_string(XML)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    recorder = MujocoEvaluationRecorder(
        model,
        bodies=["assembly", "object"],
        sites=["tip_site"],
        geoms=["tip_geom"],
        robot_body="robot",
    )
    state = recorder.capture(data, 42.0)
    assert state.ts == 42.0
    assert state.robot_contacts == {"object"}
    assert any("tip_geom" in pair for pair in state.contacts)
    np.testing.assert_allclose(state.sites["tip_site"], [0.1, 0, 1])
    np.testing.assert_allclose(state.positions["object"], [0, 0, 1])
    data.xpos[:] = 100
    data.site_xpos[:] = 100
    data.geom_xpos[:] = 100
    np.testing.assert_allclose(state.positions["object"], [0, 0, 1])
    np.testing.assert_allclose(state.geoms["tip_geom"], [0.1, 0, 1])
    np.testing.assert_allclose(state.sites["tip_site"], [0.1, 0, 1])


def test_missing_names_fail_before_recording() -> None:
    model = mujoco.MjModel.from_xml_string(XML)
    with pytest.raises(KeyError):
        MujocoEvaluationRecorder(model, bodies=["missing"], sites=[], geoms=[], robot_body="robot")


def test_module_snapshot_is_opt_in_and_throttled(monkeypatch) -> None:
    model = mujoco.MjModel.from_xml_string(XML)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    module = MujocoSimModule()
    samples = []
    module.evaluation_state.subscribe(samples.append)
    try:
        engine = SimpleNamespace(data=data)
        module._publish_shm_and_lcm(engine)
        assert not samples
        module._evaluation_recorder = MujocoEvaluationRecorder(
            model, bodies=["object"], sites=[], geoms=[], robot_body="robot"
        )
        clock = iter([10.0, 10.01, 10.2])
        monkeypatch.setattr(
            "dimos.simulation.engines.mujoco_sim_module.time.monotonic", lambda: next(clock)
        )
        for _ in range(3):
            module._publish_shm_and_lcm(engine)
        assert len(samples) == 2
        assert samples[0].robot_contacts == {"object"}
    finally:
        monkeypatch.undo()
        module.stop()
