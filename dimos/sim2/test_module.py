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
import threading
import time
from uuid import uuid4

import mujoco
import numpy as np
import pytest

from dimos.robot.manipulators.xarm.sim2 import XARM7
from dimos.sim2.module import SimulationModule
from dimos.sim2.spec import RobotInstance, WorldConfig
from dimos.utils.data import LfsPath

pytestmark = pytest.mark.mujoco
TABLE = Path(__file__).parents[1] / "robot/manipulators/xarm/assets/table.xml"


@pytest.fixture
def sim():
    module = SimulationModule(
        world=WorldConfig(TABLE, {"arm": RobotInstance(XARM7, xyz=(0, 0, 0.12))}),
        sim_id=uuid4().hex,
        tracked_bodies=("apple", "cup"),
    )
    try:
        yield module
    finally:
        module.stop()


def test_eval_geometry_matches_original_scene(sim):
    sim.build()
    original = mujoco.MjModel.from_xml_path(str(LfsPath("xarm7/scene.xml")))
    runtime = sim._require_runtime()
    for name in ("apple", "orange", "cup"):
        before, after = original.body(name), runtime.model.body(name)
        for field in ("pos", "quat", "mass", "inertia"):
            np.testing.assert_allclose(getattr(after, field), getattr(before, field))
        old_geom = original.geom(before.geomadr[0])
        new_geom = runtime.model.geom(after.geomadr[0])
        for field in ("type", "size", "friction", "solref", "solimp", "contype", "conaffinity"):
            np.testing.assert_allclose(getattr(new_geom, field), getattr(old_geom, field))
    for name in ("floor", "table_top", "table_leg1", "table_leg2", "table_leg3", "table_leg4"):
        for field in ("pos", "size", "friction"):
            np.testing.assert_allclose(
                getattr(runtime.model.geom(name), field), getattr(original.geom(name), field)
            )


def test_selected_body_transforms_publish_without_full_scene_truth(sim, mocker):
    sim.build()
    published = threading.Event()
    sink = mocker.patch.object(sim.tf, "publish", side_effect=lambda _: published.set())
    truth = mocker.patch.object(sim.sim_truth, "publish")
    before = time.time()
    sim._start_truth_publisher()
    assert published.wait(2), "no TF published"
    message = sink.call_args.args[0]
    assert [t.child_frame_id for t in message.transforms] == ["apple", "cup"]
    assert [t.frame_id for t in message.transforms] == ["world", "world"]
    assert message.transforms[0].ts >= before
    assert message.transforms[0].ts == message.transforms[1].ts
    assert message.transforms[1].translation.to_tuple() == pytest.approx((0.5, 0, 0.19))
    assert message.transforms[1].rotation.to_tuple() == pytest.approx((0, 0, 0, 1))
    truth.assert_not_called()


def test_unknown_tracked_body_fails_before_runtime_is_exposed(sim):
    sim.config.tracked_bodies = ("not-in-scene",)
    with pytest.raises(ValueError, match="unknown tracked body 'not-in-scene'"):
        sim.build()
    with pytest.raises(RuntimeError, match="has not completed"):
        sim.describe()
