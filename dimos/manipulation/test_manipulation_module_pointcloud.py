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

"""CPU native filtering at the camera-to-mapper boundary."""

import asyncio
import importlib

import pytest

pytest.importorskip("roboplan.core")

from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.monitor.world_monitor import WorldMonitor
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.tf.tf import TF, MultiTBuffer
from dimos.robot.assets.model import PlanarBaseDefinition, RobotModel

_URDF = """<robot name="test" version="1.0">
<link name="base"/><link name="arm"><collision><geometry>
<box size="0.2 0.2 0.2"/></geometry></collision></link>
<joint name="slide" type="prismatic"><parent link="base"/><child link="arm"/>
<axis xyz="1 0 0"/><limit lower="-2" upper="2" effort="1" velocity="1" acceleration="2"/>
</joint></robot>"""


@pytest.fixture
def make_module(tmp_path, monkeypatch):
    modules = []

    def make(*, xml=_URDF, joint_names=("slide",), planar=False, tf_extra_links=(), **overrides):
        model_path = tmp_path / f"robot-{len(modules)}.urdf"
        model_path.write_text(xml)
        model = RobotModel.from_file(model_path)
        base_link = "base"
        if planar:
            definition = PlanarBaseDefinition(
                velocity_limits=(1, 1, 1), acceleration_limits=(2, 2, 2)
            )
            model = model.with_planar_base(definition)
            base_link = definition.root_link
            joint_names = (*definition.joint_names, *joint_names)
        config = RobotModelConfig(
            model=model,
            joint_names=list(joint_names),
            base_link=base_link,
            base_pose=PoseStamped(frame_id="world", position=[1, 0, 0]),
            tf_extra_links=list(tf_extra_links),
            planning_groups=[
                PlanningGroupDefinition(
                    name="arm", joint_names=tuple(joint_names), base_link=base_link, tip_link="arm"
                )
            ],
        )
        world_type = importlib.reload(
            importlib.import_module("dimos.manipulation.planning.world.roboplan_world")
        ).RoboPlanWorld
        world = world_type()
        monitor = WorldMonitor(world)
        monitor.load_model(config)
        world.finalize()
        module = ManipulationModule(
            model=config,
            **overrides,
        )
        monkeypatch.setattr(module, "_world_monitor", monitor)
        monkeypatch.setattr(module, "_tf", MultiTBuffer())
        modules.append(module)
        return module

    yield make
    for module in modules:
        if module._started and module._loop is not None:
            asyncio.run_coroutine_threadsafe(
                module._loop.shutdown_default_executor(), module._loop
            ).result(timeout=5.0)
        if isinstance(module._tf, TF):
            module._tf.dispose()
        monkeypatch.setattr(module, "_tf", None)
        if module._started:
            module.stop()
        module.dispose()


def _state(module, stamp=1.0, position=0.0):
    module._on_joint_state(JointState(ts=stamp, name=["slide"], position=[position]))


def test_tf_waits_for_joint_measurement_before_publishing_dynamic_frames(make_module, mocker):
    module = make_module()
    module._world_monitor.start_state_monitor()
    publish = mocker.patch.object(module.tf, "publish")
    mocker.patch.object(
        module._tf_stop_event, "wait", side_effect=lambda _: module._tf_stop_event.set()
    )

    module._tf_publish_loop()

    publish.assert_not_called()


def test_delayed_wrist_tf_preserves_capture_state_while_the_robot_moves(make_module, mocker):
    xml = _URDF.replace(
        "</robot>",
        '<link name="wrist"/><joint name="wrist_mount" type="fixed">'
        '<parent link="arm"/><child link="wrist"/></joint></robot>',
    )
    module = make_module(
        xml=xml,
        tf_extra_links=("wrist",),
        static_transforms=[
            Transform(frame_id="wrist", child_frame_id="camera", translation=Vector3(0, 0.2, 0))
        ],
    )
    monitor = module._world_monitor
    monitor.start_state_monitor()
    _state(module, stamp=100.0, position=0.0)
    published = []

    def receive(message):
        # Exercise the wire format and fixed mount as well as dynamic FK edges.
        received = TFMessage.lcm_decode(message.lcm_encode())
        published.append(received)
        for transform in received.transforms:
            module.tfbuffer.receive_transform(transform)
        module._tf_stop_event.set()

    mocker.patch.object(module.tf, "publish", side_effect=receive)
    mocker.patch.object(module._tf_stop_event, "wait")
    get_pose = monitor.get_group_ee_pose

    def move_after_tip_fk(group_id, state):
        pose = get_pose(group_id, state)
        # A new measurement arrives between tip FK and wrist FK in one tick.
        _state(module, stamp=100.25, position=0.5)
        return pose

    mocker.patch.object(monitor, "get_group_ee_pose", side_effect=move_after_tip_fk)
    module._tf_publish_loop()
    module._tf_stop_event.clear()
    module._tf_publish_loop()

    assert [[tf.ts for tf in msg.transforms] for msg in published] == [
        [100.0, 100.0, 100.0],
        [100.25, 100.25, 100.25],
    ]
    assert [tf.child_frame_id for tf in published[0].transforms] == ["arm", "wrist", "camera"]
    assert [msg.transforms[1].translation.x for msg in published] == [1.0, 1.5]
