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

"""Exercise upstream assets and observables through the actual sim2 runtime."""

from uuid import uuid4

import numpy as np
import pytest
from robosuite.models.objects import BoxObject, CylinderObject
from robosuite.models.robots.manipulators.panda_robot import Panda
from robosuite.utils.observables import Observable, sensor

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.python_worker import PythonWorker
from dimos.core.global_config import GlobalConfig
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.sim2.blueprint import simulation
from dimos.sim2.control.adapters import ManipulatorAdapter
from dimos.sim2.module import SimulationModule
from dimos.sim2.robot import MotorManipulator, register_robot
from dimos.sim2.runtime import SimulationRuntime
from dimos.sim2.scene import scene_path
from dimos.sim2.scene_types import SceneUpdate
from dimos.sim2.sensors.spec import Camera, Mount
from dimos.sim2.spec import (
    ControlInterface,
    Joint,
    ObjectInstance,
    RobotConfig,
    RobotInstance,
    WorldConfig,
)


@register_robot(MotorManipulator)
class PandaDeviceModel(Panda):
    default_gripper = {"right": "EndEffectorFrame"}

    @property
    def naming_prefix(self):
        return f"{self.idn}/"


PANDA = RobotConfig(
    model=PandaDeviceModel,
    root_body="base",
    control=ControlInterface.MANIPULATOR,
    joints=tuple(
        Joint(f"joint{i}", f"joint{i}", f"torq_j{i}", home=home, kp=80, kd=8)
        for i, home in enumerate([0, 0.2, 0, -2.6, 0, 2.94, 0.785], 1)
    ),
    sensors=(Camera("wrist", Mount("right_hand", xyz=(0, 0, 0.05))),),
)


def test_upstream_robot_objects_observable_and_dimos_control():
    objects = (
        ObjectInstance(
            BoxObject, "box", (0.4, 0, 0.8), {"size": [0.03, 0.03, 0.03], "rgba": [1, 0.1, 0.1, 1]}
        ),
        ObjectInstance(
            CylinderObject,
            "can",
            (0.4, 0.2, 0.8),
            {"size": [0.025, 0.05], "rgba": [0.1, 0.2, 1, 1]},
        ),
    )
    config = WorldConfig(
        scene_path(None, "workbench.xml"),
        {"panda": RobotInstance(PANDA, xyz=(0, 0, 0.12))},
        objects=objects,
    )
    world = SimulationRuntime(config, "extensions-" + uuid4().hex)
    device = ManipulatorAdapter(
        address=world.snapshot_descriptor.sim_id + "/panda", dof=7, definition=PANDA
    )
    try:
        device.connect()
        target = device.read_joint_positions()
        target[0] = 0.2
        assert device.write_joint_positions(target)

        @sensor(modality="objects")
        def box_height(_cache):
            return np.array([world.data.body("box_main").xpos[2]])

        world.environment.add_observable(Observable("box_height", box_height, sampling_rate=200))
        for _ in range(100):
            world.step()
        assert device.read_joint_positions()[0] > 0.1
        assert "box_height" in world.environment.observation_names
        assert float(world.environment._get_observations()["box_height"]) > 0.1
        assert set(world.scene_state().entities) >= {"box", "can"}
        assert (
            world.model.camera("panda/sensor/wrist").bodyid
            == world.model.body("panda/right_hand").id
        )
        model = world.model
        world.set_scene_state(SceneUpdate(poses={"box": Pose(0.5, 0.1, 0.9)}))
        assert world.data.body("box_main").xpos == pytest.approx((0.5, 0.1, 0.9))
        world.reset()
        assert world.model is model
        assert world.data.body("box_main").xpos == pytest.approx((0.4, 0, 0.8))
    finally:
        device.disconnect()
        world.close()


def test_object_and_robot_factories_survive_blueprint_configuration():
    blueprint = simulation(
        scene=scene_path(None, "workbench.xml"),
        robots={"panda": RobotInstance(PANDA)},
        objects=(ObjectInstance(BoxObject, "box", (0.3, 0, 0.8), {"size": [0.03, 0.03, 0.03]}),),
        viewer=False,
    ).blueprint
    parsed = BlueprintConfigParser(blueprint).parse(environ={})
    assert parsed is not None


class BuildOnDeploy(SimulationModule):
    """Build inside the forkserver without relying on network discovery."""

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        try:
            self.build()
            self.evidence = {
                "entities": tuple(self.describe_scene().entities),
                "robots": tuple(self.status()["robots"]),
                "box_height": self.reset().entities["box"].pose.position.z,
            }
        finally:
            self.stop()


def test_factories_deploy_in_a_real_dimos_worker():
    config = WorldConfig(
        scene_path(None, "workbench.xml"),
        {"panda": RobotInstance(PANDA, xyz=(0, 0, 0.12))},
        objects=(ObjectInstance(BoxObject, "box", (0.3, 0, 0.8), {"size": [0.03, 0.03, 0.03]}),),
    )
    g = GlobalConfig(n_workers=1)
    worker = PythonWorker()
    worker.start_process()
    try:
        module = worker.deploy_module(
            BuildOnDeploy, g, {"world": config, "sim_id": "worker-proof-" + uuid4().hex}
        )
        # Exercise the real forkserver and Actor pipe, independently of Zenoh discovery.
        evidence = module.evidence
        assert "box" in evidence["entities"]
        assert evidence["robots"] == ("panda",)
        assert evidence["box_height"] == pytest.approx(0.8)
    finally:
        worker.shutdown()
