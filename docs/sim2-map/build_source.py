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

"""Embed the walkthrough's source excerpts without importing or starting DimOS."""

import ast
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
SOURCES = {
    "tick": ("dimos/control/tick_loop.py", "TickLoop._tick"),
    "tick_timer": ("dimos/control/tick_loop.py", "TickLoop._loop"),
    "controller_write": (
        "dimos/control/hardware_interface.py",
        "ConnectedWholeBody.write_joint_commands",
    ),
    "adapter_write": ("dimos/sim2/control/adapters.py", "WholeBodyAdapter.write_motor_commands"),
    "adapter_imu": ("dimos/sim2/control/adapters.py", "WholeBodyAdapter.read_imu"),
    "physics_timer": ("dimos/sim2/module.py", "SimulationModule._run"),
    "physics": ("dimos/sim2/runtime.py", "SimulationRuntime.step"),
    "actuators": ("dimos/sim2/runtime.py", "SimulationRuntime._apply"),
    "publish": ("dimos/sim2/runtime.py", "SimulationRuntime._publish"),
    "feedback": ("dimos/sim2/connections/whole_body.py", "WholeBodyConnection._publish"),
    "sensor_loop": ("dimos/sim2/sensors/module.py", "SensorModule._run"),
    "reader": ("dimos/sim2/sensors/reader.py", "WorldReader"),
    "camera": ("dimos/sim2/sensors/camera/module.py", "SimCameraModule.capture"),
    "lidar": ("dimos/sim2/sensors/lidar/module.py", "LidarModule.capture"),
    "navigation": ("dimos/navigation/replanning_a_star/module.py", "ReplanningAStarPlanner.start"),
    "movement": ("dimos/navigation/movement_manager/movement_manager.py", "MovementManager.start"),
    "composition": ("dimos/sim2/blueprint.py", "simulation"),
    "build": ("dimos/sim2/module.py", "SimulationModule.build"),
    "viewer": ("dimos/sim2/viewer.py", "main"),
    "truth": ("dimos/sim2/module.py", "SimulationModule._publish_truth"),
    "g1_definition": ("dimos/robot/unitree/g1/sim2.py", "G1_GROOT"),
    "channel_publish": ("dimos/sim2/ipc/channel.py", "RobotChannel._publish"),
    "channel_read": ("dimos/sim2/ipc/channel.py", "RobotChannel._read"),
}


def excerpt(path: str, symbol: str) -> dict:
    text = (ROOT / path).read_text()
    node = ast.parse(text)
    for part in symbol.split("."):
        node = next(
            child
            for child in node.body
            if getattr(child, "name", None) == part
            or (
                isinstance(child, ast.Assign)
                and any(
                    isinstance(target, ast.Name) and target.id == part for target in child.targets
                )
            )
        )
    return {
        "path": path,
        "symbol": symbol,
        "line": node.lineno,
        "sha256": hashlib.sha256(text.encode()).hexdigest(),
        "code": "\n".join(text.splitlines()[node.lineno - 1 : node.end_lineno]),
    }


if __name__ == "__main__":
    data = {
        "generated": datetime.now(timezone.utc).isoformat(),
        "excerpts": {key: excerpt(*value) for key, value in SOURCES.items()},
    }
    (HERE / "source.js").write_text("window.SIM2_SOURCE = " + json.dumps(data) + ";\n")
    print(f"Embedded {len(SOURCES)} source excerpts; no DimOS imports or simulation execution.")
