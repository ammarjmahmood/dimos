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
"""Scene overlay blueprints: one camera plus the overlay module.

```bash
dimos run scene-overlay-realsense --realsensecamera.serial-number <SERIAL>
dimos run scene-overlay-webcam --cameramodule.hardware.camera-index 0
```
"""

from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.camera.module import CameraModule
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.perception.scene_overlay.module import SceneOverlayModule

scene_overlay_realsense = autoconnect(
    RealSenseCamera.blueprint(
        width=640,
        height=480,
        fps=30,
        enable_depth=False,
        align_depth_to_color=False,
        enable_pointcloud=False,
    ),
    SceneOverlayModule.blueprint(),
)

scene_overlay_webcam = autoconnect(
    CameraModule.blueprint(transform=None),
    SceneOverlayModule.blueprint(),
)
