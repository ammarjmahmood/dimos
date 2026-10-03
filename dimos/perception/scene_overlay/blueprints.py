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
"""Scene overlay blueprints: one camera, the overlay module and a Rerun layout.

```bash
dimos run scene-overlay-realsense --realsensecamera.serial-number <SERIAL> --sceneoverlaymodule.scene scene-03
dimos run scene-overlay-webcam
```

Rerun opens with the camera's 3D view on the left and the overlay full size on
the right; either side hides with its eye icon.
"""

from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.camera.module import CameraModule
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.perception.scene_overlay.module import SceneOverlayModule
from dimos.visualization.rerun.bridge import RerunBridgeModule


def scene_overlay_view() -> Any:
    """3D world on the left, the overlay as a 2D view on the right."""
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial3DView(
                origin="world",
                background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
                line_grid=rrb.LineGrid3D(plane=rr.components.Plane3D.XY.with_distance(0.0)),
            ),
            rrb.Spatial2DView(origin="world/overlay_image", name="Scene overlay"),
            column_shares=[1, 2],
        ),
    )


def scene_overlay_rerun() -> Any:
    """The bridge with the overlay layout; a 2 GB cap keeps long sessions responsive."""
    return RerunBridgeModule.blueprint(blueprint=scene_overlay_view, memory_limit="2GB")


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
    scene_overlay_rerun(),
)

scene_overlay_webcam = autoconnect(
    CameraModule.blueprint(transform=None),
    SceneOverlayModule.blueprint(),
    scene_overlay_rerun(),
)
