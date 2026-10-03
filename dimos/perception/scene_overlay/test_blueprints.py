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
from dimos.perception.scene_overlay.blueprints import (
    scene_overlay_realsense,
    scene_overlay_webcam,
)
from dimos.perception.scene_overlay.module import SceneOverlayModule


def _modules(blueprint):  # type: ignore[no-untyped-def]
    return {atom.module for atom in blueprint.active_blueprints}


def test_both_blueprints_compose_a_camera_with_the_overlay() -> None:
    assert SceneOverlayModule in _modules(scene_overlay_realsense)
    assert SceneOverlayModule in _modules(scene_overlay_webcam)


def test_the_camera_feeds_the_overlay_by_stream_name() -> None:
    for blueprint in (scene_overlay_realsense, scene_overlay_webcam):
        producers = {stream.name for atom in blueprint.active_blueprints for stream in atom.streams}
        assert "color_image" in producers
