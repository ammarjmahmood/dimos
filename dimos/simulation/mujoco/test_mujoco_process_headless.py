# Copyright 2025-2026 Dimensional Inc.
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

import os
import subprocess
import sys

import pytest

PROBE = (
    "import os; import dimos.simulation.mujoco.mujoco_process as p; "
    "print(p.HEADLESS, os.environ.get('MUJOCO_GL'))"
)


def probe(env_changes):
    env = {
        k: v for k, v in os.environ.items() if k not in ("DISPLAY", "WAYLAND_DISPLAY", "MUJOCO_GL")
    }
    env.update(env_changes)
    result = subprocess.run(
        [sys.executable, "-c", PROBE], env=env, capture_output=True, text=True, timeout=120
    )
    assert result.returncode == 0, result.stderr
    return result.stdout.strip().splitlines()[-1]


@pytest.mark.skipif(not sys.platform.startswith("linux"), reason="headless mode is Linux only")
def test_no_display_renders_offscreen_with_egl():
    assert probe({}) == "True egl"


@pytest.mark.skipif(not sys.platform.startswith("linux"), reason="headless mode is Linux only")
def test_a_display_keeps_the_viewer_and_a_chosen_backend_wins():
    assert probe({"DISPLAY": ":0"}) == "False None"
    assert probe({"MUJOCO_GL": "glfw"}) == "True glfw"
