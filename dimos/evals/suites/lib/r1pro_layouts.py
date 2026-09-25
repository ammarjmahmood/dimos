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

"""Write the open-space layouts the R1 Pro suite's cases are phrased against.

Building a scene takes about a second per seed and needs MuJoCo, so the suite
reads this file instead of building scenes at import. Rerun it after changing
the scene generator; the environment refuses a case whose seed drifted.

    python -m dimos.evals.suites.lib.r1pro_layouts 5000 5001 5002
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import tempfile
from typing import Any

from dimos.robot.galaxea.r1pro.classical_selection import color_name
from dimos.robot.galaxea.r1pro.everyday_objects import sample_everyday_layout
from dimos.robot.galaxea.r1pro.open_space_scene import OPEN_PLATFORMS, prepare_open_space_scene
from dimos.robot.galaxea.r1pro.primitive_scene import bilateral_layout

OUTPUT = Path(__file__).parents[1] / "r1pro_open_space.json"


def layout(seed: int) -> dict[str, Any]:
    """The objects of one seed as the sim builds it, with the default launch options."""
    sampled = bilateral_layout(sample_everyday_layout(seed, occupied=0))
    with tempfile.TemporaryDirectory() as folder:
        _, placed, regions = prepare_open_space_scene(Path(folder) / "scene.xml", sampled)
    objects = []
    for i, obj in enumerate(placed.objects):
        # Platforms stand metres apart, so the nearest region centre is the support.
        platform = min(
            regions,
            key=lambda name: (regions[name].center[0] - obj.position[0]) ** 2
            + (regions[name].center[1] - obj.position[1]) ** 2,
        )
        objects.append(
            {
                "id": f"object_{i + 1}",
                "kind": obj.kind,
                "color": color_name(list(obj.rgba)),
                "platform": platform,
            }
        )
    return {
        "seed": seed,
        "tray_platform": "worktable",
        "objects": objects,
        "platforms": [
            {"name": p.name, "xy": list(p.xy), "height_m": p.height} for p in OPEN_PLATFORMS
        ],
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("seeds", nargs="+", type=int)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    args = parser.parse_args()
    data = {"scene": "open_space", "layouts": [layout(seed) for seed in args.seeds]}
    args.output.write_text(json.dumps(data, indent=2) + "\n")
    print(f"wrote {len(args.seeds)} layouts to {args.output}")


if __name__ == "__main__":
    main()
