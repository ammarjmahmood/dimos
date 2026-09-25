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

"""Write the layouts the R1 Pro suites' cases are phrased against.

Building a scene takes one (open space) to twelve (apartment) seconds per seed
and needs MuJoCo, so the suites read these files instead of building scenes at
import. Rerun after changing a scene generator; the environment refuses a case
whose seed drifted.

    python -m dimos.evals.suites.lib.r1pro_layouts 5000 5001 5002
    python -m dimos.evals.suites.lib.r1pro_layouts --scene apartment 5000 5001 5002
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import tempfile
from typing import Any

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.robot.galaxea.r1pro.apartment_scene import distribute_apartment_objects
from dimos.robot.galaxea.r1pro.classical_selection import color_name
from dimos.robot.galaxea.r1pro.everyday_objects import sample_everyday_layout
from dimos.robot.galaxea.r1pro.open_space_scene import OPEN_PLATFORMS, prepare_open_space_scene
from dimos.robot.galaxea.r1pro.primitive_scene import bilateral_layout, prepare_primitive_scene

OUTPUTS = {
    "open_space": Path(__file__).parents[1] / "r1pro_open_space.json",
    "apartment": Path(__file__).parents[1] / "r1pro_apartment.json",
}
# The house R1ProApartmentSim loads by default.
APARTMENT_PACKAGE = DIMOS_PROJECT_ROOT / "dimos/data/scene_packages/hssd_102344115"


def layout(seed: int, scene: str = "open_space") -> dict[str, Any]:
    """The objects of one seed as the sim builds it, with the default launch options."""
    sampled = bilateral_layout(sample_everyday_layout(seed, occupied=0))
    with tempfile.TemporaryDirectory() as folder:
        path = Path(folder) / "scene.xml"
        if scene == "apartment":
            built, placed = prepare_primitive_scene(
                path, sampled, "right", scene_package=APARTMENT_PACKAGE
            )
            placed, regions = distribute_apartment_objects(built, placed)
        else:
            _, placed, regions = prepare_open_space_scene(path, sampled)
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
    platforms = (
        [
            {
                "name": name,
                "xy": [float(r.center[0]), float(r.center[1])],
                "height_m": float(r.center[2]),
            }
            for name, r in regions.items()
        ]
        if scene == "apartment"
        else [{"name": p.name, "xy": list(p.xy), "height_m": p.height} for p in OPEN_PLATFORMS]
    )
    return {
        "seed": seed,
        "scene": scene,
        "tray_platform": "worktable",
        "objects": objects,
        "platforms": platforms,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("seeds", nargs="+", type=int)
    parser.add_argument("--scene", choices=sorted(OUTPUTS), default="open_space")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    output = args.output or OUTPUTS[args.scene]
    data = {"scene": args.scene, "layouts": [layout(seed, args.scene) for seed in args.seeds]}
    output.write_text(json.dumps(data, indent=2) + "\n")
    print(f"wrote {len(args.seeds)} layouts to {output}")


if __name__ == "__main__":
    main()
