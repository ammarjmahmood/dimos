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

"""robots.py's checks catch each way robots.json can drift from the code (the real file is checked by
test_all_blueprints_generation.py), and `resolved` applies the defaults."""

from __future__ import annotations

import copy
from pathlib import Path
from typing import Any

import pytest

from dimos.robot import robots

IP = {"global": "robot_ip", "label": "Robot IP", "required": True}


def sample() -> dict[str, Any]:
    return {
        "tags": {
            "drive": {"label": "Drive", "description": "drives"},
            "replay": {"label": "Replay", "description": "from modes"},
            "sim": {"label": "Sim", "description": "from modes"},
        },
        "modes": {"robot": {"label": "Robot", "description": "hardware"}},
        "groups": {"arms": {"name": "Arms"}},
        "args": {"robot_ip": IP},
        "robots": {
            "dog": {
                "name": "Vendor Dog",
                "description": "a dog",
                "type": "dog",
                "manufacturer": "Vendor",
                "dirs": ["dimos/robot/vendor/dog"],
                "recommended_app": {
                    "id": "dog-app",
                    "title": "Dog Ctrl",
                    "url": "https://github.com/x/dog-app",
                },
                "defaults": {
                    "modes": {
                        "robot": {"args": ["robot_ip"]},
                        "sim": {"set": {"simulation": "mujoco"}},
                    }
                },
                "blueprints": {
                    "dog-basic": {
                        "title": "Dog",
                        "description": "walks",
                        "tags": ["drive"],
                        "starter": 1,
                    },
                    "dog-test": {
                        "title": "Dog test",
                        "description": "tests",
                        "tags": [],
                        "hidden": True,
                        "modes": {"robot": {}},
                        "recommended_app": None,
                    },
                },
            }
        },
        "excluded": {"dimos/robot/assets": "shared assets"},
    }


REGISTRY = {
    "dog-basic": "dimos.robot.vendor.dog.blueprints:dog_basic",
    "dog-test": "dimos.robot.vendor.dog.blueprints:dog_test",
}


@pytest.fixture
def root(tmp_path: Path) -> Path:
    for directory in ["dimos/robot/vendor/dog/blueprints", "dimos/robot/assets"]:
        (tmp_path / directory).mkdir(parents=True)
    return tmp_path


def test_a_consistent_file_has_no_problems(root: Path) -> None:
    assert robots.problems(sample(), REGISTRY, root) == []


def test_a_robot_dir_not_in_robots_json(root: Path) -> None:
    (root / "dimos/robot/vendor/cat").mkdir()
    (root / "dimos/robot/vendor/__pycache__").mkdir()
    found = robots.problems(sample(), REGISTRY, root)
    assert len(found) == 1
    assert found[0].startswith("dimos/robot/vendor/cat is not in robots.json")
    assert "add it to `excluded`" in found[0]


def test_a_dir_robots_json_names_that_is_gone(root: Path) -> None:
    doc = sample()
    doc["excluded"]["dimos/robot/gone"] = "was here"
    assert robots.problems(doc, REGISTRY, root) == [
        "robots.json excluded names dimos/robot/gone, which is not a directory"
    ]


def test_a_blueprint_in_a_robot_dir_that_isnt_listed(root: Path) -> None:
    registry = {**REGISTRY, "dog-run": "dimos.robot.vendor.dog.blueprints.run:dog_run"}
    found = robots.problems(sample(), registry, root)
    assert len(found) == 1
    assert "blueprint dog-run" in found[0]
    assert (
        '"dog-run": {"title": ..., "description": ..., "tags": [...]} to robots.dog.blueprints'
        in found[0]
    )


def test_a_blueprint_under_dimos_robot_that_no_robot_holds(root: Path) -> None:
    registry = {**REGISTRY, "loose": "dimos.robot.vendor.loose:loose"}
    found = robots.problems(sample(), registry, root)
    assert len(found) == 1 and "no robot's dirs hold it" in found[0]


def test_a_blueprint_outside_dimos_robot_may_go_unlisted(root: Path) -> None:
    registry = {**REGISTRY, "demo-x": "dimos.agents.demo:demo_x"}
    assert robots.problems(sample(), registry, root) == []
    assert robots.resolved(sample(), registry)["unlisted"] == ["demo-x"]


def test_a_listed_blueprint_that_doesnt_exist(root: Path) -> None:
    registry = {"dog-basic": REGISTRY["dog-basic"], "dog-tests": REGISTRY["dog-test"]}
    found = robots.problems(sample(), registry, root)
    assert any(
        "lists dog-test under robots.dog, but no such blueprint is registered" in p
        and "did you mean dog-tests?" in p
        for p in found
    )


def test_a_blueprint_listed_twice(root: Path) -> None:
    doc = sample()
    doc["robots"]["cat"] = {
        "name": "Cat",
        "description": "a cat",
        "type": None,
        "manufacturer": None,
        "dirs": [],
        "blueprints": {"dog-basic": {"title": "x", "description": "y", "tags": []}},
    }
    found = robots.problems(doc, REGISTRY, root)
    assert "dog-basic is listed under several robots (dog, cat): keep one" in found


def test_an_arg_that_isnt_a_global_config_field(root: Path) -> None:
    doc = sample()
    doc["args"]["robot_ip"] = {"global": "robot_ipp", "label": "IP"}
    doc["args"]["unused"] = {"global": "robot_ips", "label": "IPs"}
    doc["robots"]["dog"]["defaults"]["modes"]["sim"]["set"] = {"simulator": "mujoco"}
    doc["robots"]["dog"]["blueprints"]["dog-test"]["modes"]["robot"]["args"] = ["robot_id"]
    found = robots.problems(doc, REGISTRY, root)
    assert (
        "args.robot_ip: 'robot_ipp' is not a GlobalConfig field (did you mean robot_ip?)" in found
    )
    assert any("sets 'simulator', which is not a GlobalConfig field" in p for p in found)
    assert (
        "robots.dog.blueprints.dog-test robot mode takes arg 'robot_id', which isn't defined in the top-level "
        "`args` (did you mean robot_ip?)" in found
    )
    assert "args.unused is defined but no mode takes it: remove it" in found


def test_a_module_arg_that_isnt_a_config_field() -> None:
    doc = sample()
    doc["args"] = {
        "a": {"module": "droneconnectionmodule", "field": "connection_strin", "label": "x"},
        "b": {"module": "nosuchmodule", "field": "ip", "label": "y"},
        "c": {"module": "droneconnectionmodule", "field": "connection_string", "label": "z"},
    }
    doc["robots"]["dog"]["blueprints"] = {
        "drone-basic": {
            "title": "Drone",
            "description": "flies",
            "tags": [],
            "modes": {"robot": {"args": ["a", "b", "c"]}},
        }
    }
    found = robots.module_arg_problems(doc)
    assert len(found) == 2
    assert "droneconnectionmodule has no config field 'connection_strin'" in found[0]
    assert "did you mean connection_string?" in found[0]
    assert "the blueprint has no module 'nosuchmodule'" in found[1]


def test_schema_violations(root: Path) -> None:
    doc = sample()
    doc["robots"]["dog"]["blueprints"]["dog-basic"]["colour"] = "brown"
    del doc["robots"]["dog"]["blueprints"]["dog-test"]["description"]
    doc["args"]["both"] = {"global": "x", "module": "m", "field": "f", "label": "both"}
    found = robots.problems(doc, REGISTRY, root)
    assert any("'colour' was unexpected" in p for p in found)
    assert any("'description' is a required property" in p for p in found)
    assert any("['args']['both']" in p for p in found)


def test_tags_groups_and_starter_ranks(root: Path) -> None:
    doc = sample()
    blueprints = doc["robots"]["dog"]["blueprints"]
    blueprints["dog-basic"]["tags"] = ["drive", "sim", "fly"]
    blueprints["dog-test"]["starter"] = 1
    doc["robots"]["dog"]["group"] = "legs"
    found = robots.problems(doc, REGISTRY, root)
    assert any("drop the 'sim' tag, it comes from having a 'sim' mode" in p for p in found)
    assert any("unknown tag 'fly'" in p for p in found)
    assert "dog-test and dog-basic both have starter rank 1: give each its own" in found
    assert any("robots.dog.group is 'legs'" in p for p in found)


def test_resolved_applies_defaults() -> None:
    doc = sample()
    before = copy.deepcopy(doc)
    out = robots.resolved(doc, REGISTRY)
    assert doc == before
    basic = out["robots"]["dog"]["blueprints"]["dog-basic"]
    assert list(basic["modes"]) == ["robot", "sim"]
    assert basic["modes"]["robot"]["args"][0] == {
        **IP,
        "id": "robot_ip",
        "key": "robot_ip",
        "scope": "global",
        "kind": "text",
    }
    assert basic["modes"]["sim"] == {"set": {"simulation": "mujoco"}, "args": []}
    assert basic["tags"] == ["drive", "sim"]
    assert basic["robot"] == "dog" and basic["registered"] and basic["hidden"] is False
    assert basic["recommended_app"]["id"] == "dog-app"
    test = out["robots"]["dog"]["blueprints"]["dog-test"]
    assert list(test["modes"]) == ["robot"] and test["recommended_app"] is None
    assert test["starter"] is None and test["hidden"] is True
    assert "defaults" not in out["robots"]["dog"] and "args" not in out and out["unlisted"] == []


def test_module_args_resolve_to_their_option() -> None:
    arg = {"module": "spothighlevel", "field": "ip", "label": "Spot IP"}
    assert robots._resolved_arg("spot_ip", arg)["key"] == "spothighlevel.ip"
    assert robots._resolved_arg("spot_ip", arg)["scope"] == "module"


def test_the_real_file_lists_every_robot_dir_blueprint() -> None:
    """Spot checks on the real file; test_all_blueprints_generation.py runs every rule on it."""
    doc = robots.load()
    assert "go2" in doc["robots"] and "unitree-go2-basic" in doc["robots"]["go2"]["blueprints"]
    out = robots.resolved(doc)
    basic = out["robots"]["go2"]["blueprints"]["unitree-go2-basic"]
    assert list(basic["modes"]) == ["robot", "replay", "sim"]
    assert basic["recommended_app"]["id"] == "dim-go2-dash"


def test_dimos_yaml_points_at_it() -> None:
    import yaml

    from dimos.constants import DIMOS_PROJECT_ROOT

    pointer = yaml.safe_load((DIMOS_PROJECT_ROOT / "dimos.yaml").read_text())["robots"]
    assert (DIMOS_PROJECT_ROOT / pointer).resolve() == robots.ROBOTS_FILE.resolve()


def test_the_catalog_names_each_blueprints_robot_from_robots_json() -> None:
    """The dimos server's catalog takes a blueprint's robot from robots.json (no folder list of its own), so every
    blueprint a robot lists, or one in a robot's dirs, has that robot."""
    from dimos.robot.all_blueprints import all_blueprints
    from dimos.server.introspect import robot_of

    doc = robots.load()
    for robot_id, robot in doc["robots"].items():
        for name in robot["blueprints"]:
            assert robot_of(all_blueprints[name], name) == robot_id, name
    assert robot_of(all_blueprints["drone-basic"]) == "drone"
    assert robot_of(all_blueprints["spot-replay"]) == "spot"
    assert robot_of(all_blueprints["mid360-realsense-record"]) == "sensors"
    assert robot_of(all_blueprints["unitree-go2-basic"]) == "go2"
    assert robot_of("dimos.agents.demo_agent:demo_agent") is None


def robot(**fields: Any) -> dict[str, Any]:
    return {
        "name": "X",
        "description": "x",
        "type": None,
        "manufacturer": None,
        "dirs": [],
        "blueprints": {},
        **fields,
    }


def test_type_and_manufacturer_follow_the_code() -> None:
    doc = sample()
    doc["robots"]["cat"] = robot(
        name="Other Cat", type="dog", manufacturer="Other", dirs=["dimos/robot/vendor/cat"]
    )
    doc["robots"]["kit"] = robot(
        name="Kit", type="wheeled", manufacturer="Kitco", dirs=["dimos/robot/diy/kit"]
    )
    doc["robots"]["claw"] = robot(name="Claw", type="arm", dirs=["dimos/robot/manipulators/claw"])
    doc["robots"]["cam"] = robot(name="Cam", type="drone", dirs=["dimos/hardware/cam"])
    found = robots.kind_problems(doc)
    assert any("Kit" in p and "doesn't start with it" in p for p in found)
    assert any("robots.kit" in p and "(DIY)" in p for p in found)
    assert any("dimos/robot/vendor (dog: 'Vendor', cat: 'Other')" in p for p in found)
    assert any("robots.claw" in p and "arms group" in p for p in found)
    assert any("robots.cam.type" in p and "under dimos/robot" in p for p in found)
    assert robots.kind_problems(sample()) == []


def test_type_and_manufacturer_are_required(root: Path) -> None:
    doc = sample()
    del doc["robots"]["dog"]["type"]
    doc["robots"]["dog"]["manufacturer"] = ""
    found = robots.problems(doc, REGISTRY, root)
    assert any("'type' is a required property" in p for p in found)
    assert any("manufacturer" in p for p in found)
    doc = sample()
    doc["robots"]["dog"]["type"] = "boat"
    assert any("['type']" in p for p in robots.problems(doc, REGISTRY, root))


def test_vendor_dir() -> None:
    assert robots.vendor_dir("dimos/robot/unitree/go2") == "dimos/robot/unitree"
    assert (
        robots.vendor_dir("dimos/experimental/robot/bosdyn/spot")
        == "dimos/experimental/robot/bosdyn"
    )
    assert robots.vendor_dir("dimos/robot/manipulators/xarm") is None
    assert robots.vendor_dir("dimos/robot/drone") is None
    assert robots.vendor_dir("dimos/hardware/sensors") is None
