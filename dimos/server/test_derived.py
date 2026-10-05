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

"""What the server keeps a copy of, checked against the dimos code it copies, so the copy can't drift."""

import dataclasses
from pathlib import Path
import typing

import yaml

from dimos.core.coordination.blueprints import StreamRef
from dimos.core.run_registry import RunEntry
from dimos.server import config, models

ROOT = Path(__file__).parents[2]


def test_a_registry_run_is_a_run_entry() -> None:
    entry_fields = {field.name for field in dataclasses.fields(RunEntry)}
    assert set(models.RegistryRun.model_fields) <= entry_fields


def test_stream_directions_are_dimos_own() -> None:
    def options(annotation: object) -> set[str]:
        return set(typing.get_args(annotation))

    dimos_own = typing.get_type_hints(StreamRef)["direction"]
    assert options(models.Stream.model_fields["direction"].annotation) == options(dimos_own)


def test_dimos_yaml_version_is_the_package_version() -> None:
    found, package = config.checkout_version(ROOT)
    assert found and yaml.safe_load((ROOT / "dimos.yaml").read_text())["version"] == package
