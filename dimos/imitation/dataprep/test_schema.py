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

"""Feature declarations reject invalid recording interpretations at the JSON boundary."""

import json

from pydantic import ValidationError
import pytest

from dimos.imitation.dataprep.schema import FeatureSpec


@pytest.mark.parametrize(
    ("changes", "location"),
    [
        ({"shape": []}, ("shape",)),
        ({"shape": [0]}, ("shape", 0)),
        ({"shape": [-1]}, ("shape", 0)),
        ({"names": [""]}, ("names", 0)),
        ({"names": [" \t\n"]}, ("names", 0)),
        ({"dtype": "invalid_dtype"}, ("dtype",)),
    ],
)
def test_invalid_feature_fields_report_their_json_location(changes, location):
    declaration = {
        "stream": "joint_state",
        "field": "position",
        "dtype": "float32",
        "shape": [1],
        "names": ["arm/joint1"],
    }

    with pytest.raises(ValidationError) as error:
        FeatureSpec.model_validate_json(json.dumps(declaration | changes))

    assert error.value.errors()[0]["loc"] == location


def test_feature_json_schema_exposes_shape_and_name_constraints():
    fields = FeatureSpec.model_json_schema()["properties"]

    assert fields["shape"]["minItems"] == 1
    assert fields["shape"]["items"]["exclusiveMinimum"] == 0
    assert fields["names"]["items"]["pattern"] == r"\S"


@pytest.mark.parametrize(
    ("changes", "reason"),
    [
        ({"names": ["joint1", "joint2"]}, "vector feature names"),
        (
            {"dtype": "video", "shape": [8, 8, 3], "names": ["height", "width"]},
            "name every axis",
        ),
        ({"source_kind": "joint_position_updates", "field": "velocity"}, "position vector"),
    ],
)
def test_cross_field_feature_constraints_remain_enforced(changes, reason):
    declaration = {
        "stream": "joint_state",
        "field": "position",
        "dtype": "float32",
        "shape": [1],
        "names": ["arm/joint1"],
    }

    with pytest.raises(ValidationError, match=reason):
        FeatureSpec.model_validate_json(json.dumps(declaration | changes))
