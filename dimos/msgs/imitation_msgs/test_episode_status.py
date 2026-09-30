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

import json

from dimos_lcm.std_msgs import String as LCMString
from pydantic import ValidationError
import pytest

from dimos.msgs.imitation_msgs.EpisodeStatus import EpisodeStatus


@pytest.mark.parametrize("task_label", ["pick", None, "", "拿起积木 🦾"])
def test_episode_status_lcm_roundtrip_preserves_status_update(
    task_label: str | None,
) -> None:
    expected = EpisodeStatus(
        ts=12.25,
        state="recording",
        episodes_saved=2,
        episodes_discarded=1,
        last_event="start",
        task_label=task_label,
    )

    actual = EpisodeStatus.lcm_decode(expected.lcm_encode())

    assert actual == expected


def test_episode_status_uses_existing_string_envelope() -> None:
    status = EpisodeStatus(
        ts=1790796295.1234567,
        state="idle",
        episodes_saved=2**31 - 1,
        episodes_discarded=0,
        task_label="拿起积木 🦾",
    )
    payload = json.loads(LCMString.lcm_decode(status.lcm_encode()).data)
    assert payload == status.model_dump()
    assert payload["schema_version"] == 1
    assert EpisodeStatus.lcm_decode(status.lcm_encode()) == status


@pytest.mark.parametrize(
    "updates", [{"schema_version": 2}, {"ts": float("nan")}, {"state": "unknown"}]
)
def test_episode_status_rejects_invalid_wire_payload(updates) -> None:
    payload = {
        "schema_version": 1,
        "ts": 1.0,
        "state": "idle",
        "episodes_saved": 0,
        "episodes_discarded": 0,
    }
    payload.update(updates)
    with pytest.raises(ValidationError):
        EpisodeStatus.lcm_decode(LCMString(data=json.dumps(payload)).lcm_encode())


@pytest.mark.parametrize("payload", ["not json", "null", "[]", '{"schema_version":2}'])
def test_episode_status_rejects_malformed_json_or_wrong_shape(payload) -> None:
    with pytest.raises(ValidationError):
        EpisodeStatus.lcm_decode(LCMString(data=payload).lcm_encode())


def test_episode_status_requires_explicit_wire_version() -> None:
    payload = '{"ts":1.0,"state":"idle","episodes_saved":0,"episodes_discarded":0}'
    with pytest.raises(ValueError, match="requires schema_version"):
        EpisodeStatus.lcm_decode(LCMString(data=payload).lcm_encode())
