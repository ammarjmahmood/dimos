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

from __future__ import annotations

from typing import ClassVar, Literal, TypeAlias, cast

from dimos_lcm.std_msgs import String as LCMString
from pydantic import BaseModel, FiniteFloat

EpisodeEvent: TypeAlias = Literal["start", "save", "discard", "init"]
RecordingState: TypeAlias = Literal["idle", "recording"]


class EpisodeStatus(BaseModel):
    """Source-timestamped status update for an imitation-learning episode.

    Versioned JSON uses the existing std_msgs.String LCM envelope. Native
    recording keeps those bytes and extracts ``ts`` for the observation time.
    Readers support schema version 1 only; legacy generated-message recordings
    are intentionally unsupported.
    """

    msg_name: ClassVar[str] = "imitation_msgs.EpisodeStatus"

    schema_version: Literal[1] = 1
    ts: FiniteFloat
    state: RecordingState
    episodes_saved: int
    episodes_discarded: int
    last_event: EpisodeEvent = "init"
    task_label: str | None = None

    def lcm_encode(self) -> bytes:
        """Carry validated episode JSON in the existing String wire envelope."""
        return cast("bytes", LCMString(data=self.model_dump_json()).lcm_encode())

    @classmethod
    def lcm_decode(cls, data: bytes) -> EpisodeStatus:
        status = cls.model_validate_json(LCMString.lcm_decode(data).data)
        if "schema_version" not in status.model_fields_set:
            raise ValueError("EpisodeStatus JSON requires schema_version")
        return status
