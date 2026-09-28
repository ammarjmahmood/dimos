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

"""Where an ingest of a ROS 2 mcap writes: into the recording itself, appended in
place, as the plain ROS messages :mod:`dimos.mapping.hyperspace.mcap_format`
describes. Nothing already in the file moves, and a crash leaves the recording as it
was at the last flush."""

from __future__ import annotations

from pathlib import Path

import numpy as np
from numpy.typing import NDArray

from dimos.mapping.hyperspace import mcap_format as fmt
from dimos.memory.store.mcap_append import ChannelSpec, McapAppender


def stamp_ns(ts: float) -> int:
    return round(ts * 1e9)


def hyperspace_topics_in(path: str | Path) -> list[str]:
    """The topics an ingest would write that the recording already has."""
    from mcap.reader import make_reader

    from dimos.memory.store.mcap_append import load_summary

    with open(path, "rb") as handle:
        summary = load_summary(make_reader(handle), path)
    topics = {channel.topic for channel in summary.channels.values()} if summary else set()
    return sorted(
        topic
        for topic in topics
        if topic in (fmt.DEPTH2DEPTH_TOPIC, fmt.THUMBNAILS_TOPIC) or fmt.is_patch_topic(topic)
    )


class McapSink:
    def __init__(self, path: str | Path) -> None:
        self.path = str(path)
        self.appender = McapAppender(self.path)
        self._channels: dict[str, int] = {}
        self.written: dict[str, int] = {}

    def _channel(self, topic: str, type_name: str, schema: bytes) -> int:
        if topic not in self._channels:
            self._channels[topic] = self.appender.add_channel(
                ChannelSpec(
                    topic,
                    "cdr",
                    schema_name=type_name,
                    schema_encoding="ros2msg",
                    schema_data=schema,
                    metadata={"dimos.writer": "hyperspace"},
                )
            )
        return self._channels[topic]

    def _write(self, topic: str, channel: int, log_time: int, data: bytes) -> None:
        self.appender.add_message(channel, log_time=log_time, data=data)
        self.written[topic] = self.written.get(topic, 0) + 1

    def depth2depth(self, frame_id: str, ts: float, depth_mm: NDArray[np.uint16]) -> None:
        channel = self._channel(fmt.DEPTH2DEPTH_TOPIC, fmt.IMAGE_TYPE, fmt.IMAGE_SCHEMA)
        at = stamp_ns(ts)
        self._write(
            fmt.DEPTH2DEPTH_TOPIC, channel, at, fmt.encode_depth_image(at, frame_id, depth_mm)
        )

    def thumbnail(self, frame_id: str, ts: float, depth_mm: NDArray[np.uint16]) -> None:
        channel = self._channel(fmt.THUMBNAILS_TOPIC, fmt.IMAGE_TYPE, fmt.IMAGE_SCHEMA)
        at = stamp_ns(ts)
        self._write(
            fmt.THUMBNAILS_TOPIC, channel, at, fmt.encode_depth_image(at, frame_id, depth_mm)
        )

    def patches(
        self,
        member: str,
        spec: str,
        frame_id: str,
        ts: float,
        grid_shape: tuple[int, int],
        rays: NDArray[np.float32],
        depths: NDArray[np.float32],
        embeddings: NDArray[np.float32],
    ) -> None:
        topic = fmt.patch_topic(member)
        channel = self._channel(topic, fmt.PATCH_TYPE, fmt.PATCH_SCHEMA)
        at = stamp_ns(ts)
        rows, cols = grid_shape
        frame = fmt.PatchFrame(
            at,
            frame_id,
            spec,
            rows,
            cols,
            np.asarray(rays, np.float32),
            np.asarray(depths, np.float32),
            np.asarray(embeddings, np.float32),
        )
        self._write(topic, channel, at, fmt.encode_patch_frame(frame))

    def flush(self) -> None:
        self.appender.flush()

    def close(self) -> None:
        self.appender.close()
