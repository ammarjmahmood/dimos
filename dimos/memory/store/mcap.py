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

"""Read-only memory store backed by an mcap file.

Generic and robot-independent. JPEG channels decode automatically because their
payload type is fixed. Other formats use a caller-supplied ``codecs`` map (wire
topic -> codec), while ``streams`` may map friendly stream names to topics. See
``dimos.robot.unitree.go2.dds.store.Go2McapStore`` for the Go2 DDS wiring.

Read-only: no append, blobs, vectors, or embeddings. Payloads decode lazily on
``obs.data``; ts and counts are cheap (counts come from the mcap index).
"""

from __future__ import annotations

from collections.abc import Iterable, Iterator, Mapping
from dataclasses import replace
from functools import partial
from typing import Any, Protocol, runtime_checkable

from dimos.memory.backend import Backend
from dimos.memory.codecs.base import codec_for
from dimos.memory.codecs.jpeg import JpegCodec
from dimos.memory.notifier.subject import SubjectNotifier
from dimos.memory.observationstore.base import ObservationStore, ObservationStoreConfig
from dimos.memory.store.base import Store, StoreConfig
from dimos.memory.type.filter import (
    AfterFilter,
    AtFilter,
    BeforeFilter,
    StreamQuery,
    TimeRangeFilter,
)
from dimos.memory.type.observation import Observation


@runtime_checkable
class StreamCodec(Protocol):
    """What the store needs to turn a channel's stored bytes into a payload."""

    @property
    def payload_type(self) -> type: ...

    def decode(self, data: bytes) -> Any: ...


class _BytesCodec:
    """Identity codec: hands back a codecless channel's stored bytes as ``Stream[bytes]``."""

    payload_type = bytes

    def decode(self, data: bytes) -> bytes:
        return data


_BYTES_CODEC = _BytesCodec()


def _slug(topic: str) -> str:
    """Auto stream name from a topic: drop the ``rt/`` prefix and ``/`` -> ``_``.

    ``rt/`` is the ROS2-over-DDS topic prefix; ``removeprefix`` only strips it
    where present (e.g. app-level ``control_log`` is left alone).
    """
    return topic.removeprefix("rt/").replace("/", "_")


def _time_window(q: StreamQuery) -> tuple[int | None, int | None]:
    """The log-time span (ns) every filter in *q* confines matches to, if any does."""
    low: float | None = None
    high: float | None = None

    def narrow(lo: float | None, hi: float | None) -> None:
        nonlocal low, high
        if lo is not None:
            low = lo if low is None else max(low, lo)
        if hi is not None:
            high = hi if high is None else min(high, hi)

    for f in q.filters:
        if isinstance(f, AtFilter):
            narrow(f.t - f.tolerance, f.t + f.tolerance)
        elif isinstance(f, TimeRangeFilter):
            narrow(f.t1, f.t2)
        elif isinstance(f, AfterFilter):
            narrow(f.t, None)
        elif isinstance(f, BeforeFilter):
            narrow(None, f.t)
    # a nanosecond of slack either side: the filters compare float seconds
    return (
        None if low is None else max(0, int(low * 1e9) - 1),
        None if high is None else int(high * 1e9) + 1,
    )


class McapObservationStoreConfig(ObservationStoreConfig):
    name: str = "<mcap>"


class McapObservationStore(ObservationStore[Any]):
    """Read-only metadata/query over one mcap channel. Payloads load lazily."""

    config: McapObservationStoreConfig

    def __init__(
        self,
        *,
        name: str,
        path: str,
        topic: str,
        codec: StreamCodec,
        count: int,
        observation_uses_publish_time: bool,
    ) -> None:
        super().__init__(name=name)
        self._path = path
        self._topic = topic
        self._codec = codec
        self._count = count
        # Immutable channel metadata: this store never writes and each iterator
        # owns its own file reader, so timestamp selection has no async state.
        self._observation_uses_publish_time = observation_uses_publish_time

    @property
    def name(self) -> str:
        return self.config.name

    def _iter(
        self, reverse: bool = False, window: tuple[int | None, int | None] = (None, None)
    ) -> Iterator[Observation[Any]]:
        """Every message, or only those logged inside *window* (ns, inclusive).

        A window is read through the chunk index, so looking up one moment of a long
        recording costs the chunks around it rather than a walk from the start. Ids
        are positions in the stream, and a windowed read does not know its position:
        its observations carry id -1.
        """
        from mcap.reader import make_reader  # optional mcap dependency

        from dimos.memory.store.mcap_append import load_summary

        decode, dtype, n = self._codec.decode, self._codec.payload_type, self._count
        start, end = window
        windowed = start is not None or end is not None
        with open(self._path, "rb") as f:
            reader = make_reader(f)
            load_summary(reader, self._path)
            msgs = reader.iter_messages(
                topics=[self._topic],
                reverse=reverse,
                start_time=start,
                # the reader's end_time is exclusive
                end_time=None if end is None else end + 1,
            )
            for i, (_schema, _channel, message) in enumerate(msgs):
                observation_time = (
                    message.publish_time
                    if self._observation_uses_publish_time
                    else message.log_time
                )
                yield Observation(
                    id=-1 if windowed else (n - 1 - i) if reverse else i,
                    ts=observation_time / 1e9,
                    data_type=dtype,
                    _loader=partial(decode, message.data),
                )

    def query(self, q: StreamQuery) -> Iterator[Observation[Any]]:
        # MCAP is natively log-time ordered, so id ordering never needs a sort.
        # Native DimOS recordings expose publish_time as observation ts; source
        # time can differ from log/reception order and must use the generic sort.
        # Log time is what the index is built on, so a time filter narrows the read
        # itself; the filters still run over what comes back, so the window only has
        # to be no narrower than they are.
        window = (None, None) if self._observation_uses_publish_time else _time_window(q)
        if q.order_field == "id" or (
            q.order_field == "ts" and not self._observation_uses_publish_time
        ):
            it = self._iter(reverse=q.order_desc, window=window)
            q = replace(q, order_field=None, order_desc=False)
            return q.apply(it)
        return q.apply(self._iter(window=window))

    def count(self, q: StreamQuery) -> int:
        if not q.filters and q.search_text is None and q.search_vec is None:
            n = self._count
            if q.offset_val:
                n = max(0, n - q.offset_val)
            if q.limit_val is not None:
                n = min(n, q.limit_val)
            return n
        return sum(1 for _ in self.query(q))

    def fetch_by_ids(self, ids: list[int]) -> list[Observation[Any]]:
        want = set(ids)
        return [o for o in self._iter() if o.id in want]

    def insert(self, obs: Observation[Any]) -> int:
        raise NotImplementedError("McapStore is read-only")


# Chunks decompressed at once by read_topic; a batch is what is held in memory.
READ_WORKERS = 8
READ_BATCH = 32


def read_topic(
    path: str,
    topics: Iterable[str],
    *,
    start: int | None = None,
    end: int | None = None,
    workers: int = READ_WORKERS,
) -> Iterator[tuple[int, bytes]]:
    """``(log_time_ns, data)`` for every message on *topics*, chunk by chunk, in file
    order, optionally only those logged in ``[start, end]`` (ns).

    The mcap reader decompresses one chunk at a time on one thread, which for a model's
    patch vectors -- incompressible floats, zstd'd anyway -- ran at 110 MB/s off an
    NVMe. Here the chunks the index says hold these topics are read with ``os.pread``
    and decompressed on a thread pool (both let go of the GIL), a batch at a time so
    only a batch is ever held. Within a chunk, messages come in the order written.
    """
    from concurrent.futures import ThreadPoolExecutor
    import os
    import struct

    from mcap.reader import make_reader

    from dimos.memory.store.mcap_append import load_summary

    with open(path, "rb") as f:
        summary = load_summary(make_reader(f), path)
    if summary is None:
        return
    wanted = {cid for cid, ch in summary.channels.items() if ch.topic in set(topics)}
    chunks = sorted(
        (
            index
            for index in summary.chunk_indexes
            if wanted & set(index.message_index_offsets)
            and (start is None or index.message_end_time >= start)
            and (end is None or index.message_start_time <= end)
        ),
        key=lambda index: index.chunk_start_offset,
    )
    if not chunks:
        return

    def messages_in(fd: int, index: Any) -> list[tuple[int, bytes]]:
        raw = os.pread(fd, index.chunk_length, index.chunk_start_offset)
        body = memoryview(raw)[9:]  # opcode + record length
        at = 8 + 8 + 8 + 4  # start, end, uncompressed size, crc
        (name_length,) = struct.unpack_from("<I", body, at)
        at += 4 + name_length
        (records_length,) = struct.unpack_from("<Q", body, at)
        at += 8
        records = body[at : at + records_length]
        if index.compression == "zstd":
            import zstandard

            records = memoryview(
                zstandard.ZstdDecompressor().decompress(
                    records, max_output_size=index.uncompressed_size
                )
            )
        elif index.compression == "lz4":
            import lz4.frame

            records = memoryview(lz4.frame.decompress(records))
        elif index.compression:
            raise ValueError(f"unsupported chunk compression {index.compression!r}")
        found = []
        at = 0
        while at + 9 <= len(records):
            opcode = records[at]
            (length,) = struct.unpack_from("<Q", records, at + 1)
            if opcode == 0x05:  # message
                channel, _, log_time = struct.unpack_from("<HIQ", records, at + 9)
                if (
                    channel in wanted
                    and (start is None or log_time >= start)
                    and (end is None or log_time <= end)
                ):
                    found.append((log_time, bytes(records[at + 9 + 22 : at + 9 + length])))
            at += 9 + length
        return found

    fd = os.open(path, os.O_RDONLY)
    try:
        with ThreadPoolExecutor(max_workers=workers) as pool:
            for first in range(0, len(chunks), READ_BATCH):
                batch = chunks[first : first + READ_BATCH]
                for found in pool.map(lambda index: messages_in(fd, index), batch):
                    yield from found
    finally:
        os.close(fd)


class McapStoreConfig(StoreConfig):
    path: str = ""


class McapStore(Store):
    """A memory store backed by an mcap file (read-only).

    Every channel present in the file with a codec is exposed. Names default to
    the slugified topic (see :func:`_slug`); ``streams`` (friendly name -> topic)
    overrides the name for specific topics.
    """

    config: McapStoreConfig

    def __init__(
        self,
        *,
        codecs: Mapping[str, StreamCodec] | None = None,
        streams: dict[str, str] | None = None,
        **kwargs: Any,
    ) -> None:
        from mcap.reader import make_reader  # optional mcap dependency

        super().__init__(**kwargs)
        self._codecs = dict(codecs or {})
        name_of = {topic: name for name, topic in (streams or {}).items()}  # topic -> override
        from dimos.memory.store.mcap_append import load_summary

        with open(self.config.path, "rb") as f:
            summary = load_summary(make_reader(f), self.config.path)
        self._stream_topic: dict[str, str] = {}  # stream name -> topic
        self._available: dict[str, int] = {}  # stream name -> message count
        self._observation_uses_publish_time: dict[str, bool] = {}
        # Channels with no registered codec are still exposed, as Stream[bytes] via
        # _BYTES_CODEC — reachable but undecoded. _raw maps their stream name to the
        # source schema so summary() can flag them [raw bytes: <schema>].
        self._raw: dict[str, str | None] = {}  # raw stream name -> source schema
        if summary is not None and summary.statistics is not None:
            for cid, ch in summary.channels.items():
                count = summary.statistics.channel_message_counts.get(cid, 0)
                name = name_of.get(ch.topic) or _slug(ch.topic)
                if ch.topic not in self._codecs and ch.message_encoding == "jpeg":
                    self._codecs[ch.topic] = JpegCodec()
                self._stream_topic[name] = ch.topic
                self._available[name] = count
                self._observation_uses_publish_time[name] = (
                    ch.metadata.get("dimos.observation_time") == "publish_time"
                )
                if ch.topic not in self._codecs:
                    sch = summary.schemas.get(ch.schema_id)
                    self._raw[name] = sch.name if sch else None

    def read_payloads(
        self, name: str, t1: float | None = None, t2: float | None = None
    ) -> Iterator[tuple[float, Any]]:
        """``(ts, payload)`` for a whole stream, or the part logged in ``[t1, t2]``, read
        with :func:`read_topic`: many times faster than iterating the stream for a
        large one, and in file order rather than as observations."""
        topic = self._stream_topic[name]
        codec = self._codecs.get(topic) or _BYTES_CODEC
        start = None if t1 is None else max(0, int(t1 * 1e9) - 1)
        end = None if t2 is None else int(t2 * 1e9) + 1
        for log_time, data in read_topic(self.config.path, [topic], start=start, end=end):
            ts = log_time / 1e9
            if (t1 is None or ts >= t1) and (t2 is None or ts <= t2):
                yield ts, codec.decode(data)

    def list_streams(self) -> list[str]:
        return sorted(set(self._available) | set(self._streams))

    def summary(self) -> str:
        """Base summary, tagging codecless streams with ``[raw bytes: <schema>]``."""
        lines = []
        for name, stream in self.streams.items():
            line = stream.summary()  # "Stream(\"name\"): ..."
            if name in self._raw:
                head = str(stream)  # "Stream(\"name\")"
                line = f"{head} [raw bytes: {self._raw[name] or '?'}]{line[len(head) :]}"
            lines.append(line)
        return "\n".join(lines)

    def _create_backend(
        self, name: str, payload_type: type | None = None, **config: Any
    ) -> Backend[Any]:
        if name not in self._available:
            raise KeyError(f"No stream {name!r}. Available: {sorted(self._available)}")
        topic = self._stream_topic[name]
        codec = self._codecs.get(topic) or _BYTES_CODEC  # no codec -> Stream[bytes]
        ptype = codec.payload_type
        obs = McapObservationStore(
            name=name,
            path=self.config.path,
            topic=topic,
            codec=codec,
            count=self._available[name],
            observation_uses_publish_time=self._observation_uses_publish_time[name],
        )
        return Backend(
            metadata_store=obs,
            codec=codec_for(ptype),  # storage codec, unused (blob_store=None)
            data_type=ptype,
            blob_store=None,
            vector_store=None,
            notifier=SubjectNotifier(),
        )
