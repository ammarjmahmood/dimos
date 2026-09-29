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

"""Hyperspace in a ROS 2 mcap: written in place by an ingest, read back by a query."""

from __future__ import annotations

from pathlib import Path

from mcap.reader import make_reader
from mcap.writer import Writer
import numpy as np
import pytest

from dimos.mapping.hyperspace import mcap_format as fmt, patches as hs
from dimos.mapping.hyperspace.ingest import IngestConfig, PatchIngestor
from dimos.mapping.hyperspace.mcap_sink import McapSink, hyperspace_topics_in
from dimos.mapping.hyperspace.ros2_mcap import open_ros2_mcap
from dimos.mapping.hyperspace.test_module import (
    CAMERA,
    HEIGHT,
    OBJECT,
    SIDE,
    WIDTH,
    WORLD,
    StubModel,
    camera_info,
    quaternion_of,
    ring,
)
from dimos.msgs.sensor_msgs.Image import Image


def _recording(path: Path) -> None:
    """A finished mcap to append to: one raw channel, as any recording has."""
    with open(path, "wb") as f:
        writer = Writer(f)
        writer.start()
        channel = writer.register_channel("/camera/raw", "raw", 0)
        for i in range(5):
            writer.add_message(
                channel, log_time=(10 + i) * 10**9, publish_time=(10 + i) * 10**9, data=bytes([i])
            )
        writer.finish()


def _topics(path: Path) -> dict[str, tuple[int, str]]:
    counts: dict[str, tuple[int, str]] = {}
    with open(path, "rb") as f:
        for schema, channel, _ in make_reader(f, validate_crcs=True).iter_messages():
            n, _ = counts.get(channel.topic, (0, ""))
            counts[channel.topic] = (n + 1, schema.name if schema else "")
    return counts


def _ingest_into(
    path: Path, poses: list[np.ndarray], keep: frozenset[str] = frozenset()
) -> PatchIngestor:
    """The flat ingest, written through a sink into *path*. Depth2depth is stood in for
    by the sensor depth itself: what is under test is where it lands, not the model."""
    model = StubModel()
    config = IngestConfig(
        gate=hs.KeyframeGateConfig(
            lookahead=0,
            novelty_threshold=0.0,
            patch_novelty_threshold=None,
            max_angular_velocity=None,
            max_dark_fraction=None,
            min_interval=None,
        ),
        min_frame_interval_s=0.0,
        flat=True,
        depth2depth_model="stub",
    )
    sink = McapSink(path, keep=keep)
    ingestor = PatchIngestor(None, model, config, copy_tf=False, sink=sink)  # type: ignore[arg-type]
    ingestor._fused_depth = lambda rgb, depth: depth  # type: ignore[method-assign]
    ingestor.add_camera_info(camera_info())
    for index, pose in enumerate(poses):
        ts = 10.0 + index
        local = np.linalg.inv(pose) @ np.append(OBJECT, 1.0)
        model.pixel = (local[0] / local[2] * 48.0 + 32.0, local[1] / local[2] * 48.0 + 24.0)
        ingestor.add_depth(
            Image.from_numpy(
                np.full((HEIGHT, WIDTH), int(local[2] * 1000), dtype=np.uint16),
                frame_id=CAMERA,
                ts=ts,
            )
        )
        ingestor.add_image(
            Image.from_numpy(
                np.full((HEIGHT, WIDTH, 3), 128, dtype=np.uint8), frame_id=CAMERA, ts=ts
            )
        )
    ingestor.flush()
    sink.close()
    return ingestor


def _append_tf(path: Path, poses: list[np.ndarray]) -> None:
    """The world->camera transforms, as a ROS 2 /tf the store can decode."""
    from dimos.memory.store.mcap_append import ChannelSpec, McapAppender

    schema = (
        b"geometry_msgs/TransformStamped[] transforms\n"
        b"================================================================================\n"
        b"MSG: geometry_msgs/TransformStamped\nstd_msgs/Header header\nstring child_frame_id\n"
        b"geometry_msgs/Transform transform\n"
    )
    with McapAppender(path) as appender:
        channel = appender.add_channel(
            ChannelSpec(
                "/tf",
                "cdr",
                schema_name="tf2_msgs/msg/TFMessage",
                schema_encoding="ros2msg",
                schema_data=schema,
            )
        )
        for index, pose in enumerate(poses):
            stamp = (10 + index) * 10**9
            writer = fmt._Writer()
            writer.u32(1)
            writer.header(stamp, WORLD)
            writer.string(CAMERA)
            writer._align(8)
            x, y, z, w = quaternion_of(pose[:3, :3])
            writer.out.extend(np.array([*pose[:3, 3], x, y, z, w], dtype="<f8").tobytes())
            appender.add_message(channel, log_time=stamp, data=bytes(writer.out))


def test_the_wire_format_round_trips_and_refuses_a_grid_it_does_not_hold() -> None:
    depth = np.arange(12, dtype=np.uint16).reshape(3, 4)
    image = fmt.decode_depth_image(fmt.encode_depth_image(5_000_000_001, "cam", depth))
    assert image.frame_id == "cam" and image.stamp_ns == 5_000_000_001
    assert np.array_equal(image.depth_mm, depth)

    frame = fmt.PatchFrame(
        7,
        "cam",
        "google/siglip2-base-patch16-224",
        2,
        3,
        np.ones((6, 2), np.float32),
        np.array([1, 2, np.nan, 4, 5, 6], np.float32),
        np.arange(42, dtype=np.float32).reshape(6, 7),
    )
    back = fmt.decode_patch_frame(fmt.encode_patch_frame(frame))
    assert (back.rows, back.cols, back.dim, back.model) == (2, 3, 7, frame.model)
    assert np.array_equal(back.embeddings, frame.embeddings)
    assert np.isnan(back.depths[2])

    with pytest.raises(ValueError):
        fmt.encode_patch_frame(
            fmt.PatchFrame(7, "cam", "m", 2, 3, frame.rays, frame.depths, frame.embeddings[:5])
        )


def test_topic_names_carry_the_model_family() -> None:
    assert (
        fmt.patch_topic("so400m-patch16-naflex-1024")
        == "/siglip2_patches__m_so400m_patch16_naflex_1024"
    )
    assert fmt.patch_topic("pe-PE-Core-B-16") == "/pe_patches__m_PE_Core_B_16"


def test_an_ingest_into_an_mcap_writes_ros_messages_in_place(tmp_path: Path) -> None:
    path = tmp_path / "rec.mcap"
    _recording(path)
    ingestor = _ingest_into(path, ring(3, 2.5))
    kept = ingestor.stats["kept"]
    assert kept == 3

    topics = _topics(path)
    assert topics["/camera/raw"][0] == 5, "nothing already in the recording moved"
    assert topics[fmt.DEPTH2DEPTH_TOPIC] == (kept, fmt.IMAGE_TYPE)
    assert topics[fmt.THUMBNAILS_TOPIC] == (kept, fmt.IMAGE_TYPE)
    assert topics["/siglip2_patches__m_stub"] == (kept, fmt.PATCH_TYPE), "one message per frame"
    assert not any(topic.startswith("/hyperspace_") for topic in topics)
    assert hyperspace_topics_in(path) == sorted(
        [fmt.DEPTH2DEPTH_TOPIC, fmt.THUMBNAILS_TOPIC, "/siglip2_patches__m_stub"]
    )

    with open(path, "rb") as f:
        messages = {
            channel.topic: message
            for _, channel, message in make_reader(f).iter_messages()
            if channel.topic != "/camera/raw"
        }
    frame = fmt.decode_patch_frame(messages["/siglip2_patches__m_stub"].data)
    assert (frame.rows, frame.cols, frame.dim) == (SIDE, SIDE, 8)
    assert frame.frame_id == CAMERA and frame.model == "stub"
    assert np.isfinite(frame.depths).all() and (frame.depths > 0).all()
    thumbnail = fmt.decode_depth_image(messages[fmt.THUMBNAILS_TOPIC].data).depth_mm
    assert thumbnail.shape == (HEIGHT // fmt.THUMBNAIL_STRIDE, WIDTH // fmt.THUMBNAIL_STRIDE)
    assert (thumbnail > 0).all()


def test_adding_a_model_keeps_the_depth_already_there(tmp_path: Path) -> None:
    path = tmp_path / "rec.mcap"
    _recording(path)
    _ingest_into(path, ring(2, 2.5), keep=frozenset({fmt.DEPTH2DEPTH_TOPIC, fmt.THUMBNAILS_TOPIC}))
    topics = _topics(path)
    assert "/siglip2_patches__m_stub" in topics
    assert fmt.DEPTH2DEPTH_TOPIC not in topics and fmt.THUMBNAILS_TOPIC not in topics


def test_a_query_reads_the_patches_and_depth_back_out_of_the_mcap(tmp_path: Path) -> None:
    from dimos.mapping.hyperspace.detect import DetectConfig, RecordingFrames
    from dimos.mapping.hyperspace.frames import BACKGROUND_PROMPTS, hot_frames, member_streams
    from dimos.mapping.hyperspace.resident import ResidentIndex

    class StubTowers:
        def query(self, spec: str, text: str) -> np.ndarray:
            del spec
            return StubModel.embed_text(text)

        def background(self, spec: str) -> np.ndarray:
            del spec
            return np.stack([StubModel.embed_text(prompt) for prompt in BACKGROUND_PROMPTS])

        def close(self) -> None:
            pass

    path = tmp_path / "rec.mcap"
    _recording(path)
    poses = ring(4, 2.5)
    ingestor = _ingest_into(path, poses)
    _append_tf(path, poses)
    store = open_ros2_mcap(path)
    try:
        assert member_streams(store) == [("stub", "siglip2_patches__m_stub")]
        held = ResidentIndex()
        held.device = "cpu"
        held.warm(store, member_streams(store))
        found = hot_frames(store, "object", towers=StubTowers(), resident=held)
        assert len(found) == ingestor.stats["kept"], "every kept frame saw the object"
        assert all(len(frame.hits) == 1 for frame in found)
        hit = found[0].hits[0]
        assert hit.frame == CAMERA and hit.grid == (SIDE, SIDE)
        assert np.isfinite(hit.depth) and hit.depth > 0

        frames = RecordingFrames(store, config=DetectConfig(world_frame=WORLD, depth2depth=""))
        depth = frames.depth(CAMERA, found[0].ts)
        assert depth is not None and depth.shape == (HEIGHT, WIDTH), "off /depth2depth"
        assert np.allclose(depth, depth.flat[0]) and depth.flat[0] > 0
        pose = frames.pose(CAMERA, found[0].ts, WORLD)
        assert pose is not None and np.allclose(pose[:3, 3], poses[0][:3, 3], atol=1e-6)
    finally:
        store.stop()


def test_a_time_window_is_read_through_the_index(tmp_path: Path) -> None:
    path = tmp_path / "rec.mcap"
    _recording(path)
    store = open_ros2_mcap(path)
    try:
        stream = store.streams["camera_raw"]
        found = stream.at(12.0, tolerance=0.5).to_list()
        assert [float(o.ts) for o in found] == [12.0]
        assert [o.id for o in found] == [-1], "a windowed read does not know its position"
        assert [float(o.ts) for o in stream.after(12.5).to_list()] == [13.0, 14.0]
        assert len(stream.to_list()) == 5 and [o.id for o in stream.to_list()] == list(range(5))
    finally:
        store.stop()
