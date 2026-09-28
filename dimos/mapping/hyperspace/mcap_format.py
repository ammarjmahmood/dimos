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

"""How hyperspace lives inside a ROS 2 mcap: topic names and the CDR wire format.

Everything hyperspace derives from a recording is written into that recording, as
plain ROS 2 messages anything can read:

- ``/depth2depth``: the filled depth of each keyframe, ``sensor_msgs/Image`` 16UC1 in
  millimetres, in the colour camera's optical frame.
- ``/depth_thumbnails``: the same depth sampled every ``THUMBNAIL_STRIDE`` pixels,
  also 16UC1 millimetres. Thumbnail pixel ``(r, c)`` is depth2depth pixel
  ``(r * stride, c * stride)``; zero means no depth.
- ``/siglip2_patches__m_<member>`` (``/pe_patches__m_<member>`` for a PE model): one
  ``dimos_msgs/msg/PatchEmbeddings`` per keyframe per model, the whole patch grid in
  one message. One message per frame rather than per patch, so reading a model's
  vectors is one ``np.frombuffer`` per frame instead of a million decodes.

The camera intrinsics are not repeated: they are the recording's own camera_info.
"""

from __future__ import annotations

from dataclasses import dataclass
import re
import struct

import numpy as np

DEPTH2DEPTH_TOPIC = "/depth2depth"
THUMBNAILS_TOPIC = "/depth_thumbnails"
SIGLIP_PATCH_PREFIX = "/siglip2_patches__m_"
PE_PATCH_PREFIX = "/pe_patches__m_"
PATCH_PREFIXES = (SIGLIP_PATCH_PREFIX, PE_PATCH_PREFIX)
THUMBNAIL_STRIDE = 4

IMAGE_TYPE = "sensor_msgs/msg/Image"
PATCH_TYPE = "dimos_msgs/msg/PatchEmbeddings"

_SEPARATOR = "=" * 80
_HEADER_DEPS = (
    f"\n{_SEPARATOR}\nMSG: std_msgs/Header\nbuiltin_interfaces/Time stamp\nstring frame_id\n"
    f"{_SEPARATOR}\nMSG: builtin_interfaces/Time\nint32 sec\nuint32 nanosec\n"
)
IMAGE_SCHEMA = (
    "std_msgs/Header header\nuint32 height\nuint32 width\nstring encoding\n"
    "uint8 is_bigendian\nuint32 step\nuint8[] data\n" + _HEADER_DEPS
).encode()
PATCH_SCHEMA = (
    "# One image's patch embeddings: a grid_rows x grid_cols grid of cells, row-major,\n"
    "# each with a view ray, a depth and an embedding.\n"
    "std_msgs/Header header\n"
    "# the checkpoint, as hyperspace names it: google/siglip2-...[@N] or pe:...\n"
    "string model\n"
    "uint32 grid_rows\n"
    "uint32 grid_cols\n"
    "uint32 dim\n"
    "# 2 per cell: the cell centre in normalised image coordinates (x/z, y/z)\n"
    "float32[] rays\n"
    "# 1 per cell: depth in metres along z, NaN where there was none\n"
    "float32[] depths\n"
    "# dim per cell\n"
    "float32[] embeddings\n" + _HEADER_DEPS
).encode()

_CDR_LE = b"\x00\x01\x00\x00"


def patch_topic(member: str) -> str:
    """``base-patch16-224`` -> ``/siglip2_patches__m_base_patch16_224``;
    ``pe-PE-Core-B-16`` -> ``/pe_patches__m_PE_Core_B_16``."""
    if member.startswith("pe-"):
        return PE_PATCH_PREFIX + _safe(member.removeprefix("pe-"))
    return SIGLIP_PATCH_PREFIX + _safe(member)


def is_patch_topic(topic: str) -> bool:
    return topic.startswith(PATCH_PREFIXES)


def _safe(text: str) -> str:
    return re.sub(r"[^A-Za-z0-9]+", "_", text).strip("_")


class _Writer:
    """Little-endian CDR, aligned from the end of the encapsulation header."""

    def __init__(self) -> None:
        self.out = bytearray(_CDR_LE)

    def _align(self, n: int) -> None:
        pad = (-(len(self.out) - 4)) % n
        self.out.extend(b"\x00" * pad)

    def u32(self, value: int) -> None:
        self._align(4)
        self.out.extend(struct.pack("<I", value))

    def i32(self, value: int) -> None:
        self._align(4)
        self.out.extend(struct.pack("<i", value))

    def u8(self, value: int) -> None:
        self.out.append(value)

    def string(self, value: str) -> None:
        raw = value.encode() + b"\x00"
        self.u32(len(raw))
        self.out.extend(raw)

    def header(self, stamp_ns: int, frame_id: str) -> None:
        self.i32(stamp_ns // 1_000_000_000)
        self.u32(stamp_ns % 1_000_000_000)
        self.string(frame_id)

    def bytes_seq(self, data: bytes) -> None:
        self.u32(len(data))
        self.out.extend(data)

    def f32_seq(self, values: np.ndarray) -> None:
        flat = np.ascontiguousarray(values, dtype="<f4").ravel()
        self.u32(flat.size)
        self._align(4)
        self.out.extend(flat.tobytes())


class _Reader:
    def __init__(self, buf: bytes | memoryview) -> None:
        if bytes(buf[:4]) != _CDR_LE:
            raise ValueError("not little-endian CDR")
        self.buf = memoryview(buf)
        self.at = 4

    def _align(self, n: int) -> None:
        self.at += (-(self.at - 4)) % n

    def u32(self) -> int:
        self._align(4)
        (value,) = struct.unpack_from("<I", self.buf, self.at)
        self.at += 4
        return int(value)

    def i32(self) -> int:
        self._align(4)
        (value,) = struct.unpack_from("<i", self.buf, self.at)
        self.at += 4
        return int(value)

    def u8(self) -> int:
        value = self.buf[self.at]
        self.at += 1
        return int(value)

    def string(self) -> str:
        n = self.u32()
        text = bytes(self.buf[self.at : self.at + n - 1]).decode()
        self.at += n
        return text

    def header(self) -> tuple[int, str]:
        sec = self.i32()
        nanosec = self.u32()
        return sec * 1_000_000_000 + nanosec, self.string()

    def bytes_seq(self) -> memoryview:
        n = self.u32()
        view = self.buf[self.at : self.at + n]
        self.at += n
        return view

    def f32_seq(self) -> np.ndarray:
        n = self.u32()
        self._align(4)
        array = np.frombuffer(self.buf, dtype="<f4", count=n, offset=self.at)
        self.at += 4 * n
        return array


def encode_depth_image(stamp_ns: int, frame_id: str, depth_mm: np.ndarray) -> bytes:
    """A uint16 millimetre depth map as ``sensor_msgs/Image`` 16UC1."""
    depth = np.ascontiguousarray(depth_mm, dtype="<u2")
    rows, cols = depth.shape
    w = _Writer()
    w.header(stamp_ns, frame_id)
    w.u32(rows)
    w.u32(cols)
    w.string("16UC1")
    w.u8(0)
    w.u32(cols * 2)
    w.bytes_seq(depth.tobytes())
    return bytes(w.out)


@dataclass(frozen=True)
class DepthImage:
    stamp_ns: int
    frame_id: str
    depth_mm: np.ndarray  # (rows, cols) uint16

    @property
    def ts(self) -> float:
        return self.stamp_ns / 1e9


def decode_depth_image(buf: bytes) -> DepthImage:
    r = _Reader(buf)
    stamp_ns, frame_id = r.header()
    rows = r.u32()
    cols = r.u32()
    encoding = r.string()
    bigendian = r.u8()
    step = r.u32()
    data = r.bytes_seq()
    if encoding not in ("16UC1", "mono16") or bigendian:
        raise ValueError(f"expected little-endian 16UC1 depth, got {encoding!r}")
    depth = np.frombuffer(data, dtype="<u2", count=rows * step // 2).reshape(rows, step // 2)
    return DepthImage(stamp_ns, frame_id, depth[:, :cols])


def thumbnail_of(depth_mm: np.ndarray, stride: int = THUMBNAIL_STRIDE) -> np.ndarray:
    return np.ascontiguousarray(depth_mm[::stride, ::stride])


@dataclass(frozen=True)
class PatchFrame:
    """One keyframe's patch grid from one model."""

    stamp_ns: int
    frame_id: str
    model: str
    rows: int
    cols: int
    rays: np.ndarray  # (cells, 2) float32
    depths: np.ndarray  # (cells,) float32, NaN = none
    embeddings: np.ndarray  # (cells, dim) float32

    @property
    def ts(self) -> float:
        return self.stamp_ns / 1e9

    @property
    def dim(self) -> int:
        return int(self.embeddings.shape[1])


def encode_patch_frame(frame: PatchFrame) -> bytes:
    cells = frame.rows * frame.cols
    if frame.rays.shape != (cells, 2) or frame.depths.shape != (cells,):
        raise ValueError("rays/depths do not match the grid")
    if frame.embeddings.shape[0] != cells:
        raise ValueError("embeddings do not match the grid")
    w = _Writer()
    w.header(frame.stamp_ns, frame.frame_id)
    w.string(frame.model)
    w.u32(frame.rows)
    w.u32(frame.cols)
    w.u32(frame.dim)
    w.f32_seq(frame.rays)
    w.f32_seq(frame.depths)
    w.f32_seq(frame.embeddings)
    return bytes(w.out)


def decode_patch_frame(buf: bytes) -> PatchFrame:
    """Zero-copy: the arrays are views into *buf*."""
    r = _Reader(buf)
    stamp_ns, frame_id = r.header()
    model = r.string()
    rows = r.u32()
    cols = r.u32()
    dim = r.u32()
    rays = r.f32_seq()
    depths = r.f32_seq()
    embeddings = r.f32_seq()
    cells = rows * cols
    if rays.size != 2 * cells or depths.size != cells or embeddings.size != cells * dim:
        raise ValueError(f"{PATCH_TYPE}: array lengths do not match a {rows}x{cols}x{dim} grid")
    return PatchFrame(
        stamp_ns,
        frame_id,
        model,
        rows,
        cols,
        rays.reshape(cells, 2),
        depths,
        embeddings.reshape(cells, dim),
    )
