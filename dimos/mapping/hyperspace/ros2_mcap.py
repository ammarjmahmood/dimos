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

"""Read a ROS 2 mcap recording as a dimos store: a codec per ROS schema.

Stream names are the topic without its leading slash, slashes turned into
underscores: ``/realsense_color_image/compressed`` reads as
``realsense_color_image_compressed``, ``/tf`` as ``tf``.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from dimos.mapping.hyperspace import mcap_format as fmt
from dimos.memory.store.mcap import McapStore
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.unitree.go2.dds import cdr, ros
from dimos.robot.unitree.go2.dds.codec import FnCodec

# ROS image encoding -> (dimos format, dtype, channels)
IMAGE_ENCODINGS: dict[str, tuple[ImageFormat, Any, int]] = {
    "rgb8": (ImageFormat.RGB, np.uint8, 3),
    "bgr8": (ImageFormat.BGR, np.uint8, 3),
    "rgba8": (ImageFormat.RGBA, np.uint8, 4),
    "bgra8": (ImageFormat.BGRA, np.uint8, 4),
    "mono8": (ImageFormat.GRAY, np.uint8, 1),
    "8UC1": (ImageFormat.GRAY, np.uint8, 1),
    "mono16": (ImageFormat.GRAY16, np.uint16, 1),
    "16UC1": (ImageFormat.DEPTH16, np.uint16, 1),
    "32FC1": (ImageFormat.DEPTH, np.float32, 1),
}


@dataclass
class _ImageWire:
    header: ros._Header
    height: int
    width: int
    encoding: str
    is_bigendian: int
    step: int
    data: np.ndarray

    __cdr_align__ = 1  # CDR aligns primitives, not structs
    __cdr_fields__ = [
        ("header", ros._Header),
        ("height", "u32"),
        ("width", "u32"),
        ("encoding", "string"),
        ("is_bigendian", "u8"),
        ("step", "u32"),
        ("data", ("seq", "u8")),
    ]


def decode_image(buf: bytes) -> Image:
    w: _ImageWire = cdr.decode(buf, _ImageWire)[0]
    if w.encoding not in IMAGE_ENCODINGS:
        raise ValueError(f"unsupported image encoding {w.encoding!r}")
    image_format, dtype, channels = IMAGE_ENCODINGS[w.encoding]
    itemsize = np.dtype(dtype).itemsize
    rows = np.frombuffer(w.data, dtype=np.uint8).reshape(w.height, w.step)  # step: padded rows
    pixels = np.ascontiguousarray(rows[:, : w.width * channels * itemsize]).view(dtype)
    if w.is_bigendian and itemsize > 1:
        pixels = pixels.byteswap().view(pixels.dtype.newbyteorder("="))
    shape = (w.height, w.width, channels) if channels > 1 else (w.height, w.width)
    return Image.from_numpy(
        pixels.reshape(shape), format=image_format, frame_id=w.header.frame_id, ts=ros._ts(w.header)
    )


@dataclass
class _RoiWire:
    x_offset: int
    y_offset: int
    height: int
    width: int
    do_rectify: int

    __cdr_align__ = 1  # CDR aligns primitives, not structs
    __cdr_fields__ = [
        ("x_offset", "u32"),
        ("y_offset", "u32"),
        ("height", "u32"),
        ("width", "u32"),
        ("do_rectify", "u8"),
    ]


@dataclass
class _CameraInfoWire:
    header: ros._Header
    height: int
    width: int
    distortion_model: str
    d: np.ndarray
    k: np.ndarray
    r: np.ndarray
    p: np.ndarray
    binning_x: int
    binning_y: int
    roi: _RoiWire

    __cdr_align__ = 1  # CDR aligns primitives, not structs
    __cdr_fields__ = [
        ("header", ros._Header),
        ("height", "u32"),
        ("width", "u32"),
        ("distortion_model", "string"),
        ("d", ("seq", "f64")),
        ("k", ("array", "f64", 9)),
        ("r", ("array", "f64", 9)),
        ("p", ("array", "f64", 12)),
        ("binning_x", "u32"),
        ("binning_y", "u32"),
        ("roi", _RoiWire),
    ]


def decode_camera_info(buf: bytes) -> CameraInfo:
    w: _CameraInfoWire = cdr.decode(buf, _CameraInfoWire)[0]
    info = CameraInfo(
        height=w.height,
        width=w.width,
        distortion_model=w.distortion_model,
        D=w.d.tolist(),
        K=w.k.tolist(),
        R=w.r.tolist(),
        P=w.p.tolist(),
        binning_x=w.binning_x,
        binning_y=w.binning_y,
        frame_id=w.header.frame_id,
        ts=ros._ts(w.header),
    )
    # CameraInfo.__init__ takes no roi and zeroes it, and the roi is what tells a crop
    # from a resize, so it is carried across by hand.
    info.roi_x_offset = int(w.roi.x_offset)
    info.roi_y_offset = int(w.roi.y_offset)
    info.roi_height = int(w.roi.height)
    info.roi_width = int(w.roi.width)
    info.roi_do_rectify = bool(w.roi.do_rectify)
    return info


@dataclass
class _TransformWire:
    translation: np.ndarray
    rotation: np.ndarray

    __cdr_align__ = 1  # CDR aligns primitives, not structs
    __cdr_fields__ = [("translation", ("array", "f64", 3)), ("rotation", ("array", "f64", 4))]


@dataclass
class _TransformStampedWire:
    header: ros._Header
    child_frame_id: str
    transform: _TransformWire

    __cdr_align__ = 1  # CDR aligns primitives, not structs
    __cdr_fields__ = [
        ("header", ros._Header),
        ("child_frame_id", "string"),
        ("transform", _TransformWire),
    ]


@dataclass
class _TfMessageWire:
    transforms: list[_TransformStampedWire]

    __cdr_align__ = 1  # CDR aligns primitives, not structs
    __cdr_fields__ = [("transforms", ("seq", _TransformStampedWire))]


def decode_tf_message(buf: bytes) -> TFMessage:
    w: _TfMessageWire = cdr.decode(buf, _TfMessageWire)[0]
    return TFMessage(
        *(
            Transform(
                translation=Vector3(*entry.transform.translation.tolist()),
                rotation=Quaternion(*entry.transform.rotation.tolist()),
                frame_id=entry.header.frame_id,
                child_frame_id=entry.child_frame_id,
                ts=ros._ts(entry.header),
            )
            for entry in w.transforms
        )
    )


# schema name -> codec
ROS2_CODECS: dict[str, FnCodec] = {
    "sensor_msgs/msg/Image": FnCodec(Image, decode_image),
    "sensor_msgs/msg/CompressedImage": FnCodec(Image, ros.decode_compressed_image),
    "sensor_msgs/msg/CameraInfo": FnCodec(CameraInfo, decode_camera_info),
    "tf2_msgs/msg/TFMessage": FnCodec(TFMessage, decode_tf_message),
    fmt.PATCH_TYPE: FnCodec(fmt.PatchFrame, fmt.decode_patch_frame),
}


def stream_name_of(topic: str) -> str:
    """``/realsense_color_image/compressed`` -> ``realsense_color_image_compressed``."""
    return topic.strip("/").removeprefix("rt/").replace("/", "_")


def is_ros2_mcap(path: str | Path) -> bool:
    return str(path).endswith(".mcap")


def open_ros2_mcap(path: str | Path) -> McapStore:
    """An mcap opened with codecs chosen by ROS 2 schema name. Read-only; hyperspace
    writes into it with :class:`dimos.mapping.hyperspace.mcap_sink.McapSink`."""
    from mcap.reader import make_reader

    from dimos.memory.store.mcap_append import load_summary

    with open(path, "rb") as handle:
        summary = load_summary(make_reader(handle), path)
    codecs: dict[str, Any] = {}
    names: dict[str, str] = {}
    if summary is not None:
        for channel in summary.channels.values():
            schema = summary.schemas.get(channel.schema_id)
            if schema is not None and schema.name in ROS2_CODECS:
                codecs[channel.topic] = ROS2_CODECS[schema.name]
            names[stream_name_of(channel.topic)] = channel.topic
    store = McapStore(path=str(path), codecs=codecs, streams=names)
    store.start()
    return store
