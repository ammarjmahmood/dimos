# Copyright 2025-2026 Dimensional Inc.
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

"""Native depth -> PointCloud2 (DepthCloud) and stereo pair -> depth + PointCloud2 (StereoCloud) modules."""

from __future__ import annotations

from typing import TYPE_CHECKING

from pydantic import Field, field_validator

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class DepthCloudConfig(NativeModuleConfig):
    cwd: str | None = "rust"
    executable: str = str(DIMOS_PROJECT_ROOT / "target" / "release" / "depth_cloud")
    build_command: str | None = "cargo build --release"
    stdin_config: bool = True
    base_fields: frozenset[str] = frozenset({"frame_id"})

    # Keep every Nth pixel on each axis, to fit the cloud to what a voxel map ray-caster can absorb.
    decimation: int = Field(default=4, ge=1)
    # Below min is the sensor's blind zone; beyond max, range error smears obstacles.
    min_range_m: float = Field(default=0.2, ge=0.0)
    max_range_m: float = Field(default=6.0, gt=0.0)
    # Multiplier onto metres for uint16 depth (0.001 for millimetres); float32 depth ignores it.
    depth_scale: float = Field(default=0.001, gt=0.0)
    # Empty rather than None so the native config gets a plain string; still falsy for Module.frame_id.
    frame_id: str | None = ""


class DepthCloud(NativeModule):
    """Depth image + intrinsics -> PointCloud2 in the camera's optical frame."""

    config: DepthCloudConfig

    depth: In[Image]
    camera_info: In[CameraInfo]
    cloud: Out[PointCloud2]


if TYPE_CHECKING:
    DepthCloud()


class StereoCloudConfig(NativeModuleConfig):
    cwd: str | None = "rust"
    executable: str = str(DIMOS_PROJECT_ROOT / "target" / "release" / "stereo_cloud")
    build_command: str | None = "cargo build --release"
    stdin_config: bool = True
    # Height bounds listed so None crosses as an explicit null, since the native config forbids absent keys.
    base_fields: frozenset[str] = frozenset({"frame_id", "min_height_m", "max_height_m"})

    # Distance between the left and right cameras; depth scales linearly with it.
    baseline_m: float = Field(gt=0.0)
    # Shrink each image by this before matching, trading depth resolution for speed.
    downscale: int = Field(default=4, ge=1, le=16)
    # Disparities searched, at the downscaled resolution; bounds the nearest measurable depth.
    disparity_range: int = Field(default=96, ge=8, le=512)
    p1: int = Field(default=8, ge=0, le=255)
    p2: int = Field(default=120, ge=1, le=4096)
    # Higher rejects more textureless regions, where stereo invents surfaces that become phantom obstacles.
    uniqueness: float = Field(default=0.10, ge=0.0, le=1.0)
    max_lr_difference: float = Field(default=1.5)
    # Drop isolated disparity patches smaller than this (pixels), which a voxel map would treat as obstacles; 0 or 1 disables.
    min_region: int = Field(default=350, ge=0)
    speckle_max_step: float = Field(default=1.5, ge=0.0)
    # Also aggregate along the diagonals: double the cost, but helps texture-poor surfaces like floors.
    diagonal_paths: bool = False
    # Below this, metres-per-pixel explodes and noise plants far-away obstacles.
    min_disparity_px: float = Field(default=1.0, ge=0.0)
    decimation: int = Field(default=2, ge=1)
    min_range_m: float = Field(default=0.3, ge=0.0)
    max_range_m: float = Field(default=7.0, gt=0.0)
    frame_id: str | None = ""
    # Maximum left/right timestamp gap for a pair; unsynchronised cameras smear anything moving.
    max_pair_skew_s: float = Field(default=0.05, ge=0.0, le=1.0)
    # Right camera's rotation relative to the left, which two monocular CameraInfos do not carry; zero means parallel.
    right_roll_rad: float = Field(default=0.0, ge=-0.2, le=0.2)
    right_pitch_rad: float = Field(default=0.0, ge=-0.2, le=0.2)
    right_yaw_rad: float = Field(default=0.0, ge=-0.2, le=0.2)
    # Depth denoise chain: filters joined by "+", each "name" or "name:arg" (see rust/src/denoise.rs); "none" disables.
    denoise: str = Field(default="median:8+plane:16:1+fill:8")
    # Keep only cloud points with base-frame height in [min, max]; None is no bound, and the depth image is never gated.
    min_height_m: float | None = None
    max_height_m: float | None = None
    # Pose of the camera's optical frame in the base frame (rpy extrinsic XYZ, as a URDF); only used by the height gate.
    base_from_camera_xyz_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    base_from_camera_rpy_rad: tuple[float, float, float] = (0.0, 0.0, 0.0)

    @field_validator("denoise")
    @classmethod
    def _denoise_parses(cls, chain: str) -> str:
        # Mirrors Chain::parse in rust/src/denoise.rs so a typo fails at blueprint time, naming the token.
        if chain in ("", "none"):
            return chain
        for token in chain.split("+"):
            name, _, argument = token.partition(":")
            if name not in _DENOISE_FILTERS:
                raise ValueError(
                    f"denoise={token!r}: {name!r} is not one of {sorted(_DENOISE_FILTERS)}"
                )
            numbers = argument.split(":") if argument else []
            if len(numbers) > (2 if name == "plane" else 1):
                raise ValueError(f"denoise={token!r}: too many arguments for {name!r}")
            for number in numbers:
                try:
                    float(number)
                except ValueError:
                    raise ValueError(f"denoise={token!r}: {number!r} is not a number") from None
        return chain


_DENOISE_FILTERS = frozenset(
    {"none", "median", "speckle", "bilateral", "coarse", "mean", "steep", "plane", "fill"}
)


class StereoCloud(NativeModule):
    """Left + right frames -> depth map + PointCloud2 by semi-global matching; depth is published to expose bad rectification."""

    config: StereoCloudConfig

    left: In[CompressedImage]
    right: In[CompressedImage]
    left_info: In[CameraInfo]
    right_info: In[CameraInfo]
    cloud: Out[PointCloud2]
    depth: Out[Image]
    depth_info: Out[CameraInfo]


if TYPE_CHECKING:
    StereoCloud()
