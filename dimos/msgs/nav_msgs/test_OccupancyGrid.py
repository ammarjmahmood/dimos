#!/usr/bin/env python3
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

"""Test the OccupancyGrid convenience class."""

from dimos_generated.std_msgs.msg import Header
import numpy as np
import pytest

from dimos.mapping.occupancy.inflation import simple_inflate
from dimos.mapping.pointclouds.occupancy import general_occupancy
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid, block_max_reduce
from dimos.msgs.occupancy import occupancy_view
from dimos.msgs.pointcloud import pointcloud_from_xyz


def test_empty_grid() -> None:
    """Test creating an empty grid."""
    grid = OccupancyGrid()
    assert grid.width == 0
    assert grid.height == 0
    assert grid.grid.shape == (0,)
    assert grid.total_cells == 0
    assert grid.frame_id == "world"


def test_grid_with_dimensions() -> None:
    """Test creating a grid with specified dimensions."""
    grid = OccupancyGrid(width=10, height=10, resolution=0.1, frame_id="map")
    assert grid.width == 10
    assert grid.height == 10
    assert grid.resolution == 0.1
    assert grid.frame_id == "map"
    assert grid.grid.shape == (10, 10)
    assert np.all(grid.grid == -1)  # All unknown
    assert grid.unknown_cells == 100
    assert grid.unknown_percent == 100.0


def test_grid_from_numpy_array() -> None:
    """Test creating a grid from a numpy array."""
    data = np.zeros((20, 30), dtype=np.int8)
    data[5:10, 10:20] = 100  # Add some obstacles
    data[15:18, 5:8] = -1  # Add unknown area

    origin = Pose(1.0, 2.0, 0.0)
    grid = OccupancyGrid(grid=data, resolution=0.05, origin=origin, frame_id="odom")

    assert grid.width == 30
    assert grid.height == 20
    assert grid.resolution == 0.05
    assert grid.frame_id == "odom"
    assert grid.origin.position.x == 1.0
    assert grid.origin.position.y == 2.0
    assert grid.grid.shape == (20, 30)

    # Check cell counts
    assert grid.occupied_cells == 50  # 5x10 obstacle area
    assert grid.free_cells == 541  # Total - occupied - unknown
    assert grid.unknown_cells == 9  # 3x3 unknown area

    # Check percentages (approximately)
    assert abs(grid.occupied_percent - 8.33) < 0.1
    assert abs(grid.free_percent - 90.17) < 0.1
    assert abs(grid.unknown_percent - 1.5) < 0.1


def test_world_grid_coordinate_conversion() -> None:
    """Test converting between world and grid coordinates."""
    data = np.zeros((20, 30), dtype=np.int8)
    origin = Pose(1.0, 2.0, 0.0)
    grid = OccupancyGrid(grid=data, resolution=0.05, origin=origin, frame_id="odom")

    # Test world to grid
    grid_pos = grid.world_to_grid((2.5, 3.0))
    assert int(grid_pos.x) == 30
    assert int(grid_pos.y) == 20

    # Test grid to world
    world_pos = grid.grid_to_world((10, 5))
    assert world_pos.x == 1.5
    assert world_pos.y == 2.25


def test_lcm_encode_decode() -> None:
    """Test LCM encoding and decoding."""
    data = np.zeros((20, 30), dtype=np.int8)
    data[5:10, 10:20] = 100  # Add some obstacles
    data[15:18, 5:8] = -1  # Add unknown area
    origin = Pose(1.0, 2.0, 0.0)
    grid = OccupancyGrid(grid=data, resolution=0.05, origin=origin, frame_id="odom")

    # Set a specific value for testing
    # Convert world coordinates to grid indices
    grid_pos = grid.world_to_grid((1.5, 2.25))
    grid.grid[int(grid_pos.y), int(grid_pos.x)] = 50

    # Encode
    lcm_data = grid.lcm_encode()
    assert isinstance(lcm_data, bytes)
    assert len(lcm_data) > 0

    # Decode
    decoded = OccupancyGrid.lcm_decode(lcm_data)

    # Check that data matches exactly (grid arrays should be identical)
    assert np.array_equal(grid.grid, decoded.grid)
    assert grid.width == decoded.width
    assert grid.height == decoded.height
    assert abs(grid.resolution - decoded.resolution) < 1e-6  # Use approximate equality for floats
    assert abs(grid.origin.position.x - decoded.origin.position.x) < 1e-6
    assert abs(grid.origin.position.y - decoded.origin.position.y) < 1e-6
    assert grid.frame_id == decoded.frame_id

    # Check that the actual grid data was preserved (don't rely on float conversions)
    assert decoded.grid[5, 10] == 50  # Value we set should be preserved in grid


def test_lcm_decode_origin_is_dimos_pose() -> None:
    """The decoded origin must be the dimos Pose (with .yaw), not the raw
    generated pose from the wire (the relay costmap encoder reads origin.yaw)."""
    quat = Quaternion.from_euler(Vector3(0.0, 0.0, 0.5))
    origin = Pose(1.0, 2.0, 0.0, quat.x, quat.y, quat.z, quat.w)
    grid = OccupancyGrid(grid=np.zeros((2, 3), dtype=np.int8), resolution=0.05, origin=origin)

    decoded = OccupancyGrid.lcm_decode(grid.lcm_encode())

    assert isinstance(decoded.origin, Pose)
    assert decoded.origin.yaw == pytest.approx(0.5)


def test_lcm_decode_empty_grid() -> None:
    """An empty grid (mapper warming up) must survive the wire; the decoder
    used to rebuild it as a 1-D array the constructor rejects."""
    decoded = OccupancyGrid.lcm_decode(OccupancyGrid().lcm_encode())
    assert decoded.grid.size == 0
    assert isinstance(decoded.origin, Pose)


def test_string_representation() -> None:
    """Test string representations."""
    grid = OccupancyGrid(width=10, height=10, resolution=0.1, frame_id="map")

    # Test __str__
    str_repr = str(grid)
    assert "OccupancyGrid[map]" in str_repr
    assert "10x10" in str_repr
    assert "1.0x1.0m" in str_repr
    assert "10cm res" in str_repr

    # Test __repr__
    repr_str = repr(grid)
    assert "OccupancyGrid(" in repr_str
    assert "width=10" in repr_str
    assert "height=10" in repr_str
    assert "resolution=0.1" in repr_str


def test_grid_property_sync() -> None:
    """Test that the grid property works correctly."""
    grid = OccupancyGrid(width=5, height=5, resolution=0.1, frame_id="map")

    # Modify via numpy array
    grid.grid[2, 3] = 100
    assert grid.grid[2, 3] == 100

    # Check that we can access grid values
    grid.grid[0, 0] = 50
    assert grid.grid[0, 0] == 50


def test_invalid_grid_dimensions() -> None:
    """Test handling of invalid grid dimensions."""
    # Test with non-2D array
    with pytest.raises(ValueError, match="Grid must be a 2D array"):
        OccupancyGrid(grid=np.zeros(10), resolution=0.1)


def test_from_pointcloud() -> None:
    """Test creating OccupancyGrid from PointCloud2."""
    # A deterministic raised cluster exercises occupancy without legacy LCM assets.
    x, y = np.meshgrid(np.arange(10) * 0.05, np.arange(10) * 0.05)
    points = np.column_stack((x.ravel(), y.ravel(), np.full(x.size, 0.5)))
    pointcloud = pointcloud_from_xyz(points, header=Header(frame_id="world"))

    # Convert pointcloud to occupancy grid
    occupancygrid = general_occupancy(pointcloud, resolution=0.05, min_height=0.1, max_height=2.0)
    # Apply inflation separately if needed
    occupancygrid = simple_inflate(occupancygrid, 0.1)

    # Check that grid was created with reasonable properties
    assert occupancygrid.info.width > 0
    assert occupancygrid.info.height > 0
    assert occupancygrid.info.resolution == pytest.approx(0.05)
    assert occupancygrid.header.frame_id == pointcloud.header.frame_id
    assert (
        np.count_nonzero(occupancy_view(occupancygrid) == 100) > 0
    )  # Should have some occupied cells


def test_filter_above() -> None:
    """Test filtering cells above threshold."""
    # Create test grid with various values
    data = np.array(
        [[-1, 0, 20, 50], [10, 30, 60, 80], [40, 70, 90, 100], [-1, 15, 25, -1]], dtype=np.int8
    )

    grid = OccupancyGrid(grid=data, resolution=0.1)

    # Filter to keep only values > 50
    filtered = grid.filter_above(50)

    # Check that values > 50 are preserved
    assert filtered.grid[1, 2] == 60
    assert filtered.grid[1, 3] == 80
    assert filtered.grid[2, 1] == 70
    assert filtered.grid[2, 2] == 90
    assert filtered.grid[2, 3] == 100

    # Check that values <= 50 are set to -1 (unknown)
    assert filtered.grid[0, 1] == -1  # was 0
    assert filtered.grid[0, 2] == -1  # was 20
    assert filtered.grid[0, 3] == -1  # was 50
    assert filtered.grid[1, 0] == -1  # was 10
    assert filtered.grid[1, 1] == -1  # was 30
    assert filtered.grid[2, 0] == -1  # was 40

    # Check that unknown cells are preserved
    assert filtered.grid[0, 0] == -1
    assert filtered.grid[3, 0] == -1
    assert filtered.grid[3, 3] == -1

    # Check dimensions and metadata preserved
    assert filtered.width == grid.width
    assert filtered.height == grid.height
    assert filtered.resolution == grid.resolution
    assert filtered.frame_id == grid.frame_id


def test_filter_below() -> None:
    """Test filtering cells below threshold."""
    # Create test grid with various values
    data = np.array(
        [[-1, 0, 20, 50], [10, 30, 60, 80], [40, 70, 90, 100], [-1, 15, 25, -1]], dtype=np.int8
    )

    grid = OccupancyGrid(grid=data, resolution=0.1)

    # Filter to keep only values < 50
    filtered = grid.filter_below(50)

    # Check that values < 50 are preserved
    assert filtered.grid[0, 1] == 0
    assert filtered.grid[0, 2] == 20
    assert filtered.grid[1, 0] == 10
    assert filtered.grid[1, 1] == 30
    assert filtered.grid[2, 0] == 40
    assert filtered.grid[3, 1] == 15
    assert filtered.grid[3, 2] == 25

    # Check that values >= 50 are set to -1 (unknown)
    assert filtered.grid[0, 3] == -1  # was 50
    assert filtered.grid[1, 2] == -1  # was 60
    assert filtered.grid[1, 3] == -1  # was 80
    assert filtered.grid[2, 1] == -1  # was 70
    assert filtered.grid[2, 2] == -1  # was 90
    assert filtered.grid[2, 3] == -1  # was 100

    # Check that unknown cells are preserved
    assert filtered.grid[0, 0] == -1
    assert filtered.grid[3, 0] == -1
    assert filtered.grid[3, 3] == -1

    # Check dimensions and metadata preserved
    assert filtered.width == grid.width
    assert filtered.height == grid.height
    assert filtered.resolution == grid.resolution
    assert filtered.frame_id == grid.frame_id


def test_max() -> None:
    """Test setting all non-unknown cells to maximum."""
    # Create test grid with various values
    data = np.array(
        [[-1, 0, 20, 50], [10, 30, 60, 80], [40, 70, 90, 100], [-1, 15, 25, -1]], dtype=np.int8
    )

    grid = OccupancyGrid(grid=data, resolution=0.1)

    # Apply max
    maxed = grid.max()

    # Check that all non-unknown cells are set to 100
    assert maxed.grid[0, 1] == 100  # was 0
    assert maxed.grid[0, 2] == 100  # was 20
    assert maxed.grid[0, 3] == 100  # was 50
    assert maxed.grid[1, 0] == 100  # was 10
    assert maxed.grid[1, 1] == 100  # was 30
    assert maxed.grid[1, 2] == 100  # was 60
    assert maxed.grid[1, 3] == 100  # was 80
    assert maxed.grid[2, 0] == 100  # was 40
    assert maxed.grid[2, 1] == 100  # was 70
    assert maxed.grid[2, 2] == 100  # was 90
    assert maxed.grid[2, 3] == 100  # was 100 (already max)
    assert maxed.grid[3, 1] == 100  # was 15
    assert maxed.grid[3, 2] == 100  # was 25

    # Check that unknown cells are preserved
    assert maxed.grid[0, 0] == -1
    assert maxed.grid[3, 0] == -1
    assert maxed.grid[3, 3] == -1

    # Check dimensions and metadata preserved
    assert maxed.width == grid.width
    assert maxed.height == grid.height
    assert maxed.resolution == grid.resolution
    assert maxed.frame_id == grid.frame_id

    # Verify statistics
    assert maxed.unknown_cells == 3  # Same as original
    assert maxed.occupied_cells == 13  # All non-unknown cells
    assert maxed.free_cells == 0  # No free cells


def test_block_max_reduce_preserves_lone_obstacle() -> None:
    cells = np.zeros((10, 10), dtype=np.int8)
    cells[3, 4] = 100
    reduced = block_max_reduce(cells, 5)
    assert reduced.shape == (2, 2)
    assert reduced.dtype == np.int8
    assert reduced[0, 0] == 100
    assert reduced[0, 1] == 0


def test_block_max_reduce_unknown_only_when_whole_block_unknown() -> None:
    cells = np.array([[-1, -1, -1, 50], [-1, -1, 0, -1]], dtype=np.int8)
    reduced = block_max_reduce(cells, 2)
    assert reduced.tolist() == [[-1, 50]]
    assert reduced.dtype == np.int8


def test_block_max_reduce_trims_remainder() -> None:
    cells = np.arange(35, dtype=np.int8).reshape(7, 5)
    reduced = block_max_reduce(cells, 2)
    # 7x5 at factor 2 keeps rows 0-5 and cols 0-3 (origin corner side).
    assert reduced.shape == (3, 2)
    assert reduced[0, 0] == 6  # max of rows 0-1, cols 0-1
    assert reduced[2, 1] == 28  # max of rows 4-5, cols 2-3


def test_block_max_reduce_thin_grid_passes_through() -> None:
    cells = np.zeros((1, 10), dtype=np.int8)
    assert block_max_reduce(cells, 5) is cells
