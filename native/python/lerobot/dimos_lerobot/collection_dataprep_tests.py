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

"""End-to-end coverage from live collection through native LeRobot export."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import av
from dimos_lerobot.dataprep import inspect_dataset, write
import numpy as np
import pyarrow.parquet as pq
import pytest

from dimos.imitation.dataprep.build import run_dataprep
from dimos.imitation.dataprep.core import OutputConfig
from dimos.imitation.test_datacollection_e2e import (
    EXPECTED_ACTION,
    EXPECTED_STATE,
    _dataprep_config,
    recorded_session as recorded_session,
)


def _read_video(path: Path) -> list[np.ndarray[Any, Any]]:
    with av.open(str(path)) as container:
        return [frame.to_ndarray(format="rgb24") for frame in container.decode(video=0)]


def test_collection_to_lerobot_roundtrip(
    tmp_path: Path,
    recorded_session: tuple[Path, dict[float, np.ndarray[Any, Any]]],
) -> None:
    db_path, recorded_images = recorded_session
    lerobot_path = run_dataprep(
        _dataprep_config(
            db_path,
            OutputConfig(
                format="lerobot",
                path=tmp_path / "lerobot",
                metadata={"robot": "synthetic", "repo_id": "dimos/collection-test"},
            ),
        ),
        writer=write,
    )

    lerobot_info = inspect_dataset(lerobot_path)
    assert (lerobot_info["episodes"], lerobot_info["frames"], lerobot_info["fps"]) == (2, 6, 1.0)
    data = pq.read_table(lerobot_path / "data/chunk-000/file-000.parquet")
    assert data.column("timestamp").to_pylist() == pytest.approx([0.0, 1.0, 2.0, 0.0, 1.0, 2.0])
    assert data.column("episode_index").to_pylist() == [0, 0, 0, 1, 1, 1]
    assert data.column("frame_index").to_pylist() == [0, 1, 2, 0, 1, 2]
    np.testing.assert_array_equal(np.asarray(data.column("state").to_pylist()), EXPECTED_STATE)
    np.testing.assert_array_equal(np.asarray(data.column("action").to_pylist()), EXPECTED_ACTION)

    episode_rows = pq.read_table(
        lerobot_path / "meta/episodes/chunk-000/file-000.parquet"
    ).to_pylist()
    assert [row["length"] for row in episode_rows] == [3, 3]
    assert [(row["dataset_from_index"], row["dataset_to_index"]) for row in episode_rows] == [
        (0, 3),
        (3, 6),
    ]
    assert [row["tasks"] for row in episode_rows] == [["pick"], ["place"]]

    video = _read_video(lerobot_path / "videos/camera/chunk-000/file-000.mp4")
    assert len(video) == 6
    expected_means = [
        recorded_images[ts].mean() for ts in (100.0, 101.0, 102.0, 108.0, 109.0, 110.0)
    ]
    np.testing.assert_allclose([frame.mean() for frame in video], expected_means, atol=5.0)
