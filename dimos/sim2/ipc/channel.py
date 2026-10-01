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

"""Coherent double-buffered shared-memory robot channel."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from multiprocessing import resource_tracker
from multiprocessing.shared_memory import SharedMemory
import os
import struct
from typing import Any, Literal

import numpy as np
from numpy.typing import NDArray

from dimos.sim2.ipc.abi import (
    ABI_VERSION,
    CHANNEL_HEADER_SIZE,
    CHANNEL_MAGIC,
    ChannelDescriptor,
    FrameLayout,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_HEADER = struct.Struct("<8sIIIIQQQQQ")
_FRAME_META = struct.Struct("<QQQQdQ")
_ACTION_ACTIVE_OFFSET = 12
_OBSERVATION_ACTIVE_OFFSET = 16
_LIFECYCLE_OFFSET = 20
_ACTION_SEQUENCE_OFFSET = 24
_OBSERVATION_SEQUENCE_OFFSET = 32
_EPISODE_OFFSET = 40
_TRACKER_PID_OFFSET = 48
_OWNER_PID_OFFSET = 56

LifecycleState = Literal["starting", "ready", "faulted", "closed"]
_LIFECYCLE_TO_INT: dict[LifecycleState, int] = {
    "starting": 0,
    "ready": 1,
    "faulted": 2,
    "closed": 3,
}
_INT_TO_LIFECYCLE = {value: key for key, value in _LIFECYCLE_TO_INT.items()}


@dataclass(frozen=True)
class FrameMetadata:
    sequence: int
    episode_id: int
    physics_tick: int
    control_tick: int
    sim_time: float
    applied_action_sequence: int = 0


@dataclass(frozen=True)
class ChannelFrame:
    metadata: FrameMetadata
    values: dict[str, NDArray[Any]]


class RobotChannel:
    """One action writer and one observation writer sharing coherent frames."""

    def __init__(self, descriptor: ChannelDescriptor, shm: SharedMemory, *, owner: bool) -> None:
        if descriptor.abi_version != ABI_VERSION:
            raise ValueError(
                f"unsupported sim2 ABI {descriptor.abi_version}; expected {ABI_VERSION}"
            )
        if shm.size < descriptor.total_size:
            raise ValueError(
                f"shared memory '{descriptor.shm_name}' is {shm.size} bytes, "
                f"expected at least {descriptor.total_size}"
            )
        self.descriptor = descriptor
        self._shm = shm
        self._owner = owner
        self._closed = False

    @property
    def _buffer(self) -> memoryview[int]:
        buffer = self._shm.buf
        if buffer is None:
            raise RuntimeError("shared memory buffer is closed")
        return buffer

    @classmethod
    def create(cls, descriptor: ChannelDescriptor) -> RobotChannel:
        """Create the shared-memory segment for `descriptor`; this process owns it.

        A segment of the same name whose creating process has died is removed
        and recreated, with one warning naming it. If that process is still
        running, another simulation with the same sim_id is up and a
        RuntimeError says so.
        """
        try:
            shm = SharedMemory(name=descriptor.shm_name, create=True, size=descriptor.total_size)
        except FileExistsError:
            _unlink_stale(descriptor)
            shm = SharedMemory(name=descriptor.shm_name, create=True, size=descriptor.total_size)
        channel = cls(descriptor, shm, owner=True)
        channel._buffer[: descriptor.total_size] = bytes(descriptor.total_size)
        _HEADER.pack_into(
            channel._buffer,
            0,
            CHANNEL_MAGIC,
            ABI_VERSION,
            0,
            0,
            _LIFECYCLE_TO_INT["starting"],
            0,
            0,
            0,
            _resource_tracker_pid(),
            os.getpid(),
        )
        return channel

    @classmethod
    def attach(cls, descriptor: ChannelDescriptor) -> RobotChannel:
        shm = SharedMemory(name=descriptor.shm_name, create=False)
        channel = cls(descriptor, shm, owner=False)
        magic, abi = struct.unpack_from("<8sI", channel._buffer, 0)
        if magic != CHANNEL_MAGIC or abi != ABI_VERSION:
            channel.close()
            raise ValueError(f"invalid sim2 channel header for robot '{descriptor.robot_id}'")
        owner_tracker_pid = struct.unpack_from("<Q", channel._buffer, _TRACKER_PID_OFFSET)[0]
        if owner_tracker_pid != _resource_tracker_pid():
            _untrack(shm)
        return channel

    @property
    def owner_pid(self) -> int:
        """Process id of the process that created this segment."""
        return int(struct.unpack_from("<Q", self._buffer, _OWNER_PID_OFFSET)[0])

    @property
    def lifecycle(self) -> LifecycleState:
        value = struct.unpack_from("<I", self._buffer, _LIFECYCLE_OFFSET)[0]
        return _INT_TO_LIFECYCLE.get(value, "faulted")

    def set_lifecycle(self, state: LifecycleState) -> None:
        struct.pack_into("<I", self._buffer, _LIFECYCLE_OFFSET, _LIFECYCLE_TO_INT[state])

    @property
    def episode_id(self) -> int:
        return int(struct.unpack_from("<Q", self._buffer, _EPISODE_OFFSET)[0])

    def set_episode(self, episode_id: int) -> None:
        struct.pack_into("<Q", self._buffer, _EPISODE_OFFSET, episode_id)

    def publish_action(self, values: dict[str, Any], metadata: FrameMetadata) -> int:
        return self._publish("action", values, metadata)

    def publish_observation(self, values: dict[str, Any], metadata: FrameMetadata) -> int:
        return self._publish("observation", values, metadata)

    def read_action(self, *, retries: int = 20) -> ChannelFrame | None:
        return self._read("action", retries=retries)

    def read_observation(self, *, retries: int = 20) -> ChannelFrame | None:
        return self._read("observation", retries=retries)

    def reset_frames(self, episode_id: int) -> None:
        self.set_episode(episode_id)
        struct.pack_into("<I", self._buffer, _ACTION_ACTIVE_OFFSET, 0)
        struct.pack_into("<I", self._buffer, _OBSERVATION_ACTIVE_OFFSET, 0)
        struct.pack_into("<Q", self._buffer, _ACTION_SEQUENCE_OFFSET, 0)
        struct.pack_into("<Q", self._buffer, _OBSERVATION_SEQUENCE_OFFSET, 0)
        start = CHANNEL_HEADER_SIZE
        self._buffer[start : self.descriptor.total_size] = bytes(self.descriptor.total_size - start)

    def _publish(
        self,
        direction: Literal["action", "observation"],
        values: dict[str, Any],
        metadata: FrameMetadata,
    ) -> int:
        layout, base_offset, active_offset, sequence_offset = self._direction(direction)
        current_active = struct.unpack_from("<I", self._buffer, active_offset)[0]
        target = 1 - current_active
        sequence = int(struct.unpack_from("<Q", self._buffer, sequence_offset)[0]) + 1
        slot_offset = base_offset + target * layout.slot_size

        self._validate_values(layout, values)
        _FRAME_META.pack_into(
            self._buffer,
            slot_offset,
            0,
            metadata.episode_id,
            metadata.physics_tick,
            metadata.control_tick,
            metadata.sim_time,
            metadata.applied_action_sequence,
        )
        for field in layout.fields:
            destination = np.ndarray(
                field.shape,
                dtype=np.dtype(field.dtype),
                buffer=self._buffer,
                offset=slot_offset + field.offset,
            )
            destination[...] = values[field.name]

        struct.pack_into("<Q", self._buffer, slot_offset, sequence)
        struct.pack_into("<Q", self._buffer, sequence_offset, sequence)
        struct.pack_into("<I", self._buffer, active_offset, target)
        return sequence

    def _read(
        self,
        direction: Literal["action", "observation"],
        *,
        retries: int,
    ) -> ChannelFrame | None:
        layout, base_offset, active_offset, sequence_offset = self._direction(direction)
        for _ in range(retries):
            sequence_before = struct.unpack_from("<Q", self._buffer, sequence_offset)[0]
            if sequence_before == 0:
                return None
            active_before = struct.unpack_from("<I", self._buffer, active_offset)[0]
            slot_offset = base_offset + active_before * layout.slot_size
            meta_raw = _FRAME_META.unpack_from(self._buffer, slot_offset)
            if meta_raw[0] != sequence_before:
                continue
            values = {
                field.name: np.ndarray(
                    field.shape,
                    dtype=np.dtype(field.dtype),
                    buffer=self._buffer,
                    offset=slot_offset + field.offset,
                ).copy()
                for field in layout.fields
            }
            sequence_after = struct.unpack_from("<Q", self._buffer, sequence_offset)[0]
            active_after = struct.unpack_from("<I", self._buffer, active_offset)[0]
            slot_sequence_after = struct.unpack_from("<Q", self._buffer, slot_offset)[0]
            if (
                sequence_before == sequence_after
                and active_before == active_after
                and slot_sequence_after == meta_raw[0]
            ):
                return ChannelFrame(
                    metadata=FrameMetadata(
                        sequence=meta_raw[0],
                        episode_id=meta_raw[1],
                        physics_tick=meta_raw[2],
                        control_tick=meta_raw[3],
                        sim_time=meta_raw[4],
                        applied_action_sequence=meta_raw[5],
                    ),
                    values=values,
                )
        raise RuntimeError(f"could not read coherent {direction} frame after {retries} retries")

    def _direction(
        self,
        direction: Literal["action", "observation"],
    ) -> tuple[FrameLayout, int, int, int]:
        if direction == "action":
            return (
                self.descriptor.action_layout,
                self.descriptor.action_offset,
                _ACTION_ACTIVE_OFFSET,
                _ACTION_SEQUENCE_OFFSET,
            )
        return (
            self.descriptor.observation_layout,
            self.descriptor.observation_offset,
            _OBSERVATION_ACTIVE_OFFSET,
            _OBSERVATION_SEQUENCE_OFFSET,
        )

    @staticmethod
    def _validate_values(layout: FrameLayout, values: dict[str, Any]) -> None:
        expected = {field.name for field in layout.fields}
        actual = set(values)
        if actual != expected:
            missing = sorted(expected - actual)
            extra = sorted(actual - expected)
            raise ValueError(f"frame fields mismatch: missing={missing}, extra={extra}")
        for field in layout.fields:
            actual_shape = np.asarray(values[field.name]).shape
            if actual_shape != field.shape:
                raise ValueError(
                    f"field '{field.name}' has shape {actual_shape}, expected {field.shape}"
                )

    def close(self) -> None:
        if self._closed:
            return
        self._shm.close()
        self._closed = True

    def unlink(self) -> None:
        """Remove the segment's name so no new process can attach; owner only."""
        if not self._owner:
            raise RuntimeError("only the channel owner may unlink shared memory")
        try:
            self._shm.unlink()
        except FileNotFoundError:
            pass

    def release(self) -> None:
        """Owner teardown: mark the channel closed, unlink the segment, unmap it.

        Every step runs even if an earlier one fails, so nothing stays behind in
        /dev/shm for the next launch to trip over. Failures are logged.
        """
        steps: tuple[Callable[[], None], ...] = (
            lambda: self.set_lifecycle("closed"),
            self.unlink,
            self.close,
        )
        for step in steps:
            try:
                step()
            except Exception:
                logger.exception(
                    "sim2 channel teardown step failed", shm_name=self.descriptor.shm_name
                )

    def __enter__(self) -> RobotChannel:
        return self

    def __exit__(self, *_: Any) -> None:
        self.close()


def _resource_tracker_pid() -> int:
    tracker = resource_tracker._resource_tracker  # type: ignore[attr-defined]
    tracker.ensure_running()
    return int(tracker._pid or 0)  # type: ignore[attr-defined]


def _untrack(shm: SharedMemory) -> None:
    """Stop this process's resource tracker from unlinking a segment it does not own."""
    try:
        resource_tracker.unregister(shm._name, "shared_memory")  # type: ignore[attr-defined]
    except Exception:
        pass


def _unlink_stale(descriptor: ChannelDescriptor) -> None:
    """Remove a leftover segment named like `descriptor`, or refuse if its creator is alive."""
    try:
        shm = SharedMemory(name=descriptor.shm_name, create=False)
    except FileNotFoundError:
        return
    try:
        owner = _owner_pid(shm)
        if owner and _pid_alive(owner):
            raise RuntimeError(
                f"sim2 shared memory '{descriptor.shm_name}' for "
                f"'{descriptor.sim_id}/{descriptor.robot_id}' belongs to running process "
                f"{owner}: a simulation with sim_id '{descriptor.sim_id}' is already up; "
                "stop it or launch with a different sim_id"
            )
        logger.warning(
            "replacing stale sim2 shared memory left by a dead process",
            shm_name=descriptor.shm_name,
            robot=f"{descriptor.sim_id}/{descriptor.robot_id}",
            owner_pid=owner,
        )
        shm.unlink()
    except BaseException:
        _untrack(shm)
        raise
    finally:
        shm.close()


def _owner_pid(shm: SharedMemory) -> int:
    """Creator pid recorded in a segment's header, or 0 if the header is not ours."""
    buffer = shm.buf
    if buffer is None or shm.size < CHANNEL_HEADER_SIZE:
        return 0
    magic, abi = struct.unpack_from("<8sI", buffer, 0)
    if magic != CHANNEL_MAGIC or abi != ABI_VERSION:
        return 0
    return int(struct.unpack_from("<Q", buffer, _OWNER_PID_OFFSET)[0])


def _pid_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True
