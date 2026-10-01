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

"""Shared-memory segment ownership across process lifetimes; no MuJoCo needed."""

import json
import logging
from multiprocessing.shared_memory import SharedMemory
import os
import subprocess
import sys
import textwrap
import uuid

import pytest

from dimos.sim2.ipc.abi import ChannelDescriptor, make_channel_descriptor
from dimos.sim2.ipc.channel import RobotChannel, _untrack
from dimos.sim2.spec import ControlInterface

_CHANNEL_LOGGER = "dimos/sim2/ipc/channel.py"

# Creates the channel, then dies without unlinking it, like a killed physics
# worker. A killed worker leaves no resource tracker to tidy up after it, so the
# interpreter's own tracker is told to forget the segment before exit.
_ABANDON = textwrap.dedent(
    """
    import json, os, sys
    from multiprocessing import resource_tracker
    from dimos.sim2.ipc.abi import ChannelDescriptor
    from dimos.sim2.ipc.channel import RobotChannel

    channel = RobotChannel.create(ChannelDescriptor.from_dict(json.loads(sys.argv[1])))
    channel.set_lifecycle("ready")
    resource_tracker.unregister(channel._shm._name, "shared_memory")
    os._exit(0)
    """
)


def _descriptor() -> ChannelDescriptor:
    tag = uuid.uuid4().hex[:12]
    return make_channel_descriptor(
        sim_id=f"test-{tag}",
        robot_id="arm",
        generation="test",
        shm_name=f"dms2_test_{tag}",
        control_interface=ControlInterface.MANIPULATOR,
        dof=2,
        physics_dt=0.005,
        control_decimation=1,
    )


def _segment_exists(descriptor: ChannelDescriptor) -> bool:
    try:
        shm = SharedMemory(name=descriptor.shm_name, create=False)
    except FileNotFoundError:
        return False
    _untrack(shm)
    shm.close()
    return True


@pytest.fixture
def channel_logs(caplog: pytest.LogCaptureFixture) -> pytest.LogCaptureFixture:
    lg = logging.getLogger(_CHANNEL_LOGGER)
    lg.addHandler(caplog.handler)
    caplog.set_level(logging.WARNING, logger=_CHANNEL_LOGGER)
    try:
        yield caplog
    finally:
        lg.removeHandler(caplog.handler)


def test_create_replaces_segment_whose_owner_died(
    channel_logs: pytest.LogCaptureFixture,
) -> None:
    descriptor = _descriptor()
    subprocess.run(
        [sys.executable, "-c", _ABANDON, json.dumps(descriptor.to_dict())],
        check=True,
        timeout=60,
    )
    assert _segment_exists(descriptor)

    channel = RobotChannel.create(descriptor)
    try:
        assert channel.owner_pid == os.getpid()
        assert channel.lifecycle == "starting"
        warnings = [r for r in channel_logs.records if "stale" in str(r.msg)]
        assert len(warnings) == 1
        assert descriptor.shm_name in str(warnings[0].msg)
    finally:
        channel.release()
    assert not _segment_exists(descriptor)


def test_create_refuses_segment_with_live_owner() -> None:
    descriptor = _descriptor()
    channel = RobotChannel.create(descriptor)
    try:
        channel.set_lifecycle("ready")
        with pytest.raises(RuntimeError, match=f"running process {os.getpid()}"):
            RobotChannel.create(descriptor)
        # The refusal must leave the live channel untouched.
        reader = RobotChannel.attach(descriptor)
        assert reader.lifecycle == "ready"
        assert reader.owner_pid == os.getpid()
        reader.close()
    finally:
        channel.release()
    assert not _segment_exists(descriptor)


def test_release_unlinks_even_when_an_earlier_step_fails() -> None:
    descriptor = _descriptor()
    channel = RobotChannel.create(descriptor)
    channel.close()  # the lifecycle write will now fail
    channel.release()
    assert not _segment_exists(descriptor)
