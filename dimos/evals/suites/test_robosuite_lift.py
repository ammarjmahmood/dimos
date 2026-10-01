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

import json
import time

import pytest

from dimos.evals.agents.lib.trajectory_builder import TrajectoryBuilder
from dimos.evals.suites.robosuite_lift import LiftEnvironment, upstream_success
from dimos.evals.types import Outcome
from dimos.memory.store.memory import MemoryStore
from dimos.porcelain.dimos import Dimos
from dimos.sim2.scene_types import SceneState


@pytest.fixture
def state():
    return SceneState(
        world_id="test",
        scene_id="robosuite:Lift",
        generation=1,
        tick=1,
        sim_time=0.1,
        ts=1.0,
        entities={},
        robots={},
        joints={},
        regions={},
        contacts=(),
        task_success=False,
    )


@pytest.fixture
def outcome(tmp_path):
    return Outcome(
        trajectory=TrajectoryBuilder("Lift", name="test", model="test").build("answer"),
        artifacts={"task_result": tmp_path / "lift-result.json"},
    )


@pytest.fixture
def environment():
    env = LiftEnvironment(blueprint=["xarm-robosuite-lift"], ready_streams=())
    try:
        yield env
    finally:
        env.stop()


@pytest.mark.parametrize("success", [False, True])
def test_capture_precedes_simulator_shutdown_and_grades_the_saved_oracle(
    mocker, environment, outcome, state, success
):
    state = state.model_copy(update={"task_success": success})
    app = mocker.patch.object(Dimos, "connect").return_value
    app.SimulationModule.scene_state.return_value = state
    path = outcome.artifacts["task_result"]
    shutdown = mocker.Mock(side_effect=lambda: upstream_success(outcome))
    environment._resources.callback(shutdown)
    # Reusing an attached recording must not preserve an earlier successful result.
    path.write_text(state.model_copy(update={"task_success": True}).model_dump_json())

    with MemoryStore() as store:
        artifacts = environment.prepare_recording(
            store, path.with_name("memory.db"), time.monotonic() + 1.0
        )
    assert artifacts == outcome.artifacts
    assert not path.exists()
    environment.stop()
    environment.stop()

    assert SceneState.model_validate_json(path.read_text()) == state
    assert upstream_success(outcome) == float(success)
    app.SimulationModule.scene_state.assert_called_once_with()
    app.stop.assert_called_once_with()
    shutdown.assert_called_once_with()


def test_failed_oracle_rpc_still_releases_the_simulator(mocker, environment, outcome):
    app = mocker.patch.object(Dimos, "connect").return_value
    app.SimulationModule.scene_state.side_effect = RuntimeError("simulator disconnected")
    shutdown = mocker.Mock()
    environment._resources.callback(shutdown)
    path = outcome.artifacts["task_result"]
    with MemoryStore() as store:
        environment.prepare_recording(store, path.with_name("memory.db"), time.monotonic() + 1.0)

    with pytest.raises(RuntimeError, match="simulator disconnected"):
        environment.stop()

    assert not path.exists()
    app.stop.assert_called_once_with()
    shutdown.assert_called_once_with()


def test_stop_before_recording_is_ready_does_not_query_a_nonexistent_simulator(mocker, environment):
    connect = mocker.patch.object(Dimos, "connect")
    environment.stop()
    connect.assert_not_called()


@pytest.mark.parametrize(
    "update", [{"task_success": None}, {"scene_id": "authored-table"}, {"task_success": "true"}]
)
def test_invalid_oracle_is_an_eval_error_not_a_task_failure(outcome, state, update):
    outcome.artifacts["task_result"].write_text(json.dumps(state.model_dump(mode="json") | update))
    with pytest.raises(ValueError):
        upstream_success(outcome)


def test_missing_result_is_an_eval_error_not_a_task_failure(outcome):
    with pytest.raises(FileNotFoundError):
        upstream_success(outcome)
