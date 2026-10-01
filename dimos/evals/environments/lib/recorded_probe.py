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

"""The ctx a case's MuJoCo probe wrote while the simulation ran."""

from __future__ import annotations

from typing import TYPE_CHECKING, TypeVar

from pydantic import TypeAdapter

from dimos.simulation.engines.mujoco_probe import ProbeReport

if TYPE_CHECKING:
    from dimos.memory.store.base import Store

T = TypeVar("T")


def probe_ctx(recording: Store, ctx_type: type[T]) -> T:
    """The probe's last recorded ctx; raises if a handler failed during the run."""
    if "sim_probe" not in recording.streams:
        raise LookupError("No sim_probe recorded; does the environment have a probe?")
    report = TypeAdapter(ProbeReport).validate_json(recording.streams.sim_probe.last().data.data)
    if report.error:
        raise RuntimeError(f"MuJoCo probe handler failed: {report.error}")
    return TypeAdapter(ctx_type).validate_python(report.ctx)
