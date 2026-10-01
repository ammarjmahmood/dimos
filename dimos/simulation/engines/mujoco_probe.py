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

"""Per-object contact handlers and per-step handlers over a running MuJoCo model."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import importlib
from types import FunctionType
from typing import TYPE_CHECKING, cast

from pydantic import JsonValue, TypeAdapter

from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    import mujoco

logger = setup_logger()


@dataclass(frozen=True)
class Contact:
    """One object starting or ceasing to touch another."""

    name: str
    """The object the handler was registered on."""
    other: str
    """The object it touches, or no longer touches."""
    time: float
    """Simulation time of the step that saw the change, in seconds."""


ContactHandler = Callable[[Contact], None]
TickHandler = Callable[["mujoco.MjModel", "mujoco.MjData"], None]


class MujocoProbe:
    """Handlers registered on a MuJoCo model, called after every physics step.

    An object is a body, named by its body name. A geom fixed to the world body is an
    object of its own, named by its geom name, or ``world`` if it has none.
    """

    release_s: float = 0.1
    """Simulated seconds two objects must stay apart before their contact ends, so a
    bouncing or chattering contact is one begin and one end."""

    def __init__(self, model: mujoco.MjModel) -> None:
        self._model = model
        self._geom_objects = [self._object_of(model, geom) for geom in range(model.ngeom)]
        self._begin: dict[str, list[ContactHandler]] = {}
        self._end: dict[str, list[ContactHandler]] = {}
        self._tick: list[TickHandler] = []
        self._steps = 0
        self._last_touch: dict[tuple[str, str], int] = {}

    @staticmethod
    def _object_of(model: mujoco.MjModel, geom: int) -> str:
        body = int(model.geom_bodyid[geom])
        if body:
            return str(model.body(body).name)
        return str(model.geom(geom).name) or "world"

    def on_contact_begin(self, name: str, handler: ContactHandler) -> None:
        """Call ``handler`` on the step ``name`` starts touching another object."""
        self._begin.setdefault(self._known(name), []).append(handler)

    def on_contact_end(self, name: str, handler: ContactHandler) -> None:
        """Call ``handler`` once ``name`` has been apart from another object for ``release_s``."""
        self._end.setdefault(self._known(name), []).append(handler)

    def on_tick(self, handler: TickHandler) -> None:
        """Call ``handler`` with the model and its state after every physics step."""
        self._tick.append(handler)

    def _known(self, name: str) -> str:
        if name not in self._geom_objects:
            known = ", ".join(sorted(set(self._geom_objects)))
            raise ValueError(f"no object with collision geometry named {name!r}; have {known}")
        return name

    def step(self, data: mujoco.MjData) -> None:
        """Dispatch the contact changes since the previous step, then the tick handlers."""
        self._steps += 1
        if self._begin or self._end:
            touching = self._pairs(data)
            began = sorted(touching - self._last_touch.keys())
            self._last_touch.update(dict.fromkeys(touching, self._steps))
            release_steps = self.release_s / self._model.opt.timestep
            ended = sorted(
                pair
                for pair, step in self._last_touch.items()
                if self._steps - step > release_steps
            )
            for pair in ended:
                del self._last_touch[pair]
            for handlers, pairs in ((self._begin, began), (self._end, ended)):
                for first, second in pairs:
                    for name, other in ((first, second), (second, first)):
                        for handler in handlers.get(name, ()):
                            handler(Contact(name=name, other=other, time=float(data.time)))
        for tick in self._tick:
            tick(self._model, data)

    def _pairs(self, data: mujoco.MjData) -> set[tuple[str, str]]:
        pairs = set()
        for geom1, geom2 in data.contact.geom.tolist():
            if geom1 < 0 or geom2 < 0:
                continue  # flex contacts have no geom
            first, second = sorted((self._geom_objects[geom1], self._geom_objects[geom2]))
            if first != second:
                pairs.add((first, second))
        return pairs


ProbeSetup = Callable[[MujocoProbe], object]
"""Registers a case's handlers and returns the ctx object they write to."""


def setup_path(setup: ProbeSetup) -> str:
    """The ``module:name`` a simulator process imports ``setup`` by."""
    if not isinstance(setup, FunctionType):
        raise ValueError(f"a probe setup must be a module-level function, got {setup!r}")
    path = f"{setup.__module__}:{setup.__qualname__}"
    try:
        found = load_setup(path)
    except (ImportError, AttributeError) as e:
        raise ValueError(f"probe setup {path} is not importable: {e}") from e
    if found is not setup:
        raise ValueError(f"probe setup {path} imports as a different object")
    return path


def load_setup(path: str) -> ProbeSetup:
    """The setup function at ``module:name``."""
    module, _, name = path.partition(":")
    return cast("ProbeSetup", getattr(importlib.import_module(module), name))


@dataclass(frozen=True)
class ProbeReport:
    """What a probed simulation publishes for the grader."""

    ctx: JsonValue
    """The object the setup function returned."""
    error: str = ""
    """The handler exception that stopped the probe; empty while it runs."""


class ProbeRun:
    """A case's probe attached to one simulation."""

    def __init__(self, setup: ProbeSetup, model: mujoco.MjModel) -> None:
        self._probe = MujocoProbe(model)
        self._ctx = setup(self._probe)
        self._ctx_json: TypeAdapter[object] = TypeAdapter(type(self._ctx))
        self._error = ""

    def step(self, data: mujoco.MjData) -> None:
        """Run the handlers for one physics step; the first exception stops the probe."""
        if self._error:
            return
        try:
            self._probe.step(data)
        except Exception as e:
            self._error = repr(e)
            logger.exception("MuJoCo probe handler failed; the probe is stopped")

    def report(self) -> str:
        """The ctx and any handler error, as the JSON of a ``ProbeReport``."""
        report = ProbeReport(
            ctx=self._ctx_json.dump_python(self._ctx, mode="json"), error=self._error
        )
        return TypeAdapter(ProbeReport).dump_json(report).decode()
