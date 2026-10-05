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
# ruff: noqa: N815  (field names are the wire format's, camelCase as Desktop's)

"""The /dimos API's request bodies, answers and events, as pydantic models: what openapi.json is generated from.

The server builds its answers as plain dicts; FastAPI validates each against the route's model. Models allow extra
fields (an answer never loses one), and the tests check no answer carries one the model doesn't declare.
"""

from __future__ import annotations

from typing import Annotated, Literal

from pydantic import BaseModel, ConfigDict, Field, JsonValue


class ApiModel(BaseModel):
    model_config = ConfigDict(extra="allow", populate_by_name=True)


class ErrorResponse(ApiModel):
    """Every error answer (4xx and 5xx)."""

    error: str = Field(
        description="What went wrong, in words a person can act on",
        examples=["no upload u7"],
    )


# server


class Info(ApiModel):
    """The dimos checkout this server launches runs from."""

    dir: str = Field(description="The checkout's folder", examples=["/home/me/dimos"])
    found: bool = Field(description="A dimos checkout (pyproject.toml naming dimos) is there")
    installed: bool = Field(description="Its `.venv/bin/dimos` exists, so it can run blueprints")
    version: str | None = Field(
        description="Its pyproject version (null: none found)", examples=["0.0.14"]
    )
    range: str = Field(
        description="The dimos versions Desktop supports ($DESKTOP_DIMOS_RANGE); empty = any",
        examples=[">=0.0.14b1 <0.1"],
    )
    inRange: bool = Field(description="`version` is inside `range` (a launch is refused when not)")


class Paths(ApiModel):
    """Where dimos keeps its things."""

    dimosDir: str = Field(description="The checkout", examples=["/home/me/dimos"])
    runsDir: str = Field(
        description="dimos's run registry (one <run_id>.json per live run)",
        examples=["/home/me/.local/state/dimos/runs"],
    )
    logsDirs: list[str] = Field(
        description="Where runs' logs are, searched in order: the checkout's logs/, then the library install's",
        examples=[["/home/me/dimos/logs", "/home/me/.local/state/dimos/logs"]],
    )
    recordingsDir: str = Field(
        description="Desktop's recordings folder (config.yaml `recordings.dir`)",
        examples=["/home/me/.dimos/recordings"],
    )


class Stopping(ApiModel):
    stopping: Literal[True] = Field(description="Always true: the server exits in 0.2 s")


# blueprints


class BlueprintName(ApiModel):
    name: str = Field(description="The name `dimos run` takes", examples=["unitree-go2-basic"])
    kind: Literal["builtin", "external"] = Field(
        description="builtin: in dimos itself; external: from an installed package's entry points"
    )


class BlueprintList(ApiModel):
    blueprints: list[BlueprintName] = Field(
        description="Built-in blueprints (sorted, without demo-*), then external ones"
    )


class Stream(ApiModel):
    name: str = Field(description="The stream's name on its module", examples=["color_image"])
    type: str = Field(description="Its message type", examples=["dimos.msgs.sensor_msgs.Image"])
    direction: Literal["in", "out", "inout"] = Field(
        description="in: the module reads it; out: it publishes it; inout: both"
    )


class BlueprintModule(ApiModel):
    name: str = Field(description="The module's name in the blueprint", examples=["camera"])
    class_: str = Field(
        alias="class",
        description="Its Python class",
        examples=["dimos.hardware.camera.CameraModule"],
    )
    streams: list[Stream] = Field(description="Its streams")


class Blueprint(ApiModel):
    name: str = Field(description="The blueprint", examples=["unitree-go2-basic"])
    modules: list[BlueprintModule] = Field(description="Its modules, in blueprint order")


class ConfigArg(ApiModel):
    """One field of a module's pydantic `config` model."""

    name: str = Field(description="The field", examples=["robot_ip"])
    type: str | None = Field(description="Its annotation", examples=["str | None"])
    default: JsonValue = Field(description="Its default (null when required)", examples=[None])
    description: str | None = Field(description="The field's description, if it has one")
    required: bool = Field(description="It has no default")
    base: bool = Field(description="Inherited from dimos's ModuleConfig (every module has it)")
    choices: list[JsonValue] | None = Field(
        default=None,
        description="Only for an Enum or Literal: the values it takes",
        examples=[["webrtc", "ros"]],
    )
    value: JsonValue = Field(
        default=None, description="Only when the blueprint sets it: the value it sets"
    )


class ModuleConfig(ApiModel):
    module: str = Field(description="The module's name in the blueprint", examples=["camera"])
    class_: str = Field(alias="class", description="Its Python class")
    args: list[ConfigArg] = Field(description="Its configurable args (empty when `error`)")
    error: str | None = Field(
        default=None,
        description="Only when its config couldn't be read: why",
        examples=["TypeError: CameraModule.config (dict) isn't a pydantic model"],
    )


class BlueprintConfig(ApiModel):
    name: str = Field(description="The blueprint", examples=["unitree-go2"])
    modules: list[ModuleConfig] = Field(description="Each module's configurable args")


class Port(ApiModel):
    name: str = Field(description="The stream's name", examples=["odom"])
    type: str = Field(description="Its message type's name", examples=["Odometry"])


class SkillParam(ApiModel):
    name: str = Field(description="The parameter", examples=["distance"])
    type: str | None = Field(description="Its annotation, if any", examples=["float"])
    default: str | None = Field(description="repr() of its default (null: none)", examples=["1.0"])


class CatalogBlueprint(ApiModel):
    name: str = Field(description="The blueprint", examples=["unitree-go2"])
    ref: str = Field(
        description="Where it's defined, `module:attribute`",
        examples=["dimos.robot.unitree.go2.blueprints.basic:unitree_go2"],
    )
    robot: str | None = Field(
        description="The robot folder it's under (null: none)", examples=["go2"]
    )
    modules: list[str] = Field(description="Its modules' catalog names")


class CatalogModule(ApiModel):
    name: str = Field(description="The module's catalog name", examples=["CameraModule"])
    class_: str = Field(alias="class", description="Its Python class, `module.QualName`")
    doc: str = Field(description="Its docstring's first paragraph (at most 300 characters)")
    robots: list[str] = Field(description="Robots whose blueprints use it")
    inputs: list[Port] = Field(description="Streams it reads")
    outputs: list[Port] = Field(description="Streams it publishes")
    skills: list[str] = Field(description="Its skills' names")


class CatalogSkill(ApiModel):
    name: str = Field(description="The skill", examples=["move"])
    doc: str = Field(description="Its docstring's first paragraph")
    params: list[SkillParam] = Field(description="Its parameters")
    module: str = Field(description="The module it's on")
    robots: list[str] = Field(description="Robots whose blueprints have that module")


class Catalog(ApiModel):
    blueprints: list[CatalogBlueprint] = Field(description="Every built-in blueprint")
    modules: list[CatalogModule] = Field(description="Every module")
    skills: list[CatalogSkill] = Field(description="Every skill, on every module")
    errors: list[str] = Field(
        description="What couldn't be imported (at most 50)",
        examples=[["blueprint g1-sim: ImportError: no mujoco"]],
    )


# global config


class GlobalConfig(ApiModel):
    json_schema: dict[str, JsonValue] = Field(
        alias="schema", description="dimos's GlobalConfig as a JSON Schema (draft 2020-12)"
    )
    defaults: dict[str, JsonValue] = Field(
        description="Each field's default (fields whose default is computed are left out)",
        examples=[{"n_workers": 2, "simulation": False}],
    )
    overrides: dict[str, JsonValue] = Field(
        description="Desktop's overrides (config.yaml `dimos.global_config`): `--key value` on every launch",
        examples=[{"robot_ip": "192.168.12.1"}],
    )


class GlobalConfigUpdate(ApiModel):
    overrides: dict[str, JsonValue] = Field(
        description="The new overrides, {key: value}, replacing the old ones; a null value removes that key. "
        "Keys are letters, digits and `_`",
        examples=[{"robot_ip": "192.168.12.1", "simulation": None}],
    )


# runs


class RegistryRun(ApiModel):
    """A live run in dimos's run registry (also runs started from a terminal)."""

    run_id: str = Field(description="The run's id", examples=["20260101-120000-unitree-go2"])
    pid: int = Field(description="Its process id", examples=[41233])
    blueprint: str = Field(description="What it runs", examples=["unitree-go2"])
    started_at: str = Field(description="When it started (ISO 8601)")
    log_dir: str = Field(description="Its log folder (main.jsonl is there)")


class Launch(ApiModel):
    """The last launch this server started; its phase is worked out from disk on every call."""

    blueprint: str = Field(description="What was launched", examples=["unitree-go2"])
    phase: Literal["starting", "running", "stopped", "failed"] = Field(
        description="starting: alive, not registered yet; running: in dimos's run registry (every module built); "
        "stopped: ran and is gone; failed: exited before running"
    )
    startedAt: str = Field(
        description="When it was launched (ISO 8601, UTC)", examples=["2026-01-01T12:00:00Z"]
    )
    pid: int = Field(
        description="The `dimos run` process (its own process group)", examples=[41233]
    )
    output: str = Field(
        description="The end of its output (at most 200 kB), starting with the command line"
    )
    runId: str | None = Field(description="Its registry run id, once running")
    logDir: str | None = Field(description="Its log folder, once running")
    error: str | None = Field(
        description="When failed: the output's first `Error: ` line, else its last line",
        examples=["Error: no blueprint named unitree-go3"],
    )


class RunList(ApiModel):
    runs: list[RegistryRun] = Field(description="Live runs, newest first")
    launch: Launch | None = Field(description="The launch this server started (null: none yet)")


class LaunchRequest(ApiModel):
    blueprint: str = Field(description="blueprint name", examples=["unitree-go2"])
    replay: bool = Field(default=False, description="Run on a recording: adds `--replay`")
    overrides: dict[str, JsonValue] = Field(
        default_factory=dict,
        description='GlobalConfig overrides for this launch, e.g. {"simulation": "mujoco"}; they win over '
        "Desktop's saved ones",
        examples=[{"simulation": "mujoco"}, {"robot_ip": "192.168.12.1"}],
    )


class StopRequest(ApiModel):
    runId: str | None = Field(
        default=None,
        description="run id (any live run in the registry); none: the launch this server started",
        examples=["20260101-120000-unitree-go2"],
    )


class StopResult(ApiModel):
    output: str = Field(
        description="What was stopped", examples=["stopped unitree-go2 (pid 41233)"]
    )


# logs


class LogRecord(ApiModel):
    """One line of a run's main.jsonl (structlog JSON); a line that isn't JSON is a `raw` record."""

    timestamp: str = Field(description="As logged", examples=["2026-01-01T12:00:01.123Z"])
    level: str = Field(
        description="debug, info, warning, error or critical (lowercased); raw for a non-JSON line",
        examples=["warning"],
    )
    logger: str = Field(description="The logger's name", examples=["dimos.navigation"])
    event: str = Field(description="The message", examples=["no path to goal"])
    extra: dict[str, JsonValue] = Field(description="Every other field of the record")
    raw: str = Field(description="The line as written")


class LogPage(ApiModel):
    runId: str | None = Field(
        description="The run read (`latest` resolved)", examples=["20260101-120000-unitree-go2"]
    )
    records: list[LogRecord] = Field(description="Matching records, oldest first")
    offset: int = Field(description="Byte offset to pass back as `after` for only newer records")
    loggers: list[str] = Field(description="Every logger name seen, sorted (for a filter menu)")


# cloud


class Account(ApiModel):
    loggedIn: bool = Field(
        description="A cloud key is stored (or in the environment) and wasn't refused"
    )
    email: str | None = Field(description="Who it belongs to", examples=["me@example.com"])
    scopes: list[str] | None = Field(description="What it may do, per the cloud")
    source: Literal["env", "stored"] | None = Field(
        description="env: DIMOS_API_KEY; stored: `dimos login`'s saved key; null: none"
    )
    cloudUrl: str = Field(
        description="The Dimensional cloud asked", examples=["https://api.dimensional.org"]
    )
    error: str | None = Field(
        description="Why it couldn't be checked, or that the key was revoked",
        examples=["The saved login was revoked or is invalid: log in again."],
    )


class Login(ApiModel):
    """The device login: a URL and a code the user approves in any browser signed in to Dimensional cloud."""

    state: Literal["idle", "starting", "pending", "approved", "denied", "expired", "failed"] = (
        Field(
            description="idle: none; starting: asking for a code; pending: waiting for approval; then approved, denied, "
            "expired or failed"
        )
    )
    url: str | None = Field(
        description="Where to approve it", examples=["https://console.dimensional.org/device"]
    )
    urlComplete: str | None = Field(description="The same URL with the code filled in")
    code: str | None = Field(description="The code to show", examples=["ABCD-EFGH"])
    expiresAt: int | None = Field(description="When the code expires (Unix ms)")
    email: str | None = Field(description="Who approved it")
    error: str | None = Field(description="Why it failed")


# uploads


class Upload(ApiModel):
    """One upload in the queue."""

    id: str = Field(description="The upload's id", examples=["u3"])
    path: str = Field(
        description="The recording", examples=["/home/me/.dimos/recordings/walk.mcap"]
    )
    name: str = Field(description="Its file name", examples=["walk.mcap"])
    size: int = Field(description="Its size in bytes when queued")
    robotId: str | None = Field(description="The robot id it's tagged with", examples=["go2-lab"])
    kind: str | None = Field(
        description="recording, video, pointcloud, log or blob (null: from the file)"
    )
    state: Literal["queued", "uploading", "done", "failed", "cancelled"] = Field(
        description="queued (first in, first out), uploading (one at a time), then done, failed or cancelled"
    )
    phase: str | None = Field(
        description="While uploading: preparing, compress, hash, upload, then finishing",
        examples=["upload"],
    )
    bytesDone: int = Field(description="Bytes done in this phase")
    bytesTotal: int = Field(description="Bytes in this phase (0: indeterminate)")
    rateBps: float | None = Field(description="Smoothed speed in bytes a second")
    etaSeconds: float | None = Field(
        description="Time left in this phase (null for the first second)"
    )
    uploadId: str | None = Field(description="The cloud's id for it, once done")
    skipped: bool = Field(description="The cloud had it already")
    notice: str | None = Field(description="Something to tell the user, e.g. about the quota")
    error: str | None = Field(description="Why it failed, or that it waits for a login")
    errorCode: str | None = Field(
        description="not_logged_in, network, quota, file_missing or failed",
        examples=["not_logged_in"],
    )
    log: str | None = Field(description="The log with the details of a failure")
    createdAt: int = Field(description="When it was queued (Unix ms)")
    startedAt: int | None = Field(description="When its upload started (Unix ms)")
    finishedAt: int | None = Field(description="When it finished (Unix ms)")
    link: str | None = Field(description="Its page in the Dimensional console, once done")


class UploadList(ApiModel):
    uploads: list[Upload] = Field(description="The queue, in order")
    waitingForLogin: bool = Field(
        description="An upload found no cloud login: the queue waits for one"
    )


class UploadRequest(ApiModel):
    path: str = Field(
        description="the recording's absolute path (a /recordings entry's path): an .mcap or a .db",
        examples=["/home/me/.dimos/recordings/walk.mcap"],
    )
    robotId: str | None = Field(
        default=None, description="robot id to tag it with", examples=["go2-lab"]
    )
    kind: str | None = Field(
        default=None,
        description="recording (default for .mcap/.db), video, pointcloud, log, blob",
        examples=["recording"],
    )


class Uploaded(ApiModel):
    """A recording that is in the cloud (remembered even after the upload list is cleared)."""

    path: str = Field(
        description="The recording", examples=["/home/me/.dimos/recordings/walk.mcap"]
    )
    uploadId: str = Field(description="The cloud's id for it")
    size: int = Field(description="Its size when uploaded")
    mtimeMs: int = Field(description="Its modification time when uploaded (Unix ms)")
    uploadedAt: int = Field(description="When the upload finished (Unix ms)")
    link: str | None = Field(description="Its page in the Dimensional console")
    changed: bool = Field(
        description="The file's size or modification time differs from the uploaded one"
    )


class UploadedByPath(ApiModel):
    byPath: dict[str, Uploaded] = Field(description="Every uploaded recording, by path")


class Ok(ApiModel):
    ok: Literal[True] = Field(description="Always true")


# events: on zenoh at <ns>/dimos/events/<type>, and (deprecated) the SSE stream /dimos/events


def _zenoh(kind: str, text: str) -> ConfigDict:
    return ConfigDict(
        json_schema_extra={"description": text, "x-zenoh-key": f"<ns>/dimos/events/{kind}"},
    )


class LaunchEvent(ApiModel):
    model_config = _zenoh(
        "launch", "The launch's (blueprint, phase, runId) changed; also first on every SSE connect"
    )
    type: Literal["launch"]
    launch: Launch | None = Field(description="The launch now (null: none)")


class LogEvent(ApiModel):
    model_config = _zenoh("log", "A new warning-or-worse record in the current launch's main.jsonl")
    type: Literal["log"]
    runId: str | None = Field(description="The launch's run id")
    record: LogRecord = Field(description="The record")


class UploadEvent(ApiModel):
    model_config = _zenoh("upload", "An upload changed (progress: a few times a second at most)")
    type: Literal["upload"]
    upload: Upload = Field(description="The upload now")


class UploadsEvent(ApiModel):
    model_config = _zenoh("uploads", "The queue as a whole changed: GET /dimos/uploads for it")
    type: Literal["uploads"]
    waitingForLogin: bool = Field(description="The queue waits for a cloud login")
    cleared: bool | None = Field(default=None, description="Only after DELETE /dimos/uploads: true")


class UploadRemovedEvent(ApiModel):
    model_config = _zenoh("upload-removed", "A finished upload was removed from the list")
    type: Literal["upload-removed"]
    id: str = Field(description="Its id", examples=["u3"])


class CloudLoginEvent(ApiModel):
    model_config = _zenoh("cloud-login", "The device login's state changed")
    type: Literal["cloud-login"]
    login: Login = Field(description="The login now")


EVENT_MODELS: tuple[type[ApiModel], ...] = (
    LaunchEvent,
    LogEvent,
    UploadEvent,
    UploadsEvent,
    UploadRemovedEvent,
    CloudLoginEvent,
)

DimosEvent = Annotated[
    LaunchEvent | LogEvent | UploadEvent | UploadsEvent | UploadRemovedEvent | CloudLoginEvent,
    Field(discriminator="type"),
]
