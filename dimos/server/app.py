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

"""The dimos server's HTTP API, `/dimos/...`: blueprints, global config, runs and their logs, events, cloud uploads.

dimOS Desktop proxies `/dimos/` to it and documents it (its OpenAPI, `x-family: dimos`); fixtures/ holds a copy of
those paths, and test_contract.py checks every one is served here. An error is `{"error": "<message>"}`.
"""

from __future__ import annotations

import asyncio
from collections.abc import AsyncIterator
from contextlib import asynccontextmanager
from dataclasses import dataclass, field
import os
from pathlib import Path
import re
from typing import Any

from fastapi import Body, FastAPI, Request
from fastapi.exceptions import RequestValidationError
from fastapi.responses import HTMLResponse, JSONResponse, PlainTextResponse, StreamingResponse
from starlette.exceptions import HTTPException as StarletteHTTPException

from dimos.server import blueprints, config, events, logs, runs
from dimos.server.uploads import Uploads

LIST_TTL_S = 60.0
INTROSPECT_TTL_S = 600.0


class ApiError(Exception):
    def __init__(self, status: int, message: str) -> None:
        super().__init__(message)
        self.status = status


@dataclass
class ServerState:
    dimos_dir: Path
    bus: events.Bus
    uploads: Uploads
    cache: blueprints.Cache = field(default_factory=blueprints.Cache)
    # what the server runs in the background (the launch watcher, the upload queue)
    background: list[asyncio.Task[None]] = field(default_factory=list)
    # set by serve(): makes the process exit
    exit: Any = None


def default_state(dimos_dir: Path) -> ServerState:
    bus = events.Bus()
    # uploaded.json is read from beside uploads.json
    config.state_file("uploaded.json")
    uploads = Uploads(
        dimos_dir, bus, config.state_file("uploads.json"), config.logs_dir() / "uploads.log"
    )
    return ServerState(dimos_dir=dimos_dir, bus=bus, uploads=uploads)


def fresh_flag(value: str | None) -> bool:
    return value in ("1", "true", "yes", "")


def create_app(state: ServerState, background: bool = True) -> FastAPI:
    @asynccontextmanager
    async def lifespan(_: FastAPI) -> AsyncIterator[None]:
        if background:
            state.background += [
                asyncio.create_task(events.watch_launch(state.bus)),
                asyncio.create_task(state.uploads.work()),
            ]
        yield
        state.uploads.shutdown()
        for task in state.background:
            task.cancel()

    app = FastAPI(
        title="dimos server",
        lifespan=lifespan,
        openapi_url="/dimos/openapi.json",
        docs_url=None,
        redoc_url=None,
    )
    app.state.server = state
    s = state

    @app.exception_handler(ApiError)
    async def api_error(_: Request, error: ApiError) -> JSONResponse:
        return JSONResponse({"error": str(error)}, status_code=error.status)

    @app.exception_handler(StarletteHTTPException)
    async def http_error(request: Request, error: StarletteHTTPException) -> JSONResponse:
        message = (
            f"no such route: {request.url.path}" if error.status_code == 404 else str(error.detail)
        )
        return JSONResponse({"error": message}, status_code=error.status_code)

    @app.exception_handler(RequestValidationError)
    async def bad_request(_: Request, error: RequestValidationError) -> JSONResponse:
        problems = "; ".join(f"{'.'.join(map(str, e['loc']))}: {e['msg']}" for e in error.errors())
        return JSONResponse({"error": f"bad request: {problems}"}, status_code=400)

    @app.exception_handler(Exception)
    async def failed(_: Request, error: Exception) -> JSONResponse:
        return JSONResponse({"error": str(error) or type(error).__name__}, status_code=500)

    async def introspected(key: str, args: list[str]) -> Any:
        try:
            return await s.cache.get(
                key, INTROSPECT_TTL_S, lambda: blueprints.introspect(s.dimos_dir, args)
            )
        except blueprints.IntrospectError as error:
            raise ApiError(500, str(error))

    def check_name(name: str) -> None:
        if not blueprints.valid_name(name):
            raise ApiError(400, f"bad blueprint name: {name}")

    @app.get("/healthz", response_class=PlainTextResponse, include_in_schema=False)
    @app.get("/dimos/healthz", response_class=PlainTextResponse)
    async def healthz() -> str:
        """The dimos server's liveness: `ok`"""
        return "ok"

    @app.get("/dimos/info")
    async def info() -> dict[str, Any]:
        """The dimos checkout: dir, version, installed"""
        return config.info(s.dimos_dir).to_json()

    @app.get("/dimos/paths")
    async def paths() -> dict[str, Any]:
        """Where dimos keeps its things: the checkout, run registry, log folders, recordings folder"""
        from dimos.constants import STATE_DIR

        return {
            "dimosDir": str(s.dimos_dir),
            "runsDir": str(STATE_DIR / "runs"),
            "logsDirs": [str(d) for d in logs.logs_dirs(s.dimos_dir)],
            "recordingsDir": str(config.recordings_dir()),
        }

    @app.get("/dimos/blueprints")
    async def blueprint_list(fresh: str | None = None) -> dict[str, Any]:
        """Every blueprint dimos can run (name, builtin/external); `?fresh=1` skips the 60 s cache"""
        if fresh_flag(fresh):
            s.cache.forget("list")

        async def compute() -> dict[str, Any]:
            return {"blueprints": await asyncio.to_thread(blueprints.blueprint_list)}

        result: dict[str, Any] = await s.cache.get("list", LIST_TTL_S, compute)
        return result

    @app.get("/dimos/blueprints/{name}")
    async def blueprint(name: str) -> Any:
        """A blueprint's modules and each module's streams (topics, types, in/out)"""
        check_name(name)
        return await introspected(f"bp:{name}", ["blueprint", name])

    @app.get("/dimos/blueprints/{name}/config")
    async def blueprint_config(name: str) -> Any:
        """A blueprint's configurable args per module (a module that can't be read carries its own error)"""
        check_name(name)
        return await introspected(f"config:{name}", ["config", name])

    @app.get("/dimos/catalog")
    async def catalog() -> Any:
        """Every blueprint, module and skill (imports them all, in a child process: slow the first time)"""
        return await introspected("catalog", ["catalog"])

    async def global_config_value() -> dict[str, Any]:
        async def compute() -> dict[str, Any]:
            return await asyncio.to_thread(blueprints.global_config_schema)

        value: dict[str, Any] = await s.cache.get("gc", INTROSPECT_TTL_S, compute)
        return {**value, "overrides": config.global_config_overrides()}

    @app.get("/dimos/global-config")
    async def global_config() -> dict[str, Any]:
        """dimos GlobalConfig: JSON schema, defaults, Desktop's overrides"""
        return await global_config_value()

    @app.put("/dimos/global-config")
    async def put_global_config(
        overrides: dict[str, Any] = Body(..., embed=True),
    ) -> dict[str, Any]:
        """Save the GlobalConfig overrides (null removes one); they become `--key value` on every launch"""
        for key in overrides:
            if not re.fullmatch(r"[A-Za-z0-9_]+", key):
                raise ApiError(400, f"bad config key: {key}")
        config.set_global_config_overrides(overrides)
        return await global_config_value()

    @app.get("/dimos/runs")
    async def run_list() -> dict[str, Any]:
        """Running blueprints (run id, blueprint, pid, log_dir) and the launch this server started"""
        return {
            "runs": await asyncio.to_thread(runs.registry_runs),
            "launch": await asyncio.to_thread(runs.current_launch),
        }

    @app.post("/dimos/runs")
    async def launch(
        blueprint: str = Body(..., embed=True),
        replay: bool = Body(False, embed=True),
        overrides: dict[str, Any] = Body(default_factory=dict, embed=True),
    ) -> dict[str, Any]:
        """Launch a blueprint (stops nothing; check /dimos/runs first)"""
        checkout = config.info(s.dimos_dir)
        if checkout.installed and not checkout.in_range and not config.ignore_version_range():
            raise ApiError(
                400,
                f"dimos {checkout.version or '?'} is outside the range Desktop supports ({checkout.range}); "
                "set dimos.ignore_version_range to launch anyway",
            )
        if not blueprint or blueprint.startswith("-"):
            raise ApiError(400, "bad blueprint name")
        merged = {**config.global_config_overrides(), **overrides}
        if replay:
            merged["replay"] = True
        try:
            started = runs.start(s.dimos_dir, blueprint, config.global_config_flags(merged))
        except runs.RunError as error:
            raise ApiError(500, str(error))
        s.bus.send({"type": "launch", "launch": started})
        return started

    @app.post("/dimos/runs/stop")
    async def stop(request: Request) -> dict[str, Any]:
        """Stop the blueprint this server launched (or `runId`)"""
        body = await request.json() if await request.body() else {}
        run_id = body.get("runId") if isinstance(body, dict) else None
        try:
            return {"output": await runs.stop(run_id)}
        except runs.RunError as error:
            raise ApiError(500, str(error))

    @app.get("/dimos/runs/{run_id}/log")
    async def log(
        run_id: str,
        after: int | None = None,
        level: str | None = None,
        q: str | None = None,
        limit: int | None = None,
    ) -> dict[str, Any]:
        """A run's structured log (main.jsonl): records with level, logger, event; `after` = byte offset to tail from"""
        filter = logs.Filter(query=q or None, min_level=level or None)
        return await asyncio.to_thread(logs.read, s.dimos_dir, run_id, after, limit or 1000, filter)

    @app.get("/dimos/events")
    async def event_stream() -> StreamingResponse:
        """Live events (SSE): launch phases, warning+ log records, uploads, the cloud login"""
        stream = s.bus.stream(lambda: {"type": "launch", "launch": runs.current_launch()})
        return StreamingResponse(
            stream, media_type="text/event-stream", headers={"Cache-Control": "no-cache"}
        )

    @app.post("/dimos/server/stop")
    async def stop_server() -> dict[str, Any]:
        """Make the dimos server exit (Desktop starts it again when needed)"""
        s.uploads.shutdown()
        loop = asyncio.get_running_loop()
        loop.call_later(0.2, s.exit or (lambda: os._exit(0)))
        return {"stopping": True}

    @app.get("/dimos/cloud/account")
    async def cloud_account(fresh: str | None = None) -> dict[str, Any]:
        """Whether this machine is logged in to Dimensional cloud, and as whom (`?fresh=1`: not the 20 s cache)"""
        return await s.uploads.account(fresh_flag(fresh))

    @app.get("/dimos/cloud/login")
    async def cloud_login() -> dict[str, Any]:
        """The cloud login in progress: state (idle, starting, pending, approved, denied, expired, failed), url, code"""
        return s.uploads.login_state()

    @app.post("/dimos/cloud/login")
    async def start_cloud_login() -> dict[str, Any]:
        """Start the device login: a URL and a code the user approves in any signed-in browser"""
        return await s.uploads.start_login()

    @app.delete("/dimos/cloud/login")
    async def cancel_cloud_login() -> dict[str, Any]:
        """Cancel the pending cloud login"""
        return s.uploads.cancel_login()

    @app.get("/dimos/cloud/login/page", response_class=HTMLResponse)
    async def cloud_login_page(theme: str | None = None) -> str:
        """A small page for an app's iframe that runs the cloud login and posts {type:"dimos-cloud-login", state,
        email} to its parent (`?theme=light|dark`)"""
        return (Path(__file__).parent / "login_page.html").read_text()

    @app.post("/dimos/cloud/logout")
    async def cloud_logout() -> dict[str, Any]:
        """Log this machine out of Dimensional cloud"""
        return await s.uploads.logout()

    @app.get("/dimos/uploads")
    async def upload_list() -> dict[str, Any]:
        """The upload queue: each upload's state, progress, speed, time left and error"""
        return s.uploads.listing()

    @app.post("/dimos/uploads")
    async def enqueue_upload(
        path: str = Body(..., embed=True),
        robotId: str | None = Body(None, embed=True),
        kind: str | None = Body(None, embed=True),
    ) -> dict[str, Any]:
        """Upload a recording (.mcap or .db) to Dimensional cloud: it joins the queue (one at a time)"""
        try:
            return s.uploads.enqueue(path, robotId, kind)
        except ValueError as error:
            raise ApiError(400, str(error))

    @app.delete("/dimos/uploads")
    async def clear_uploads() -> dict[str, Any]:
        """Clear the finished uploads (done, failed, cancelled) from the list"""
        return s.uploads.clear_finished()

    @app.get("/dimos/uploads/uploaded")
    async def uploaded(path: str | None = None) -> Any:
        """Which recordings are in the cloud, by path, with a console link; `?path=` for one (null: not uploaded)"""
        return s.uploads.uploaded() if path is None else s.uploads.uploaded_one(path)

    @app.delete("/dimos/uploads/{id}")
    async def cancel_upload(id: str) -> dict[str, Any]:
        """Cancel a queued or running upload, or remove a finished one from the list"""
        try:
            s.uploads.cancel(id)
        except KeyError as error:
            raise ApiError(404, error.args[0])
        return {"ok": True}

    @app.post("/dimos/uploads/{id}/retry")
    async def retry_upload(id: str) -> dict[str, Any]:
        """Queue a failed or cancelled upload again"""
        try:
            return s.uploads.retry(id)
        except KeyError as error:
            raise ApiError(404, error.args[0])
        except ValueError as error:
            raise ApiError(409, str(error))

    return app
