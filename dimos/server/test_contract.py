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


"""Every /dimos path and method in dimOS Desktop's OpenAPI (fixtures/desktop_openapi_dimos.json, saved from
`GET /api/desktop/openapi` on Desktop 0.2.0, jeff/agent_chat as of 2026-10-05) is served here, so Desktop can switch to this server unchanged."""

import json
from pathlib import Path
import re

from fastapi.routing import APIRoute

from dimos.server.app import ServerState, create_app
from dimos.server.events import Bus
from dimos.server.uploads import Uploads

FIXTURE = Path(__file__).parent / "fixtures" / "desktop_openapi_dimos.json"


def normalized(path: str) -> str:
    return re.sub(r"\{[^}]+\}", "{}", path)


def test_every_desktop_dimos_endpoint_is_served(tmp_path: Path) -> None:
    contract = json.loads(FIXTURE.read_text())["paths"]
    bus = Bus()
    app = create_app(
        ServerState(tmp_path, bus, Uploads(tmp_path, bus, None, tmp_path / "log")), background=False
    )
    served = {
        (normalized(route.path), method.lower())
        for route in app.routes
        if isinstance(route, APIRoute)
        for method in route.methods
    }
    wanted = {
        (normalized(path), method) for path, methods in contract.items() for method in methods
    }
    assert len(wanted) >= 25
    assert wanted - served == set()
