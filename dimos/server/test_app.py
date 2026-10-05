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


"""The /dimos routes, with fakes: a temporary DIMOS_HOME and state, a fake `dimos` and cloud worker, no network."""

from collections.abc import Iterator
import json
from pathlib import Path
import sys
import time
from typing import Any

from fastapi.testclient import TestClient
import pytest

from dimos.server import blueprints, config, events, runs
from dimos.server.app import ServerState, create_app
from dimos.server.uploads import Uploads


@pytest.fixture
def state(server_home: Path, checkout: Path, fake_worker: list[str]) -> ServerState:
    bus = events.Bus()
    uploads = Uploads(checkout, bus, None, server_home / "uploads.log", worker=fake_worker)
    return ServerState(dimos_dir=checkout, bus=bus, uploads=uploads)


@pytest.fixture
def client(state: ServerState) -> Iterator[TestClient]:
    with TestClient(create_app(state, background=False)) as client:
        yield client


def test_health_info_and_paths(
    client: TestClient, checkout: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    assert client.get("/healthz").text == "ok"
    assert client.get("/dimos/healthz").text == "ok"
    info = client.get("/dimos/info").json()
    assert info == {
        "dir": str(checkout),
        "found": True,
        "installed": True,
        "version": "0.0.14",
        "range": "",
        "inRange": True,
    }
    monkeypatch.setenv(config.RANGE_ENV, ">=0.0.14b1 <0.1")
    assert client.get("/dimos/info").json()["inRange"] is True
    monkeypatch.setenv(config.RANGE_ENV, ">=0.1")
    assert client.get("/dimos/info").json()["inRange"] is False
    paths = client.get("/dimos/paths").json()
    assert paths["dimosDir"] == str(checkout)
    assert paths["recordingsDir"] == str(config.dimos_home() / "recordings")
    assert set(paths) == {"dimosDir", "runsDir", "logsDirs", "recordingsDir"}
    missing = client.get("/dimos/nope")
    assert (missing.status_code, missing.json()) == (404, {"error": "no such route: /dimos/nope"})


def test_blueprint_list_is_read_in_process(client: TestClient) -> None:
    listed = client.get("/dimos/blueprints").json()["blueprints"]
    assert {"name": "unitree-go2-basic", "kind": "builtin"} in listed
    assert not any(b["name"].startswith("demo-") for b in listed)
    assert client.get("/dimos/blueprints?fresh=1").json()["blueprints"] == listed


# what introspect.py answers, in full, so the answers are checked against their models
INTROSPECTED: dict[str, dict[str, Any]] = {
    "blueprint": {
        "name": "unitree-go2",
        "modules": [
            {
                "name": "camera",
                "class": "dimos.hardware.camera.CameraModule",
                "streams": [
                    {"name": "color_image", "type": "dimos.msgs.Image", "direction": "out"}
                ],
            }
        ],
    },
    "config": {
        "name": "unitree-go2",
        "modules": [
            {
                "module": "camera",
                "class": "dimos.hardware.camera.CameraModule",
                "args": [
                    {
                        "name": "fps",
                        "type": "int",
                        "default": 30,
                        "description": "frames a second",
                        "required": False,
                        "base": False,
                        "value": 15,
                    },
                    {
                        "name": "mode",
                        "type": "Literal['rgb', 'depth']",
                        "default": "rgb",
                        "description": None,
                        "required": False,
                        "base": False,
                        "choices": ["rgb", "depth"],
                    },
                ],
            },
            {"module": "odd", "class": "x.Odd", "args": [], "error": "TypeError: not pydantic"},
        ],
    },
    "catalog": {
        "blueprints": [
            {
                "name": "unitree-go2",
                "ref": "dimos.robot.unitree.go2:bp",
                "robot": "go2",
                "modules": ["Cam"],
            }
        ],
        "modules": [
            {
                "name": "Cam",
                "class": "dimos.hardware.camera.CameraModule",
                "doc": "A camera.",
                "robots": ["go2"],
                "inputs": [],
                "outputs": [{"name": "color_image", "type": "Image"}],
                "skills": ["snap"],
            }
        ],
        "skills": [
            {
                "name": "snap",
                "doc": "Take a picture.",
                "params": [{"name": "size", "type": "int", "default": "1"}],
                "module": "Cam",
                "robots": ["go2"],
            }
        ],
        "errors": ["blueprint g1: ImportError: no mujoco"],
    },
}


def test_blueprint_details_are_introspected_and_cached(
    client: TestClient, monkeypatch: pytest.MonkeyPatch
) -> None:
    calls: list[list[str]] = []

    async def fake(dimos_dir: Path, args: list[str], **_: Any) -> dict[str, Any]:
        calls.append(args)
        if args[1:] == ["broken"]:
            raise blueprints.IntrospectError("ImportError: no torch")
        return INTROSPECTED[args[0]]

    monkeypatch.setattr(blueprints, "introspect", fake)
    assert client.get("/dimos/blueprints/unitree-go2").json() == INTROSPECTED["blueprint"]
    client.get("/dimos/blueprints/unitree-go2")
    # absent optional fields (choices, value, error) stay absent
    assert client.get("/dimos/blueprints/unitree-go2/config").json() == INTROSPECTED["config"]
    assert client.get("/dimos/catalog").json() == INTROSPECTED["catalog"]
    assert calls == [["blueprint", "unitree-go2"], ["config", "unitree-go2"], ["catalog"]]
    broken = client.get("/dimos/blueprints/broken")
    assert (broken.status_code, broken.json()) == (500, {"error": "ImportError: no torch"})
    assert client.get("/dimos/blueprints/-rf").status_code == 400


async def test_introspection_runs_in_a_child_with_a_timeout(tmp_path: Path) -> None:
    python = tmp_path / "python"
    python.write_text(
        f"#!{sys.executable}\n"
        "import sys, time, json\n"
        "args = sys.argv[3:]\n"
        "print('noise from an import')\n"
        "if args[1] == 'hang': time.sleep(30)\n"
        "if args[1] == 'crash': raise SystemExit('segfault-ish')\n"
        "result = {'error': 'KeyError: x'} if args[1] == 'bad' else {'name': args[1]}\n"
        "print('\\n@@DIMOS_SERVER@@' + json.dumps(result))\n"
    )
    python.chmod(0o755)
    assert await blueprints.introspect(tmp_path, ["blueprint", "ok"], python=str(python)) == {
        "name": "ok"
    }
    for name, message in [
        ("bad", "KeyError: x"),
        ("crash", "segfault-ish"),
        ("hang", "took over 1 s"),
    ]:
        with pytest.raises(blueprints.IntrospectError, match=message):
            await blueprints.introspect(
                tmp_path, ["blueprint", name], timeout=1, python=str(python)
            )


def test_global_config_overrides_live_in_desktops_config(client: TestClient) -> None:
    config.config_file().parent.mkdir(parents=True)
    config.config_file().write_text("desktop:\n  port: 7341\ndimos:\n  dir: /somewhere\n")
    value = client.get("/dimos/global-config").json()
    assert "robot_ip" in value["schema"]["properties"] and "robot_ip" in value["defaults"]
    assert value["overrides"] == {}
    saved = client.put(
        "/dimos/global-config", json={"overrides": {"robot_ip": "10.0.0.2", "n_workers": None}}
    )
    assert saved.json()["overrides"] == {"robot_ip": "10.0.0.2"}
    on_disk = config.load_desktop_config()
    assert on_disk["desktop"] == {"port": 7341} and on_disk["dimos"]["dir"] == "/somewhere"
    assert client.put("/dimos/global-config", json={"overrides": {"a-b": 1}}).status_code == 400
    assert client.put("/dimos/global-config", json={}).status_code == 400


def test_launch_log_and_stop(client: TestClient, monkeypatch: pytest.MonkeyPatch) -> None:
    assert client.get("/dimos/runs").json() == {"runs": [], "launch": None}
    config.config_file().parent.mkdir(parents=True)
    config.config_file().write_text("dimos:\n  global_config:\n    robot_ip: 10.0.0.2\n")
    launched = client.post(
        "/dimos/runs",
        json={"blueprint": "unitree-go2", "replay": True, "overrides": {"n_workers": 2}},
    ).json()
    assert (launched["blueprint"], launched["phase"]) == ("unitree-go2", "starting")
    assert launched["output"].startswith(
        "$ dimos --n-workers 2 --replay --robot-ip 10.0.0.2 run unitree-go2"
    )
    again = client.post("/dimos/runs", json={"blueprint": "unitree-go2"})
    assert again.status_code == 400 and "still starting" in again.json()["error"]

    # it shows as running once it is in dimos's registry
    entry = {
        "run_id": "r1",
        "pid": launched["pid"],
        "blueprint": "unitree-go2",
        "started_at": "t",
        "log_dir": "d",
    }
    monkeypatch.setattr(runs, "registry_runs", lambda: [entry])
    listed = client.get("/dimos/runs").json()
    assert listed["runs"] == [entry] and (listed["launch"]["phase"], listed["launch"]["runId"]) == (
        "running",
        "r1",
    )
    monkeypatch.setattr(runs, "registry_runs", lambda: [])

    stopped = client.post("/dimos/runs/stop")
    assert stopped.json() == {"output": f"stopped unitree-go2 (pid {launched['pid']})"}
    assert client.get("/dimos/runs").json()["launch"]["phase"] == "stopped"
    nothing = client.post("/dimos/runs/stop", json={"runId": "nope"})
    assert nothing.status_code == 500 and nothing.json() == {"error": "no live run nope"}
    assert client.post("/dimos/runs", json={"blueprint": "-x"}).status_code == 400


def test_launch_refuses_a_version_outside_desktops_range(
    client: TestClient, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv(config.RANGE_ENV, ">=0.1")
    refused = client.post("/dimos/runs", json={"blueprint": "unitree-go2"})
    assert refused.status_code == 400 and "outside the range" in refused.json()["error"]


def test_run_log(client: TestClient, checkout: Path) -> None:
    run = checkout / "logs" / "20260101-000000-x"
    run.mkdir(parents=True)
    (run / "main.jsonl").write_text(
        '{"level":"info","event":"a"}\n{"level":"error","event":"b","logger":"nav"}\n'
    )
    page = client.get("/dimos/runs/latest/log?level=warning").json()
    assert (page["runId"], [r["event"] for r in page["records"]], page["loggers"]) == (
        "20260101-000000-x",
        ["b"],
        ["nav"],
    )
    tail = client.get(f"/dimos/runs/20260101-000000-x/log?after={page['offset']}").json()
    assert tail["records"] == []


async def test_events_start_with_the_launch_then_follow_the_bus() -> None:
    bus = events.Bus()
    stream = bus.stream(lambda: {"type": "launch", "launch": None}, keep_alive=0.05)
    assert await anext(stream) == 'data: {"type": "launch", "launch": null}\n\n'
    bus.send({"type": "upload-removed", "id": "u1"})
    assert json.loads((await anext(stream))[len("data: ") :]) == {
        "type": "upload-removed",
        "id": "u1",
    }
    assert await anext(stream) == ":\n\n"
    await stream.aclose()
    assert not bus.queues


def test_uploads(client: TestClient, tmp_path: Path) -> None:
    mcap = tmp_path / "a.mcap"
    mcap.write_bytes(b"1234")
    bad = client.post("/dimos/uploads", json={"path": str(tmp_path / "a.txt")})
    assert bad.status_code == 400 and bad.json()["error"].startswith("no such file")
    upload = client.post("/dimos/uploads", json={"path": str(mcap), "robotId": "go2"}).json()
    assert (upload["id"], upload["state"], upload["robotId"], upload["size"]) == (
        "u1",
        "queued",
        "go2",
        4,
    )
    assert client.get("/dimos/uploads").json() == {"uploads": [upload], "waitingForLogin": False}
    assert client.post("/dimos/uploads/u1/retry").status_code == 409
    assert client.delete("/dimos/uploads/u1").json() == {"ok": True}
    assert client.post("/dimos/uploads/u1/retry").json()["state"] == "queued"
    assert client.delete("/dimos/uploads/nope").status_code == 404
    assert client.post("/dimos/uploads/nope/retry").status_code == 404
    client.delete("/dimos/uploads/u1")
    assert client.delete("/dimos/uploads").json() == {"uploads": [], "waitingForLogin": False}
    assert client.get("/dimos/uploads/uploaded").json() == {"byPath": {}}
    assert client.get("/dimos/uploads/uploaded", params={"path": str(mcap)}).json() is None


def test_cloud_login_account_logout(client: TestClient) -> None:
    assert client.get("/dimos/cloud/login").json()["state"] == "idle"
    assert client.get("/dimos/cloud/account").json()["email"] == "a@b.c"
    started = client.post("/dimos/cloud/login").json()
    assert (started["state"], started["code"]) == ("pending", "ABCD")
    assert client.delete("/dimos/cloud/login").json()["state"] == "idle"
    assert client.post("/dimos/cloud/logout").json()["loggedIn"] is True
    page = client.get("/dimos/cloud/login/page?theme=dark")
    assert page.headers["content-type"].startswith("text/html") and "dimos-cloud-login" in page.text


def test_server_stop(client: TestClient, state: ServerState) -> None:
    stopped: list[bool] = []
    state.exit = lambda: stopped.append(True)
    assert client.post("/dimos/server/stop").json() == {"stopping": True}
    for _ in range(40):
        if stopped:
            break
        time.sleep(0.05)
    assert stopped
