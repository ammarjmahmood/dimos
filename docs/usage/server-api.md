# The dimos server's HTTP API

`python -m dimos.server` serves the `/dimos` HTTP API that dimOS Desktop uses: blueprints, global config, runs and
their logs, events, and Dimensional cloud uploads. Desktop starts it on a unix socket and forwards `/dimos/...` to it
unchanged (`--port 8123` also serves it on `127.0.0.1:8123`).

## The OpenAPI document

Every endpoint is described in an OpenAPI 3.1 document: summaries and descriptions, request and response schemas
with examples, parameters, error answers (always `{"error": "<message>"}`), and each event's payload.

- **Live:** `GET /dimos/openapi.json` from a running server. `info.version` is the API's version, and
  `info.x-dimos-version` is the version of dimos serving it.
- **Checked in:** [`dimos/server/openapi.json`](/dimos/server/openapi.json), which [`dimos.yaml`](/dimos.yaml) names
  under `api:`, next to the API's version. Desktop reads it per tag over HTTP without running anything.
- **Regenerate** the checked-in file after changing an endpoint or a model in
  [`dimos/server/models.py`](/dimos/server/models.py):

  ```sh
  python -m dimos.server --write-openapi
  ```

  A test fails while it's stale. The API is versioned with semver (`API_VERSION` in
  [`dimos/server/openapi.py`](/dimos/server/openapi.py#L18), and `api.version` in `dimos.yaml`): a breaking change is a
  major bump.

There is no Swagger page (`/docs`): it would load its scripts from a CDN, and robots are often offline. Load the JSON
into any OpenAPI viewer instead.

Operations carry Desktop's extensions: `x-family: dimos`; `x-agent: true` for what Desktop's agent can find; and
`x-mcp-tool` for an operation an MCP tool also does.

## Groups (tags)

| Tag             | What                                                                                 |
| --------------- | ------------------------------------------------------------------------------------ |
| `server`        | liveness, the checkout it serves, where things are, stopping it                      |
| `blueprints`    | the blueprint list, a blueprint's modules and config, the catalog of modules/skills  |
| `global-config` | dimos's GlobalConfig schema and defaults, and Desktop's saved overrides               |
| `runs`          | launching and stopping blueprints, and the live runs                                  |
| `logs`          | a run's structured log, with filters and tailing                                     |
| `cloud`         | the Dimensional cloud login (device flow) and account                                |
| `uploads`       | the upload queue, and which recordings are already in the cloud                      |
| `events`        | the event payloads, and the deprecated SSE stream                                    |
| `discovery`     | the discovery cache: blueprints, modules, their config, message types, robot ranking |
| `docs`          | the guide to adding your own robot, and links into the docs site                     |
| `extras`        | dimos's optional extras, which are installed, and installing more                    |
| `jobs`          | long jobs (an extras install): their output, live on zenoh and as a snapshot         |

## Launch diagnostics

A launch's `steps` and `problems` are stable codes with data, never words: clients own the wording. The server reads
them from the run's structured log (`main.jsonl`), not from the console:

- **Steps:** dimos logs a `stage` field as it starts: `starting`, `run_log` (where the run's log goes on, with
  `run_id` and `log_dir`), `building`, `starting_modules` (with `modules`, how many), `module_deployed`, and
  `started`.
- **Problems:** a refusal dimos knows logs a `problem` field (`bad_arguments`, `unknown_blueprint`,
  `requirement_unmet`). An exception is logged with `exception_chain` (every exception class behind it),
  `exception_code` (an errno or SQLite error name) and `missing_module` (`exception_fields` in
  [`dimos/utils/logging_config.py`](/dimos/utils/logging_config.py)). The server maps a class or code to a problem
  code in [`dimos/server/diagnose.py`](/dimos/server/diagnose.py).

To make a new failure recognizable, raise a dedicated exception class (like `MissingRobotIpError`) or log a `problem`
field, then add its code to `diagnose.py`. Don't match on message text. The server starts `dimos run` with
`DIMOS_RUN_LOG_DIR` set, so even the records from before the run has an id are in a file it can read.

## Events

The server publishes its events on zenoh at `<ns>/dimos/events/<type>` (`<ns>` is Desktop's namespace): `launch`,
`log`, `upload`, `uploads`, `upload-removed`, `cloud-login`, `discovery` and `job`. Each payload is a component schema in the document
(`LaunchEvent`, ... ; `DimosEvent` is any of them) with its zenoh key in `x-zenoh-key`. `GET /dimos/events` streams
the same events as server-sent events, and is deprecated.

A job's output lines are on `<ns>/dimos/jobs/<job>` (`{type: "line", n, line}`, then `{type: "done", ok, error,
failure, lines}`), the same shape as Desktop's own jobs; `GET /dimos/jobs/{job}/log?after=<n>` is the snapshot.

## Discovery

When the server starts it imports every blueprint in dimos's registry ([`dimos/robot/all_blueprints.py`](/dimos/robot/all_blueprints.py))
and every registry module once, in child processes ([`dimos/server/discover.py`](/dimos/server/discover.py)): a blueprint that
hangs or crashes its process costs only itself. The answer (whether each blueprint imports and why not, its modules
and their streams and topics, every module's config fields, the message types) is saved under
`<state>/dimos/server/discovery/`, keyed by the checkout's commit, its dirty files and the installed packages, so a
restart answers at once. The key is checked every 30 s and after an extras install; when it changes the old answer is
served (`stale: true`) until the new one is in. See [`dimos/server/discovery.py`](/dimos/server/discovery.py#L25).

`GET /dimos/robots/{robot}/modules` ranks the modules of a robot's blueprints by how specific they are to it:

```
score = (robot's blueprints using the module / robot's blueprints) * ln(robots / robots using the module)
```

so the robot's own connection module comes first and a module every robot uses scores 0. Robots are
[`dimos/robot/robots.json`](/dimos/robot/robots.json)'s, and only blueprints that import count.
