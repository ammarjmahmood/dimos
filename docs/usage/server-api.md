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

## Events

The server publishes its events on zenoh at `<ns>/dimos/events/<type>` (`<ns>` is Desktop's namespace): `launch`,
`log`, `upload`, `uploads`, `upload-removed` and `cloud-login`. Each payload is a component schema in the document
(`LaunchEvent`, ... ; `DimosEvent` is any of them) with its zenoh key in `x-zenoh-key`. `GET /dimos/events` streams
the same events as server-sent events, and is deprecated.
