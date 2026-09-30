# Desktop recovery and continued CDR validation

Verified on 2026-09-30 in the dedicated `feat/cdr-runtime-cutover` worktree.
This is a progress checkpoint, not final migration acceptance.

## Preserved history

The recovered local tip `ae00dd3e8c` was clean. CC Desktop had three linear
successors, `87ad59d853`, `08cb4eb4ba`, and `be3832092d`. They were fetched and
fast-forwarded locally without changing the source machine or rewriting commits.
These migrate MCP observation images, keyboard commands and hosted map compression.

## Changes and visible demos

- `scripts/test_message_user_story.sh` generates an application-local
  `story_msgs/msg/DeviceReading`, builds Python/C++/Rust, and runs a real Python
  module handler followed by native CDR file consumers. Output returns sequence
  42, value 23.5, label `new-local-type/python/cpp/rust`, preserving nanoseconds.
- `demo_video_stats.py` converts browser JSON to the explicit generated
  `dimos_msgs/msg/VideoStats`, records and reopens SQLite, and displays the exact
  integer counter 4294967297. It removes its temporary database automatically.
- Video telemetry consumers now import the generated type. The old positional
  Joy carrier and its legacy codec methods are removed. Invalid unsigned metric
  values fail at the JSON boundary with a descriptive ValueError.
- The unused legacy `lcm_msg_type` discovery function is removed. Tests resolve
  generated types by `package/msg/Type`; old dotted names do not import codecs.
- `docs/development/messages.md` provides message-authoring, Python module,
  C++/Rust codec and native module, packaging and recording/replay user stories.
  Its runnable shell commands were executed and local source/doc links checked.

## Executed validation

Use the generated demo extension on PYTHONPATH and the activated project venv.
The restored native dependency prefix is `build/message-codegen/install`.

- Generated/built the complete demo definitions including the new telemetry type.
- 79 Python tests passed across generated video stats/type discovery, memory CDR
  codecs, MCAP store, Rust recorder configuration, MCP images, keyboard and map
  compression. Tests used `--noconftest --import-mode=importlib -o addopts=''`.
- 3 native recorder E2E tests passed over explicit local Zenoh TCP: Rust SQLite
  and MCAP artifacts read through Python, plus TF capture and Python replay.
  The two LCM CLI cases were deselected. Tests used the actual local Cargo-built
  `target/debug/dimos-memory-recorder`; no result from the separate main-branch
  MCAP compatibility thread is used as CDR evidence.
- `cargo test -p dimos-memory-recorder --lib`: 16 passed.
- `cargo test -p dimos-module --lib`: 146 passed.
- `cargo build -p dimos-native-module-examples --offline`: passed after fetching
  the lockfile dependencies during explicit setup/build.
- Ruff check/format, shell syntax and diff whitespace checks passed for this
  change. Mypy passed the two changed helper modules with generated typing on
  MYPYPATH. This is not a full-repository mypy claim.

Terminal and build logs remain ignored under
`build/message-codegen/demo/evidence/recovery-*`; user-story CDR files and its
terminal capture live under `build/message-codegen/user-story/evidence/`.
Standard setuptools egg-info was generated to expose the checkout's declared
`dimos.messages` entry point; simply putting an uninstalled extension on
PYTHONPATH does not register an installed provider.

## Remaining gates

The OpenSpec checklist remains unchanged because these slices do not complete
all stage-4, stage-5 or stage-6 requirements. Old wrappers and consumers remain
in other teleop, robot and perception paths. The public type replacement and
complete old-dependency removal still need coordinated migration.

The C++ native SDK needs local LCM and pinned Zenoh C/C++ development packages;
this host currently lacks those packages. The standalone C++ codec consumer was
built and run, but a C++/Rust transport relay was not rerun here. LCM default
multicast self-test is blocked on this host; no networking settings were changed.
Full blueprint/viewer demo, final installed packaging matrix, authenticated
Foxglove UI evidence, and exact published-commit CI acceptance remain pending.
No hardware commands, model inference or model downloads were used.
