# sim2: Follow The Loops

Open `index.html` directly. No server, simulator, installation or network
request is needed. This is a linear architecture walkthrough, not a runtime
control panel. It replaces the rejected AST/symbol-table/Graphviz UI.

The main example is the existing G1 GR00T simulation blueprint:

1. ControlCoordinator: read joints/IMU, compute tasks, arbitrate, write targets.
2. Physics: read latest targets, apply motors, step MuJoCo, publish observations.
3. Device feedback: turn the robot observation into normal DimOS messages.
4. Camera and lidar: independent capture loops reading world snapshots.
5. Navigation: existing DimOS modules return the next desired velocity.

Construction, RPC/reset, viewers, M20/xArm differences and design questions
follow the main story. Helpers, data channels and definitions are not counted
as separate running loops. Each loop's actual owner and code path are named.

## Evidence And Limits

Source-reviewed on 2026-09-11 in the native sim2 worktree, including existing
uncommitted changes. No simulator restart or production-code edit occurred.
This is not the robosuite comparison branch.

G1 configured targets: controller 50 Hz, physics 200 Hz, public device feedback
200 Hz, camera and lidar 10 Hz each. Feedback gets its rate from the Imu
definition through `simulation()`, overriding the connection's class default.
World snapshots are published inside the physics loop, nominally near 60 Hz
and quantized to whole physics steps. The timing strip is illustrative, not
a measured trace or guarantee about relative thread wakeup order.

M20 uses 50 Hz control and 1,000 Hz physics. xArm7 uses the default 100 Hz
coordinator, 200 Hz physics, native position servos, 50 Hz joint feedback and
a 10 Hz wrist camera. It has no whole-body IMU or lidar in this definition.

The prose explains source behavior and architectural tradeoffs; it is not a
formal proof, live profiler or simulator acceptance result. It explicitly
notes independent clocks, latest-target reuse, model-copy cost, overlapping
joint publication, ground-truth odometry and ideal lidar. The per-physics-step
actuator path has no elapsed-time stale-target watchdog; task velocity-command
timeouts are a separate mechanism. No fix is implied by documenting this.

## Source Excerpts

`build_source.py` reads selected functions/classes using Python's AST parser,
without importing DimOS, resolving blueprints, loading models or starting
threads. It writes a small offline `source.js` containing exact source text,
line numbers, file hashes and capture time. The browser does not expose an AST.
Rebuild after changing relevant source:

```bash
.venv/bin/python docs/sim2-map/build_source.py
open docs/sim2-map/index.html
```

The prose and rate claims remain source-reviewed documentation. They do not
automatically change when a default or blueprint changes. Source links open
local files; excerpts can also be expanded within the page.

The download icon is Lucide 0.468.0; its license is retained in
`THIRD_PARTY_LICENSES.txt`. No Graphviz, npm or browser library is needed to
build or open this replacement. Browser review notes retain the existing
local-storage key, and export includes earlier notes without deleting them.

Browser checks passed for desktop/mobile layout (1440px/390px), source
expansion, chapter links, review-note persistence/export and every local
reference. No JavaScript errors or document horizontal overflow; screenshots
were inspected. This is separate from robot behavior or transport verification.
