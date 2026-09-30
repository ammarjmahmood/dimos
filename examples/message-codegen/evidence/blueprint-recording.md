# Native blueprint recording and replay

Verified locally on 2026-09-30 using the current runtime batch on top of
`f4a269ce4bc766a1ff876ed2b6f5ac432e6d3900`.

- 33 recorder, SQLite path, stamped-message and TF regression tests passed.
- Strict scoped mypy passed for RustRecorder and the SQLite replay helper.
- The actual Python -> C++ -> Rust -> Python Zenoh relay produced three
  custom LineSegments3D samples, three 640x480 RGB images, and three PoseStamped
  samples. Native edits incremented line weight twice; image bytes were exact.
- Rust recorder reported received=9, written=9, encode_errors=0 and exited cleanly.
- MCAP had CDR channels and embedded ros2msg schemas. Independent rosbags Jazzy
  decoding succeeded for every row; all three streams contained samples 0,1,2.
- After all producers stopped, DimOS ReplayModule emitted every recorded value
  with byte equality and per-stream order preserved.

The retained local artifact is
`build/message-codegen/demo/evidence/blueprint-current-zenoh-v3.mcap`; terminal
log is `/tmp/cdr-blueprint-recording-v3.log`. Earlier failed artifacts were
preserved. This demo uses installed built-in custom messages; arbitrary installed
external-message capture has separate acceptance requirements.

The first recorder run exposed inherited TF wiring being passed despite
`record_tf=False`. RustRecorder now filters native launch topics to declared
recording streams; regression tests cover TF enabled and disabled. Another run
exposed float-second replay treating 1 ns spaced epoch timestamps as static.
The demo now uses realistic 100 ms intervals while checking exact nanoseconds;
sub-float-resolution scheduling is not claimed.

No hardware ran and host tuning was check-only. Default-interface LCM, human
Foxglove/Rerun inspection, full dependency retirement and final installed-package
acceptance remain open. This evidence does not mark stages 5 or 6 complete.

## Follow-up regression batch

The published parent `f4a269ce4` codegen workflow 36763634296 succeeded. Main
workflow 36763630343 ended cancelled after ARM reported 6170 passed, 228 skipped
and one documentation-branding failure. Both offending spellings in
`docs/usage/lcm.md` are repaired; other Python jobs were cancelled, not passed.

Follow-up local checks: 15 Joy/Header/LineSegments3D/branding tests and 53
stamped-covariance/time tests passed. Migrated suites retain numeric payloads,
empty and large sequences, covariance patterns, copy independence, explicit
clock and datetime conversions, and add independent ROS CDR checks. Legacy
convenience constructors, custom string formatting, inheritance, Path-based
line packing and implicit timestamps are intentionally retired. Generated ROS
time overflow now reports an explicit ValueError; both int32 boundaries and
values immediately outside them are covered. Scoped mypy passed time.py.
