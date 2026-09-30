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
