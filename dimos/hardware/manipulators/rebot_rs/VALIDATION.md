# B601 RS validation record

## October 7, 2026

Nine adapter mock tests pass on an x86 Linux desktop and on a physical development Jetson Orin using Python 3.11. The isolated environment loads the real DimOS manipulator protocol and registry from the PR source. Motors are replaced only in the adapter mock tests. The full DimOS agent stack was not installed or exercised.

The Jetson test run completed in 0.82 seconds. An independent import, registry discovery and Motorbridge native ABI load took 524.90 milliseconds wall time and 802.73 milliseconds process CPU time, with maximum resident memory of 53,792 KiB. CPU time can exceed wall time because native libraries use multiple threads. These are initialization measurements without CAN access. They do not establish control loop latency or watchdog timing. This Jetson test does not qualify a Raspberry Pi deployment.

Primary package versions match the PR lock file for Python 3.11 Linux: Motorbridge 0.5.6, dimos-lcm 0.1.4, NumPy 2.3.5, plum-dispatch 2.5.7, reactivex 4.1.0, structlog 25.5.0, sortedcontainers 2.4.0 and pytest 8.3.5. The isolated installation includes the necessary adapter dependencies rather than every optional DimOS dependency. Missing unrelated pytest plugins produce three configuration warnings.

A separate Motorbridge diagnostic on the physical B601 RS discovered all seven motor IDs on the Jetson USB CAN interface `can1` at 1,000,000 bit per second using host ID `0xFD`. A separate position probe read joint angles, in order, of 2.527, 0.210, 0.176, 0.094, 9.268, negative 178.407 and negative 8.598 degrees. The operator reported that the arm was physically at its zero pose. These encoder readings do not satisfy the adapter zero check. A regression test verifies refusal without configuration writes or torque enable.

Three seconds of passive observation after position reads produced no status reports for temperature qualification. Missing reports were retained as unavailable. No active report setting, zero value, mode or torque command was written. The CAN interface was independently verified down afterward.

The actual DimOS registry then created `rebot_rs` using the installed native Motorbridge aarch64 library. Its direct connection to the physical arm succeeded in 142.00 milliseconds. Position reads returned 2.529, 0.208, 0.183, 0.086, 9.261, negative 178.409 and negative 8.578 degrees. The adapter zero check returned false with joints 5, 6 and 7 identified. The adapter reported enabled state false and activation was never called. The connection, position reads and zero check took 324.86 milliseconds including CAN interface setup, with maximum resident memory of 53,720 KiB. Disconnect and interface cleanup were verified afterward. This establishes physical connection and guard refusal, not motion readiness.

## Qualification still required

Zero calibration reconciliation, gripper range, independent emergency stop, per joint signs, holding behavior, timeout behavior, tracking faults, thermal checks and control timing under load remain pending. No physical movement was performed during this validation session.

Motorbridge `get_state()` returns cached state without a receive timestamp. The adapter freshness calculation does not yet establish that every motor has delivered a new report. The proposed CAN timeout and feedback checks must not be described as verified link loss protection until cached feedback handling and each motor timeout are tested. Keep this contribution in draft until that safety gap is resolved and qualified.

Seeed calibration procedure: https://wiki.seeedstudio.com/rebot_b601_rs_getting_started/ . RobStride protocol evidence: https://github.com/RobStride/Python_Sample/blob/main/robstride_dynamics/protocol.py and https://github.com/RobStride/Python_Sample/blob/main/robstride_dynamics/bus.py . Motorbridge feedback semantics: https://github.com/motorbridge/motorbridge/blob/main/bindings/python/README.md .
