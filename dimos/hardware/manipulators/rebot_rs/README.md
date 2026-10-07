# Seeed Studio reBot Arm B601 RS

This adapter supports the seven RobStride motors in the B601 RS over SocketCAN. IDs 1 through 6 are arm joints. ID 7 is the gripper motor coordinate.

The default motor models, MIT gains, 200 Hz command rate, joint limits, 110 degree Celsius temperature threshold and gripper range follow the current Seeed Studio B601 RS documentation and control configuration. Review them against the installed arm before activation. Sources: https://wiki.seeedstudio.com/rebot_arm_b601_rs_mit_control/ and https://github.com/LAN-GER/rebot_control/blob/main/config/rebotarm_rs.yaml

Install `motorbridge` separately before importing the adapter. Configure SocketCAN at 1,000,000 bit per second and verify motor zero calibration with MotorBridge Studio. Connection verifies all motors and reads positions. It does not enable torque.

Call `confirm_zero_pose()` while the arm is physically at its zero pose after every connection. `activate()` refuses to enable torque until that succeeds. The caller must provide an independent emergency stop and remain able to remove power.

Activation configures a 250 millisecond motor CAN timeout before torque is enabled. A stopped streaming loop therefore cannot leave a stale command active indefinitely. The physical behavior of that timeout still requires validation on the installed firmware.

```python
from dimos.hardware.manipulators.registry import adapter_registry

arm = adapter_registry.create("rebot_rs", channel="can0")
arm.connect()
assert arm.confirm_zero_pose()
```

Physical activation is intentionally omitted from the example. Measure the installed gripper range before relying on the default. The adapter passes mock tests on an x86 desktop and a physical Jetson Orin. Registry discovery, direct adapter connection and position reads have been exercised on the physical B601 RS. The zero guard rejected the observed encoder readings and no activation was attempted. Motion qualification remains pending. See `VALIDATION.md` for the measured scope and outstanding checks.

Activation rechecks the zero readings before motor configuration. Failed zero checks revoke any previous confirmation. A physical zero pose does not establish encoder calibration. If readings disagree with the prescribed pose, complete the Seeed calibration procedure rather than widening the confirmation tolerance.

Status freshness comes from an independent local SocketCAN receive socket with Linux kernel packet timestamps. Every active motor must supply a recent status report. Initial activation requires a reset mode report with a finite temperature and no fault bits. Receiver queue overload, absent timestamps, stale status and unexpected motor mode cause a fault. Physical timing and motor timeout behavior remain unqualified.

`activate_gripper()` is an explicit diagnostic path. It enables only motor 7 after the full zero check. It uses a written and verified 0.5 N m torque limit, MIT gains 2 and 0.1, maximum speed two degrees per second and maximum acceleration five degrees per second squared. Targets are restricted to zero through five motor degrees. Arm joints cannot be commanded through this path. Disconnect disables motor 7. The torque limit is changed on the motor; no parameter store command is issued. Do not run this diagnostic without mechanical support and explicit local authorization.
