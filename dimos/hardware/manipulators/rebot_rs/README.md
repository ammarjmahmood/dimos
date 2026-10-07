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

Physical activation is intentionally omitted from the example. Measure the installed gripper range before relying on the default. This adapter has only been tested with a simulated Motorbridge controller so far.
