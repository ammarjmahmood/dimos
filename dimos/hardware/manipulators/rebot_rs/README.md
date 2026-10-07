# Seeed Studio reBot Arm B601 RS

This adapter supports the seven RobStride motors in the B601 RS over SocketCAN. IDs 1 through 6 are arm joints. ID 7 is the gripper motor coordinate.

Install `motorbridge` separately before importing the adapter. Configure SocketCAN at 1,000,000 bit per second and verify motor zero calibration with MotorBridge Studio. Connection verifies all motors and reads positions. It does not enable torque.

Call `confirm_zero_pose()` while the arm is physically at its zero pose after every connection. `activate()` refuses to enable torque until that succeeds. The caller must provide an independent emergency stop and remain able to remove power.

Activation configures a 250 millisecond motor CAN timeout before torque is enabled. A stopped streaming loop therefore cannot leave a stale command active indefinitely. The physical behavior of that timeout still requires validation on the installed firmware.

```python
from dimos.hardware.manipulators.registry import adapter_registry

arm = adapter_registry.create("rebot_rs", channel="can0")
arm.connect()
assert arm.confirm_zero_pose()
```

Physical activation is intentionally omitted from the example. The default gripper motor limits are broad and must be replaced with values measured for the installed linkage. This adapter has only been tested with a simulated Motorbridge controller so far.
