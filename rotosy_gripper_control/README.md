# rotosy_gripper_control

Doosan E0509 flange I/O examples for RoToSY gripper control.

## Hardware setup

1. Wire the electromagnet gripper control input to flange `DO1`.
2. Wire the gripper reference to flange `GND`.
3. In DART, set `Flange I/O` `Supply Voltage` to `12V`.
4. Keep the DO1 load within the Doosan flange digital output rating. Use an
   external relay or isolated driver if the electromagnet current exceeds the
   flange DO rating.

## Keyboard example

Start the Doosan driver first so `/dsr01/io/set_tool_digital_output` exists.

```bash
ros2 run rotosy_gripper_control keyboard_electromagnet_gripper
```

Keys:

- `o`: DO1 ON, outputs 12V while the flange supply voltage is set to 12V.
- `f`: DO1 OFF, output becomes open.
- `q`: quit. The node turns DO1 OFF before exiting by default.

With a different DSR namespace:

```bash
ros2 run rotosy_gripper_control keyboard_electromagnet_gripper --ros-args -p robot_ns:=dsr01
```
