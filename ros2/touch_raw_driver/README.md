# touch_raw_driver

ROS 2 (Jazzy, `ament_python`) driver for the 3D Systems Touch that commands the motors
directly over USB, without OpenHaptics.

```bash
ros2 launch touch_raw_driver touch_raw_driver.launch.py     # with config/touch_raw_driver.yaml
# or
ros2 run touch_raw_driver touch_raw_driver                  # with the defaults in the code
```

- Interface (topics, frame): see the main [README](../../README.md#ros-2-interface-of-touch_raw_driver).
- Design and results: [docs/02_deliverable2_motor_control.md](../../docs/02_deliverable2_motor_control.md).
- Calibration files: [docs/calibration.md](../../docs/calibration.md).
- Safety: [docs/safety.md](../../docs/safety.md).

Unit tests of the kinematics (no hardware needed):

```bash
colcon test --packages-select touch_raw_driver && colcon test-result --verbose
```
