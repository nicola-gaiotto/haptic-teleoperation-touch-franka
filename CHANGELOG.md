# Changelog

All notable milestones of the project. The newest entry is at the top.
Format: `## [version] - date` followed by what was added, changed or fixed.

## [Unreleased]
### Added
- Package `touch_teleop`: node `twist_to_target` that turns velocity commands
  (`geometry_msgs/Twist`, e.g. from `teleop_twist_keyboard`) into a continuously moving
  Cartesian target, with workspace box, lead limit and command timeout.
- Launch file with driver, velocity bridge, `world → touch_base` transform, dashboard
  and optional RViz; RViz configuration (stylus position, teleop target, workspace box).
- `touch_dashboard`: 2D GUI (top and front views, position read-out, motors/mode buttons)
  with arrow-key teleoperation that stops exactly on key release.
### Changed
- `touch_raw_driver`: log messages for incoming targets are throttled (continuous targets).
### Fixed
- `twist_to_target`: downward drift when moving left/right (lead limit now per commanded
  axis; resync by distance instead of after every pause).
### Planned
- Gravity compensation tuning (gain, thermal limit) and validation.
- Stylus orientation from the gimbal encoders.
- Bridge node towards the Franka Panda.

## [0.2.0] - 2026-10 - Deliverable 2: commanding the Touch from ROS 2
### Added
- `touch_raw_driver`: ROS 2 driver that commands the Touch motors directly over USB
  (reverse-engineered HID protocol), bypassing OpenHaptics.
- Joint-space PID control with speed-limited reference, saturation and safe shutdown.
- Kinematic model identified against OpenHaptics (read-only): `/touch/pose` in metres
  and Cartesian targets on `/touch/target` (analytic inverse kinematics).
- Physical gravity-compensation model, `float` mode, thermal protection with
  hysteresis and integral anti-windup.
- Tools: USB motor tests, motor/encoder characterisation, kinematic and gravity
  calibration.
### Found
- On Ubuntu 24.04 the 3D Systems Linux library always sends zero motor current
  (verified with usbmon), with both driver versions 2024.09.19 and 2025.12.10.

## [0.1.0] - 2026-09 - Deliverable 1: reading the Touch pose in ROS 2
### Added
- Setup of TouchDriver + OpenHaptics 3.4 on Ubuntu 24.04 / ROS 2 Jazzy.
- Geomagic_Touch_ROS2 driver (third party) with a fix of the publishing timer
  (from ~18 kHz to the intended 1 kHz).
- Verification of units and axes of `/phantom/pose`.
