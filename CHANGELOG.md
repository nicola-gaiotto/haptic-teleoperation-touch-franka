# Changelog

All notable milestones of the project. The newest entry is at the top.
Format: `## [version] - date` followed by what was added, changed or fixed.

## [Unreleased]
### Added
- `franka_teleop`: contact force limit (`contact_force_limit`, 10 N): no motion further into a
  contact above the limit. Lab 2026-11-20: a rigid contact reached 30 N in 0.5 s (collision
  reflex); with the limit, 3 contacts on a covered cube stayed at about 10 N, no reflex.
- Force-mode calibration with a kitchen scale (`tools/calibration/force_calibration.py`,
  up/down force sequence to cancel the joint friction): `force_gain` 6780 command/N m
  (was 2000, estimated); maximum force about 0.7 N. Defaults of walls and force feedback
  converted to real newtons. See [docs/calibration.md](docs/calibration.md).
- Package `franka_teleop`: keyboard velocity teleoperation of the Franka Research 3 through
  franky (short Cartesian velocity motions, starts disabled, 2 cm/s limit, ±10 cm workspace
  box, command timeout, `/franka/recover` service, keys in the operator frame via
  `operator_yaw_deg`, optional `go_home` to the Franka ready pose). Tested in franky-sim with
  ROS 2 Jazzy and on the real FR3 in the lab (ROS 2 Humble, franky 1.1.1);
  see [docs/04_franka_teleoperation.md](docs/04_franka_teleoperation.md).
- `franka_teleop`: node `touch_to_franka` (Touch stylus position -> Franka target, scale,
  clutch) and position input `/franka/target` in `franka_teleop` (proportional loop, target
  clipped to the workspace box); launch file `touch_to_franka.launch.py`. Tested on the real
  FR3 in the lab (2026-11-17): the robot follows the stylus, lag ~0.33 s and mean error
  8 mm at 5 cm/s; see [docs/05_touch_to_franka.md](docs/05_touch_to_franka.md).
- Touch driver and keyboard teleoperation running on the lab PC (ROS 2 Humble, pyusb with
  `pip --user`, udev rule by the tutor); `/touch/pose` at 100 Hz.
- Force feedback Franka -> Touch (lab, 2026-11-17 and 2026-11-19); see
  [docs/06_force_feedback.md](docs/06_force_feedback.md):
  - `touch_raw_driver`: `force` mode, Cartesian force from `/touch/force` rendered with the
    Jacobian transpose (`force.py`), with force limit, timeout, slew limit and damping;
    `/touch/force_rendered`. Directions verified on the device.
  - `franka_teleop`: `/franka/wrench_ext` (O_F_ext_hat_K, 100 Hz) and `/franka/workspace_centre`.
  - `touch_to_franka`: virtual walls at the workspace box (`walls_enabled`, `fake_franka` for
    Touch-only tests) and FR3 external force on the Touch (`force_feedback_enabled`: bias at
    engage, sign, low-pass, dead zone, gain).
- `tools/analysis/touch_franka_bag_report.py`: lag, error and speed from a Touch -> Franka
  rosbag (reads the sqlite3 bag without ROS), optional plot.
### Found
- Starting far from the base (0.63 m) the FR3 rejected commands near the box corner
  ("cannot start at singular pose"); starting from the ready pose avoids it.
- Lab PC booted with the generic kernel: franky refuses to run ("Running kernel does not have
  realtime capabilities"); boot the `6.8.0-rt8` kernel.
- With max_speed 2 cm/s the robot speed saturates during Touch teleoperation (lag ~1.2-1.5 s);
  5 cm/s works better.
- O_F_ext_hat_K is the force the robot applies to the environment (pushing down gives +Fz) and
  changes by several newtons with the pose and during motion, without contact: a 2 N dead zone
  was needed. Pushing the robot by hand above 30 N triggers the collision reflex.
- A force-feedback episode oscillated (about 1 Hz) without Touch damping; with 3 N s/m damping
  and the 2 N dead zone no sustained oscillation was seen.
### Planned
- Stylus orientation in `/touch/pose`.
- Stylus buttons (gripper, clutch) for the Franka teleoperation.

## [0.3.0] - 2026-10-28 - Keyboard teleoperation and GUI (validated by the tutor)
### Added
- Package `touch_teleop`: node `twist_to_target` that turns velocity commands
  (`geometry_msgs/Twist`, e.g. from `teleop_twist_keyboard`) into a continuously moving
  Cartesian target, with workspace box, lead limit and command timeout.
- Launch file with driver, velocity bridge, `world → touch_base` transform, dashboard
  and optional RViz; RViz configuration (stylus position, teleop target, workspace box).
- `touch_dashboard`: control panel (position read-out, motors/mode buttons, speed) with
  arrow-key teleoperation that stops exactly on key release.
- 3D view in RViz: model of the Touch (omni_description URDF) driven in real time through
  `touch_model_bridge` and `robot_state_publisher`, operator viewpoint, target and workspace.
- `touch_raw_driver`: gimbal angles (stylus orientation sensors) in `/touch/joint_states`.
### Changed
- `touch_raw_driver`: log messages for incoming targets are throttled (continuous targets).
- `touch_raw_driver`: velocity feedforward and direction-dependent friction compensation;
  tuned defaults (lag from up to 1.2 s down to ~0.1 s).
- Teleoperation workspace top lowered to z = 5 cm (higher up the motors cannot hold the arm).
- Tools: `tools/analysis/teleop_tracking_report.py` (lag, errors, saturation, plot) and
  `tools/calibration/friction_identification.py` (breakaway friction per joint).
### Fixed
- `twist_to_target`: downward drift when moving left/right (lead limit now per commanded
  axis; resync by distance instead of after every pause).
- `touch_raw_driver`: crash when publishing the gimbal angles in `/touch/joint_states`.
- 3D model in RViz: missing TF prefix, and black meshes (normals recomputed at launch).
### Removed
- Files that were never used to obtain results: `tools/usb/raw_state_monitor.py`,
  `tools/calibration/Makefile`, unit test stub, `docs/repository_workflow.md`,
  `docs/media/README.md`, and the legacy gravity-model format in the driver.

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
