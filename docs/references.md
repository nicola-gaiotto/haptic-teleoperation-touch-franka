# References and credits

## Third-party code

| Project | Use in this repository |
|---|---|
| [tstoyanov/Geomagic_Touch_ROS2](https://github.com/tstoyanov/Geomagic_Touch_ROS2) | **Used** for Deliverable 1 (tested at commit `7a13541`). Not copied here: only my patch is provided in `third_party/`. |
| [IvoD1998/Geomagic_Touch_ROS2](https://github.com/IvoD1998/Geomagic_Touch_ROS2) | Original ROS 2 port from which the tstoyanov version derives (consulted). |
| [bharatm11/Geomagic_Touch_ROS_Drivers](https://github.com/bharatm11/Geomagic_Touch_ROS_Drivers) | Original ROS 1 driver (consulted, to understand the force and "lock" logic). |
| [RViMLab/geomagic_touch_x_ros2](https://github.com/RViMLab/geomagic_touch_x_ros2) | Suggested by the supervisor; ROS 2 interface for the Touch X (consulted for the interface design). |
| [quantumbeam/3DSystems_Touch_ROS_Drivers](https://github.com/quantumbeam/3DSystems_Touch_ROS_Drivers) | ROS 1 fork tested with the HID Touch (consulted). |
| [OltanS/UR5e_Touch_haptic_teleoperation](https://github.com/OltanS/UR5e_Touch_haptic_teleoperation) | Teleoperation UR5e + Touch in ROS 1 (architectural reference). |
| [constanton/TouchP2P](https://github.com/constanton/TouchP2P) | Position-position teleoperation between two Touch devices (reference). |
| [jhu-saw/sawSensablePhantom](https://github.com/jhu-saw/sawSensablePhantom) | JHU driver with ROS 2 support (consulted). |

All the code in `ros2/` and `tools/` was written for this project (MIT License).

## Proprietary software (not included)

- 3D Systems **OpenHaptics** 3.4 Developer Edition for Linux.
- 3D Systems **Touch Device Driver** 2025.12.10 (and 2024.09.19 for comparison).

Both are downloaded from the official 3D Systems support site and are not redistributed.

## Libraries

- [PyUSB](https://github.com/pyusb/pyusb) (`python3-usb`), [NumPy](https://numpy.org/).
- Linux `usbmon` for USB traffic capture.

## Papers and material

- Y. Yang, D. Constantinescu, Y. Shi, "Input-to-State Stable Bilateral Teleoperation by
  Dynamic Interconnection and Damping Injection: Theory and Experiments", IEEE
  Transactions on Industrial Electronics (position-position P+d controllers with two
  Touch devices).
- Material indicated by the supervisor: DOI
  [10.1007/978-3-030-23807-0_26](https://doi.org/10.1007/978-3-030-23807-0_26).
- 3D Systems Touch datasheet (force and workspace specifications).
