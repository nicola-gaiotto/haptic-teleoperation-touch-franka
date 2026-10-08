# Geomagic_Touch_ROS2 (third party) — Deliverable 1 driver

The driver used for Deliverable 1 is
[tstoyanov/Geomagic_Touch_ROS2](https://github.com/tstoyanov/Geomagic_Touch_ROS2).
It is **not copied** into this repository: clone it from its authors and apply my patch.

## Tested version

- Commit `7a13541` of `tstoyanov/Geomagic_Touch_ROS2`.
- Patch: [`patches/0001-fix-publish-timer-period.patch`](patches/) — fixes the
  publishing timer of `omni_common/src/omni_state.cpp`
  (`std::chrono::seconds(1/publish_rate)` is an integer division equal to 0 →
  ~18 kHz; replaced by `std::chrono::microseconds(1000000 / publish_rate)` → ~1 kHz).

## Install

```bash
cd ~/geomagic_ws/src
git clone https://github.com/tstoyanov/Geomagic_Touch_ROS2.git
cd Geomagic_Touch_ROS2
git checkout 7a13541
git checkout -b fix/timer-period
git apply ~/geomagic_ws/src/haptic-teleoperation-touch-franka/third_party/geomagic_touch_ros2/patches/0001-fix-publish-timer-period.patch
cd ~/geomagic_ws
colcon build --symlink-install --packages-select omni_msgs omni_description omni_common
```

Requires OpenHaptics and the Touch Device Driver ([docs/00_setup.md](../../docs/00_setup.md)).

## License

The driver and the context lines contained in the patch belong to their authors and are
subject to the license of the original repository, not to the MIT license of this
project.
