# Deliverable 1 — Reading the Touch pose in ROS 2

**Goal:** see the pose of the stylus in ROS 2 while moving it by hand.

**Status:** ✅ done (shown to the supervisor).

## Approach

The third-party driver [tstoyanov/Geomagic_Touch_ROS2](https://github.com/tstoyanov/Geomagic_Touch_ROS2)
(packages `omni_common`, `omni_description`, `omni_msgs`) was used. It is a ROS 2 port
of the ROS 1 driver by bharatm11, itself derived from the `phantom_omni` driver, and it
reads the device through OpenHaptics. The driver is **not copied** into this repository:
[third_party/geomagic_touch_ros2](../third_party/geomagic_touch_ros2/README.md) explains
how to get it at the tested commit and apply my patch.

## Bug fixed: publishing rate

In `omni_common/src/omni_state.cpp` the publishing timer was created with

```cpp
std::chrono::seconds(1 / publish_rate)
```

`1 / publish_rate` is an integer division, equal to 0, so the timer period was 0 s and
the node published as fast as possible (about **18 000 Hz** measured). Fixed to

```cpp
std::chrono::microseconds(1000000 / publish_rate)
```

which gives the intended **~1000 Hz**. The fix is on the branch `fix/timer-period` of my
local clone and is provided as a patch.

## Running it

```bash
# Terminal 1 (stylus in the inkwell at start-up)
source ~/geomagic_ws/install/setup.bash
ros2 launch omni_common omni_state.launch.py

# Terminal 2
source ~/geomagic_ws/install/setup.bash
ros2 topic echo /phantom/pose --field pose.position
ros2 topic hz /phantom/pose        # ~1000 Hz
```

A readable 10 Hz monitor (position in mm, roll/pitch/yaw) is in
[tools/monitoring](../tools/monitoring/README.md).

## Verified experimentally

| Property | Result |
|---|---|
| Units of `/phantom/pose` | metres |
| Axes (seen from the operator) | x right, y forward (towards the device), z up |
| Origin | OpenHaptics origin, roughly the centre of the workspace |
| Stylus in the inkwell | (0.000, 0.0881, −0.0655) m |
| Publishing rate | ~1000 Hz after the fix |

Axis conversion in the driver code: `x_ros = x_hd`, `y_ros = −z_hd`, `z_ros = y_hd`.

## Known limitations of the third-party driver (from reading its code)

- The orientation in `/phantom/pose` is not reliable: the rotation applied is not
  consistent with the axis change of the position.
- `/phantom/state` reports the position in millimetres although the message says metres.
- `frame_id` is `"/world"`, with a leading slash that tf2 in ROS 2 does not accept.
- If the haptic loop stops on an error, the node keeps republishing the last value with
  new timestamps.
- On this system it **cannot produce forces** (see Deliverable 2).
