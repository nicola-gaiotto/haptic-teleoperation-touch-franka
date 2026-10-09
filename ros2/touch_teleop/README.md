# touch_teleop

Velocity (keyboard) teleoperation of the 3D Systems Touch on top of `touch_raw_driver`.

- Node `twist_to_target`: `geometry_msgs/Twist` on `/touch/cmd_vel` → moving Cartesian
  target on `/touch/target`, limited to a workspace box and to 2 cm ahead of the stylus.
- Node `touch_dashboard`: 2D GUI (top and front views, position, motors/mode buttons) with
  arrow-key teleoperation (motion lasts exactly while the key is held).
- `launch/teleop.launch.py`: driver + `twist_to_target` + `world → touch_base` transform +
  dashboard (`dashboard:=false` to disable) + RViz (`rviz:=true` to enable).
- `rviz/touch_teleop.rviz`: stylus position, teleop target, workspace box.

```bash
sudo apt install ros-jazzy-teleop-twist-keyboard python3-tk python3-matplotlib
ros2 launch touch_teleop teleop.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/touch/cmd_vel
```

Details, key mapping and GUI tools: [docs/03_keyboard_teleoperation.md](../../docs/03_keyboard_teleoperation.md).
