# touch_teleop

Velocity (keyboard) teleoperation of the 3D Systems Touch on top of `touch_raw_driver`.

- Node `twist_to_target`: `geometry_msgs/Twist` on `/touch/cmd_vel` → moving Cartesian
  target on `/touch/target`, limited to a workspace box and to 2 cm ahead of the stylus.
- Node `touch_dashboard`: control panel (position, motors/mode buttons, speed) with
  arrow-key teleoperation (motion lasts exactly while the key is held).
- Node `touch_model_bridge`: driver joint angles → joints of the `omni_description` model.
- `launch/teleop.launch.py`: driver + `twist_to_target` + control panel + 3D view
  (`robot_state_publisher`, frames `world → touch_base → touch/base`, RViz).
- `rviz/touch_teleop.rviz`: operator view of the Touch model, target and workspace.

```bash
sudo apt install ros-jazzy-teleop-twist-keyboard python3-tk ros-jazzy-xacro
ros2 launch touch_teleop teleop.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/touch/cmd_vel
```

Details, key mapping and GUI tools: [docs/03_keyboard_teleoperation.md](../../docs/03_keyboard_teleoperation.md).
