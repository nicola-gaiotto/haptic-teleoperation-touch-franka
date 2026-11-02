# franka_teleop

Velocity teleoperation of the Franka Research 3 through [franky](https://github.com/TimSchneider42/franky).
The keyboard (`teleop_twist_keyboard`) publishes `geometry_msgs/Twist` on `/franka/cmd_vel`;
the node turns each command into a short Cartesian velocity motion (linear only, orientation fixed).

## Safety behaviour
- Starts **disabled**: nothing moves until `true` is published on `/franka/enable`.
- Speed limited to `max_speed` (default 2 cm/s); franky dynamics factor 5 %.
- Workspace box of ±10 cm around the pose at enable time (outward motion blocked).
- Each command is a 150 ms motion: if commands stop, the robot stops by itself.
- Any robot error disables the node; recovery only through `/franka/recover`.

## Install
franky is installed with pip in a virtual environment that also sees the ROS 2 packages:
```bash
python3 -m venv --system-site-packages ~/franky_venv
source ~/franky_venv/bin/activate
pip install franky-control franky-sim      # franky-sim only for simulation
```
On the robot PC franky also needs a real-time kernel and the `realtime` group
(see the franky installation guide) and the wheel matching the robot server version.

Build **with the venv activated and through its Python**, so that the node's script runs
with the venv interpreter (otherwise: `ModuleNotFoundError: No module named 'franky'`):
```bash
source ~/franky_venv/bin/activate
cd ~/geomagic_ws && python3 -m colcon build --symlink-install --packages-select franka_teleop
head -1 install/franka_teleop/lib/franka_teleop/franka_teleop   # -> #!.../franky_venv/bin/python3
```

## Run
Terminal 1 (node):
```bash
source ~/franky_venv/bin/activate
source /opt/ros/jazzy/setup.bash && source ~/geomagic_ws/install/setup.bash
ros2 launch franka_teleop franka_teleop.launch.py simulate:=true        # franky-sim
ros2 launch franka_teleop franka_teleop.launch.py robot_ip:=<FCI IP>    # real robot
```
Terminal 2 (enable, keyboard):
```bash
source ~/geomagic_ws/install/setup.bash
ros2 topic pub --once -w 1 /franka/enable std_msgs/msg/Bool "{data: true}"
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/franka/cmd_vel
```
Keys (operator's view, rotated into the base frame by `operator_yaw_deg`, 90° in the lab):
`i`/`,` away/towards, `j`/`l` left/right, `t`/`b` up/down, `k` stop.

| Topic / service | Type | Content |
|---|---|---|
| `/franka/cmd_vel` | `geometry_msgs/Twist` (in) | velocity command, operator frame |
| `/franka/enable` | `std_msgs/Bool` (in) | allow motion (starts disabled) |
| `/franka/pose` | `geometry_msgs/PoseStamped` (out) | end-effector pose in `fr3_link0` |
| `/franka/status` | `std_msgs/String` (out) | `disabled`, `enabled` or `ERROR: ...` |
| `/franka/recover` | `std_srvs/Trigger` | clear robot errors (node must be disabled) |

Parameters: [config/franka_teleop.yaml](config/franka_teleop.yaml).
