# Analysis tools

| Script | Purpose |
|---|---|
| `teleop_tracking_report.py [seconds]` | Records target, stylus pose and motor commands during keyboard teleoperation; prints lag, off-axis error (e.g. drop in z), settling time and motor saturation for each motion; saves `~/touch_tracking.png`. |
| `touch_franka_bag_report.py <bag.db3> --centre x y z [--plot out.png]` | Reads a Touch -> Franka rosbag (sqlite3, no ROS needed); prints topic rates, target-stylus offset, lag per axis, error to the box-clipped target, robot and stylus speed; optional plot. |

Run it while `touch_teleop` is running with the motors ON (mode position). Requires
`python3-matplotlib` for the plot.
