# Analysis tools

| Script | Purpose |
|---|---|
| `teleop_tracking_report.py [seconds]` | Records target, stylus pose and motor commands during keyboard teleoperation; prints lag, off-axis error (e.g. drop in z), settling time and motor saturation for each motion; saves `~/touch_tracking.png`. |

Run it while `touch_teleop` is running with the motors ON (mode position). Requires
`python3-matplotlib` for the plot.
