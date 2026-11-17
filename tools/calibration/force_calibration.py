#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Part of haptic-teleoperation-touch-franka - Copyright (c) 2026 Nicola Gaiotto
"""
force_calibration.py - command -> newton calibration of the Touch force mode, with a scale.

The stylus tip rests on a kitchen scale. touch_raw_driver runs in 'force' mode and this
script asks it for downward forces of known size (in the driver's units, i.e. with the
current force_gain). For each step you type the scale reading in grams; the script fits

    F_measured = k * F_commanded       (F_measured = grams / 1000 * 9.81)

and prints the corrected gain: force_gain_new = force_gain_old / k.

Before running (other terminals, system Python, Humble sourced):
    ros2 run touch_raw_driver touch_raw_driver --ros-args --params-file <yaml> \
        -p max_force:=1.5
    ros2 topic pub --once -w 1 /touch/enable std_msgs/msg/Bool "{data: true}"
    ros2 topic pub --once -w 1 /touch/mode std_msgs/msg/String "{data: force}"
Put the tip on the scale (stylus roughly vertical), let go, TARE the scale, then run:
    python3 force_calibration.py [--steps 0.25 0.5 0.75 1.0 1.25] [--out results.csv]
Stopping (Ctrl+C) sends zero force; the driver also returns to zero after force_timeout.
"""
import argparse
import csv
import threading
import time

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node

G = 9.81


class ForcePublisher(Node):

    def __init__(self):
        super().__init__('force_calibration')
        self._pub = self.create_publisher(Vector3Stamped, '/touch/force', 10)
        self.force_z = 0.0
        self.create_timer(0.02, self._tick)          # 50 Hz, well inside force_timeout

    def _tick(self):
        m = Vector3Stamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'touch_base'
        m.vector.z = float(self.force_z)
        self._pub.publish(m)


def fit_through_origin(x, y):
    """Least-squares slope k of y = k x and RMS residual."""
    sxx = sum(a * a for a in x)
    k = sum(a * b for a, b in zip(x, y)) / sxx
    rms = (sum((b - k * a) ** 2 for a, b in zip(x, y)) / len(x)) ** 0.5
    return k, rms


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--steps', type=float, nargs='+', default=[0.25, 0.5, 0.75, 1.0, 1.25],
                    help='downward forces to request [N, driver units]')
    ap.add_argument('--repeat', type=int, default=2, help='passes over the steps')
    ap.add_argument('--out', default='force_calibration.csv')
    args = ap.parse_args()

    rclpy.init()
    node = ForcePublisher()
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    rows = []
    try:
        print('Tip on the scale, scale TARED with zero force. Ctrl+C to stop.')
        for rep in range(args.repeat):
            for f in args.steps:
                node.force_z = -f                    # downward, touch_base z is up
                time.sleep(1.0)
                txt = input(f'[pass {rep + 1}] requested {f:.2f} N down -> scale reading [g]: ')
                grams = float(txt.replace(',', '.'))
                rows.append((f, grams, grams / 1000.0 * G))
                node.force_z = 0.0
                time.sleep(0.5)
    except KeyboardInterrupt:
        print()
    finally:
        node.force_z = 0.0
        time.sleep(0.2)
        node.destroy_node()
        rclpy.try_shutdown()

    if len(rows) < 2:
        print('Not enough points.')
        return
    with open(args.out, 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['requested_N', 'scale_g', 'measured_N'])
        w.writerows(rows)
    k, rms = fit_through_origin([r[0] for r in rows], [r[2] for r in rows])
    print(f'\n{len(rows)} points saved to {args.out}')
    print(f'measured = {k:.3f} x requested   (RMS residual {rms:.3f} N)')
    print(f'-> multiply force_gain by {1.0 / k:.3f}')


if __name__ == '__main__':
    main()
