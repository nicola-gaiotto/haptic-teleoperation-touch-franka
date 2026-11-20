#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Part of haptic-teleoperation-touch-franka - Copyright (c) 2026 Nicola Gaiotto
"""
force_calibration.py - command -> newton calibration of the Touch force mode, with a scale.

The stylus lies flat on a kitchen scale, under the gimbal (the point of /touch/pose), so the
arm pushes down through it. touch_raw_driver runs in 'force' mode and this
script asks it for downward forces of known size (in the driver's units, i.e. with the
current force_gain). For each step you type the scale reading in grams; the script fits

    F_measured = k * F_commanded       (F_measured = grams / 1000 * 9.81)

and prints the corrected gain: force_gain_new = force_gain_old / k.

Joint friction (about 170-190 command units) holds part of the load while the arm is still, so
the forces are applied going UP and then DOWN (default sequence): on the way up the scale reads
about k F - f, on the way down about k F + f. The mean of the two readings at the same force
cancels the friction f (lab, 2026-11-20: with up-only steps the scale read 0 g up to 0.75 N).

Before running (other terminals, system Python, Humble sourced):
    ros2 run touch_raw_driver touch_raw_driver --ros-args --params-file <yaml> \
        -p max_force:=3.0
    ros2 topic pub --once -w 1 /touch/enable std_msgs/msg/Bool "{data: true}"
    ros2 topic pub --once -w 1 /touch/mode std_msgs/msg/String "{data: force}"
Lay the stylus flat on the scale under the gimbal, let go, TARE the scale, then run:
    python3 force_calibration.py [--out results.csv]
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


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--steps', type=float, nargs='+',
                    default=[0.5, 1.0, 1.5, 2.0, 2.5, 2.0, 1.5, 1.0, 0.5],
                    help='downward forces to request [N, driver units], up then down; the '
                         'force is NOT set to zero between steps')
    ap.add_argument('--repeat', type=int, default=2, help='passes over the steps')
    ap.add_argument('--out', default='force_calibration.csv')
    args = ap.parse_args()

    rclpy.init()
    node = ForcePublisher()
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    rows = []
    try:
        print('Stylus flat on the scale, scale TARED with zero force. Ctrl+C to stop.')
        for rep in range(args.repeat):
            for f in args.steps:
                node.force_z = -f                    # downward, touch_base z is up
                time.sleep(1.0)
                txt = input(f'[pass {rep + 1}] requested {f:.2f} N down, wait 2 s -> scale [g]: ')
                grams = float(txt.replace(',', '.'))
                rows.append((f, grams, grams / 1000.0 * G))
            node.force_z = 0.0
            time.sleep(1.0)
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
    print(f'\n{len(rows)} points saved to {args.out}')
    # Mean of all readings at each requested force (up and down): friction cancels.
    forces = sorted({r[0] for r in rows})
    means = [sum(r[2] for r in rows if r[0] == f) / sum(1 for r in rows if r[0] == f)
             for f in forces]
    for f, m in zip(forces, means):
        print(f'  requested {f:.2f} N -> measured (mean up/down) {m:.3f} N')
    n = len(forces)
    mx, my = sum(forces) / n, sum(means) / n
    sxx = sum((x - mx) ** 2 for x in forces)
    k = sum((x - mx) * (y - my) for x, y in zip(forces, means)) / sxx
    b = my - k * mx
    rms = (sum((y - (k * x + b)) ** 2 for x, y in zip(forces, means)) / n) ** 0.5
    print(f'measured = {k:.3f} x requested {b:+.3f} N   (RMS residual {rms:.3f} N)')
    print(f'-> multiply force_gain by {1.0 / k:.3f}')


if __name__ == '__main__':
    main()
