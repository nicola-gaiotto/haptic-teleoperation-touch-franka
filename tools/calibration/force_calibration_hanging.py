#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Part of haptic-teleoperation-touch-franka - Copyright (c) 2026 Nicola Gaiotto
"""
force_calibration_hanging.py - command -> newton calibration of the Touch force mode with a
hanging weight of known mass (no scale needed; the mass can be measured with
weigh_with_franka.py).

Idea (friction cancels out):
  - UP ramp: the upward force grows slowly until the stylus starts rising:
        F_up   = W + friction + residual            (W = weight in driver units)
  - the stylus is pushed up until it stops (top stop), then the DOWN ramp decreases the force
    slowly until the stylus starts falling:
        F_down = W - friction + residual
  - mean = W + residual. "residual" (uncompensated arm gravity) is removed with the same
    test WITHOUT the weight (--no-mass), so  W_driver = mean_with - mean_without.
  - The rendered force is k times the requested one, with k = m g / W_driver:
        force_gain_new = force_gain_old / k

Setup: touch_raw_driver in 'force' mode with max_force 3.0. Stylus tip resting on a support
(e.g. a book) near the bottom of the workspace, weight hanging from the tip with a string,
in the air. Run first with --no-mass (string removed), then with the weight:
    python3 force_calibration_hanging.py --no-mass
    python3 force_calibration_hanging.py --mass-g 150
"""
import argparse
import statistics
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from rclpy.node import Node

G = 9.81


class Ramp(Node):

    def __init__(self):
        super().__init__('force_calibration_hanging')
        self._pub = self.create_publisher(Vector3Stamped, '/touch/force', 10)
        self.create_subscription(PoseStamped, '/touch/pose', self._on_pose, 10)
        self.force_z = 0.0
        self.z = None
        self.create_timer(0.01, self._tick)

    def _on_pose(self, msg):
        self.z = msg.pose.position.z

    def _tick(self):
        m = Vector3Stamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'touch_base'
        m.vector.z = float(self.force_z)
        self._pub.publish(m)


def wait_pose(node):
    while node.z is None:
        time.sleep(0.05)


def ramp_until(node, start, rate, f_limit, moved):
    """Change force_z from `start` at `rate` N/s until moved(dz) or |force| > f_limit."""
    z0 = node.z
    f = start
    while abs(f) <= f_limit:
        node.force_z = f
        time.sleep(0.01)
        if moved(node.z - z0):
            return f
        f += rate * 0.01
    return None


def one_trial(node, rate, f_max, thr):
    wait_pose(node)
    node.force_z = 0.0
    time.sleep(0.5)
    f_up = ramp_until(node, 0.0, rate, f_max, lambda dz: dz > thr)
    if f_up is None:
        return None
    # Keep pushing a little more, so the stylus rises until it stops (mechanical top stop or
    # balance), then ramp down from there until it starts falling.
    node.force_z = f_up + 0.1
    t_end = time.monotonic() + 8.0
    last = node.z
    still = 0.0
    while time.monotonic() < t_end and still < 0.5:
        time.sleep(0.05)
        still = still + 0.05 if abs(node.z - last) < 0.0002 else 0.0
        last = node.z
    f_down = ramp_until(node, f_up + 0.1, -rate, f_max, lambda dz: dz < -thr)
    node.force_z = 0.0
    if f_down is None:
        return None
    return f_up, f_down


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--mass-g', type=float, help='mass of the hanging weight [g]')
    ap.add_argument('--no-mass', action='store_true', help='baseline run, no weight')
    ap.add_argument('--baseline', type=float, help='mean force of the --no-mass run [N]')
    ap.add_argument('--repeat', type=int, default=5)
    ap.add_argument('--rate', type=float, default=0.2, help='ramp rate [N/s]')
    ap.add_argument('--max', type=float, default=2.9, help='largest force requested [N]')
    ap.add_argument('--threshold-mm', type=float, default=3.0, help='motion detected [mm]')
    args = ap.parse_args()

    rclpy.init()
    node = Ramp()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    means = []
    try:
        for i in range(args.repeat):
            input(f'[{i + 1}/{args.repeat}] Stylus tip on the support, hands off. Enter...')
            r = one_trial(node, args.rate, args.max, args.threshold_mm / 1000.0)
            if r is None:
                print('  no motion detected within the force limit: skipped')
                continue
            f_up, f_down = r
            means.append(0.5 * (f_up + f_down))
            print(f'  rises at {f_up:.3f} N, falls at {f_down:.3f} N -> mean {means[-1]:.3f} N,'
                  f' friction {0.5 * (f_up - f_down):.3f} N')
    except KeyboardInterrupt:
        print()
    finally:
        node.force_z = 0.0
        time.sleep(0.2)
        node.destroy_node()
        rclpy.try_shutdown()

    if not means:
        return
    mean = statistics.mean(means)
    spread = max(means) - min(means)
    print(f'\nmean of {len(means)} trials: {mean:.3f} N (spread {spread:.3f} N)')
    if args.no_mass:
        print(f'Baseline. Now run with: --mass-g <grams> --baseline {mean:.3f}')
    elif args.mass_g is not None and args.baseline is not None:
        w_driver = mean - args.baseline
        w_true = args.mass_g / 1000.0 * G
        k = w_true / w_driver
        print(f'weight: driver units {w_driver:.3f} N, true {w_true:.3f} N -> k = {k:.3f}')
        print(f'-> multiply force_gain by {1.0 / k:.3f}')


if __name__ == '__main__':
    main()
