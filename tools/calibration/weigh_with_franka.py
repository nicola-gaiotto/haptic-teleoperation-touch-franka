#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Part of haptic-teleoperation-touch-franka - Copyright (c) 2026 Nicola Gaiotto
"""
weigh_with_franka.py - use the FR3 external-force estimate as a scale (no motion).

The robot stays still (no motion is commanded). The script averages O_F_ext_hat_K z for a few
seconds without the object, then asks you to hang the object on the gripper (e.g. with a string
loop over a finger) and averages again. Hanging a weight pulls the flange down, which gives a
positive change of Fz (lab, 2026-11-17: pushing the end effector down gave Fz up to +9 N), so

    m = (Fz_with - Fz_without) / g

Run in the conda environment with franky:
    conda activate franky_env
    python weigh_with_franka.py --ip 172.16.0.2 [--seconds 3] [--repeat 3]
"""
import argparse
import statistics
import time

import franky

G = 9.81


def mean_fz(robot, seconds, rate=100.0):
    vals = []
    t_end = time.monotonic() + seconds
    while time.monotonic() < t_end:
        vals.append(float(robot.state.O_F_ext_hat_K[2]))
        time.sleep(1.0 / rate)
    return statistics.mean(vals), statistics.pstdev(vals)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--ip', default='172.16.0.2')
    ap.add_argument('--seconds', type=float, default=3.0)
    ap.add_argument('--repeat', type=int, default=3)
    args = ap.parse_args()
    robot = franky.Robot(args.ip)
    masses = []
    for i in range(args.repeat):
        input(f'[{i + 1}/{args.repeat}] Object OFF the robot, hands off. Press Enter...')
        f0, s0 = mean_fz(robot, args.seconds)
        input('Hang the object on the gripper, let it stop swinging, hands off. Press Enter...')
        f1, s1 = mean_fz(robot, args.seconds)
        m = (f1 - f0) / G
        masses.append(m)
        print(f'  Fz without {f0:+.3f} N (std {s0:.3f}), with {f1:+.3f} N (std {s1:.3f})'
              f' -> {m * 1000:.0f} g')
    print(f'\nmass = {statistics.mean(masses) * 1000:.0f} g '
          f'(spread {(max(masses) - min(masses)) * 1000:.0f} g over {len(masses)} tries)')


if __name__ == '__main__':
    main()
