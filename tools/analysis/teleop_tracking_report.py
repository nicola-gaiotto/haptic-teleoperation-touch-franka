#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Part of haptic-teleoperation-touch-franka - Copyright (c) 2026 Nicola Gaiotto
"""
teleop_tracking_report.py - measures how well the Touch follows the teleoperation target.

Records /touch/target, /touch/pose and /touch/motor_cmd for a few seconds while you move
the stylus with the keyboard, then prints, for every motion (key held):
  - direction, duration and speed of the target;
  - lag: how far behind the stylus is along the motion direction [mm] and in time [s];
  - deviation on the other axes [mm] (e.g. the drop in z while moving left/right);
  - settling: after the key is released, time to get within 2 mm of the target;
  - peak motor command and fraction of time a motor is saturated.
It also saves a plot (target dashed, stylus solid) to ~/touch_tracking.png.

Usage (touch_teleop running, motors ON, mode position):
  python3 teleop_tracking_report.py [seconds] [max_delta]     default 25 s, 600
  (max_delta = the driver's command limit; "saturated" means above 95 % of it)
Suggested motion: right 3 s, pause 2 s, left 3 s, pause 2 s, forward 3 s, pause, ...
"""
import os
import sys
import time

import numpy as np

AXES = ('x', 'y', 'z')
DIRECTION_NAMES = {('x', 1): 'RIGHT', ('x', -1): 'LEFT', ('y', 1): 'FORWARD',
                   ('y', -1): 'BACKWARD', ('z', 1): 'UP', ('z', -1): 'DOWN'}
SEGMENT_GAP = 0.3          # s without targets = end of a motion
SETTLE_TOL = 0.002         # m
SATURATION = 0.95 * 600    # command units (95 % of max_delta), set from the command line


def record(duration):
    import rclpy
    from geometry_msgs.msg import Point, PoseStamped
    from std_msgs.msg import Int32MultiArray

    rclpy.init()
    node = rclpy.create_node('teleop_tracking_report')
    pose, target, cmd = [], [], []
    t0 = time.monotonic()
    node.create_subscription(PoseStamped, '/touch/pose', lambda m: pose.append(
        (time.monotonic() - t0, m.pose.position.x, m.pose.position.y, m.pose.position.z)), 50)
    node.create_subscription(Point, '/touch/target', lambda m: target.append(
        (time.monotonic() - t0, m.x, m.y, m.z)), 50)
    node.create_subscription(Int32MultiArray, '/touch/motor_cmd', lambda m: cmd.append(
        (time.monotonic() - t0, *m.data[:3])), 50)
    print(f'Recording {duration:.0f} s: move the stylus with the keyboard now '
          '(e.g. right 3 s, pause, left 3 s, pause).')
    next_print = 0.0
    while time.monotonic() - t0 < duration:
        rclpy.spin_once(node, timeout_sec=0.05)
        t = time.monotonic() - t0
        if t >= next_print:
            print(f'  {duration - t:4.0f} s left', flush=True)
            next_print += 5.0
    node.destroy_node()
    rclpy.shutdown()
    return np.array(pose), np.array(target), np.array(cmd)


def segments(target):
    """Split the target stream into motions (target is published only while moving)."""
    if len(target) == 0:
        return []
    cuts = np.where(np.diff(target[:, 0]) > SEGMENT_GAP)[0]
    starts = np.concatenate([[0], cuts + 1])
    ends = np.concatenate([cuts, [len(target) - 1]])
    return [(s, e) for s, e in zip(starts, ends) if e - s >= 10]


def analyze(pose, target, cmd):
    if len(pose) < 10:
        print('No /touch/pose data: is touch_raw_driver running?')
        return
    segs = segments(target)
    if not segs:
        print('No motion recorded on /touch/target: did you move the stylus with the keys '
              '(motors ON, mode position)?')
        return
    tp = pose[:, 0]
    print('\n' + '=' * 78)
    for k, (s, e) in enumerate(segs, start=1):
        seg = target[s:e + 1]
        t = seg[:, 0]
        p = np.column_stack([np.interp(t, tp, pose[:, i]) for i in (1, 2, 3)])
        delta = seg[-1, 1:4] - seg[0, 1:4]
        a = int(np.argmax(np.abs(delta)))
        sign = 1 if delta[a] >= 0 else -1
        duration = t[-1] - t[0]
        speed = abs(delta[a]) / duration if duration > 0 else 0.0
        err = seg[:, 1:4] - p                      # target - stylus
        along = err[:, a] * sign                   # >0: stylus behind the target
        mid = slice(len(t) // 4, len(t))           # skip the start-up transient
        lag_mm = 1000 * float(np.mean(along[mid]))
        lag_s = (lag_mm / 1000) / speed if speed > 1e-6 else float('nan')
        others = [i for i in range(3) if i != a]
        dev = {AXES[i]: (1000 * float(np.mean(err[mid, i])), 1000 * float(np.max(np.abs(err[:, i]))))
               for i in others}

        # settling after the key release
        final = seg[-1, 1:4]
        after = (tp > t[-1]) & (tp < t[-1] + 3.0)
        settle = float('nan')
        if np.any(after):
            d = np.linalg.norm(pose[after, 1:4] - final, axis=1)
            inside = np.where(d < SETTLE_TOL)[0]
            if len(inside):
                settle = float(tp[after][inside[0]] - t[-1])

        name = DIRECTION_NAMES[(AXES[a], sign)]
        print(f'Motion {k}: {name:8s} {abs(delta[a]) * 1000:5.1f} mm in {duration:4.1f} s '
              f'({speed * 100:.1f} cm/s)')
        print(f'   lag along {AXES[a]}: {lag_mm:5.1f} mm  (~{lag_s:4.2f} s behind)')
        for ax, (mean_e, max_e) in dev.items():
            label = '  <-- stylus BELOW target' if ax == 'z' and mean_e > 1.0 else ''
            print(f'   error on {ax}: mean {mean_e:+5.1f} mm, max {max_e:4.1f} mm{label}')
        print('   settling after release: ' +
              (f'{settle:4.2f} s' if settle == settle else 'not within 2 mm in 3 s'))
        if len(cmd):
            in_seg = (cmd[:, 0] >= t[0]) & (cmd[:, 0] <= t[-1])
            if np.any(in_seg):
                c = np.abs(cmd[in_seg, 1:4])
                peaks = ', '.join(f'm{i} {int(c[:, i].max())}' for i in range(3))
                sat = ', '.join(f'm{i} {100 * np.mean(c[:, i] >= SATURATION):.0f}%'
                                for i in range(3))
                print(f'   peak motor command: {peaks} | saturated: {sat}')
        print('-' * 78)
    save_plot(pose, target)


def save_plot(pose, target):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print('matplotlib not installed: no plot (sudo apt install python3-matplotlib).')
        return
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    labels = ('x (right) [mm]', 'y (forward) [mm]', 'z (up) [mm]')
    for i, ax in enumerate(axs):
        ax.plot(pose[:, 0], pose[:, i + 1] * 1000, '-', color='tab:blue', label='stylus')
        if len(target):
            ax.plot(target[:, 0], target[:, i + 1] * 1000, '--', color='tab:orange',
                    marker='.', ms=2, lw=1, label='target')
        ax.set_ylabel(labels[i])
        ax.grid(True, alpha=0.3)
    axs[0].legend(loc='upper right')
    axs[-1].set_xlabel('time [s]')
    fig.suptitle('Touch teleoperation: target vs stylus')
    fig.tight_layout()
    out = os.path.expanduser('~/touch_tracking.png')
    fig.savefig(out, dpi=110)
    print(f'Plot saved to {out}')


def main():
    global SATURATION
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 25.0
    if len(sys.argv) > 2:
        SATURATION = 0.95 * float(sys.argv[2])
    pose, target, cmd = record(duration)
    analyze(pose, target, cmd)


if __name__ == '__main__':
    main()
