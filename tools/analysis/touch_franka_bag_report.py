#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Part of haptic-teleoperation-touch-franka - Copyright (c) 2026 Nicola Gaiotto
"""Tracking report of a Touch -> Franka recording (rosbag2, sqlite3 storage).

Reads the bag directly (no ROS installation needed) and compares the FR3 end effector
(/franka/pose) with the target computed from the Touch (/franka/target):
  - rate of each topic;
  - offset target - stylus (constant if the mapping is exact, scale 1);
  - lag per axis (shift that best aligns the robot with the target);
  - error to the target clipped to the workspace box (the box is part of the design);
  - robot and stylus speed;
  - optional plot.

Record in the lab with:
    ros2 bag record -o <name> /touch/pose /franka/pose /franka/target \
        /touch_to_franka/status /franka/status
Then:
    python3 tools/analysis/touch_franka_bag_report.py <name>/<name>_0.db3 \
        --centre 0.307 0.0 0.487 --half 0.10 --plot out.png --title "max 5 cm/s"
`--centre` is the robot position when it was enabled (centre of the workspace box).
"""
import argparse
import sqlite3
import struct

import numpy as np


def _pose(blob):
    b = blob[4:]                                   # skip the CDR encapsulation header
    length = struct.unpack_from('<I', b, 8)[0]     # header.frame_id length
    off = (12 + length + 7) // 8 * 8               # doubles are 8-byte aligned
    return struct.unpack_from('<3d', b, off)       # position x, y, z


def _point(blob):
    return struct.unpack_from('<3d', blob, 4)


def read_bag(path):
    db = sqlite3.connect(path)
    topics = dict(db.execute('SELECT id, name FROM topics'))
    out = {'/touch/pose': [], '/franka/pose': [], '/franka/target': []}
    for tid, stamp, blob in db.execute(
            'SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp'):
        name = topics[tid]
        if name in ('/touch/pose', '/franka/pose'):
            out[name].append((stamp * 1e-9, *_pose(blob)))
        elif name == '/franka/target':
            out[name].append((stamp * 1e-9, *_point(blob)))
    return {k: np.array(v) for k, v in out.items()}


def resample(src, t):
    return np.column_stack([np.interp(t, src[:, 0], src[:, k]) for k in (1, 2, 3)])


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('bag', help='path of the .db3 file')
    ap.add_argument('--centre', type=float, nargs=3, required=True,
                    help='workspace box centre [m] (robot position at enable time)')
    ap.add_argument('--half', type=float, default=0.10, help='box half size [m]')
    ap.add_argument('--plot', help='save a plot (PNG) to this path')
    ap.add_argument('--title', default='', help='text added to the plot title')
    a = ap.parse_args()

    d = read_bag(a.bag)
    T, F, G = d['/touch/pose'], d['/franka/pose'], d['/franka/target']
    c = np.array(a.centre)

    print(f'Duration {F[-1, 0] - F[0, 0]:.1f} s')
    for name, arr in (('/touch/pose', T), ('/franka/pose', F), ('/franka/target', G)):
        rate = (len(arr) - 1) / (arr[-1, 0] - arr[0, 0])
        print(f'{name:16s} {len(arr):5d} msgs, {rate:6.1f} Hz')

    off = G[:, 1:] - resample(T, G[:, 0])
    print('Offset target - stylus: std ' +
          ', '.join(f'{v * 1000:.2f}' for v in off.std(0)) + ' mm (x, y, z)')

    target = np.clip(resample(G, F[:, 0]), c - a.half, c + a.half)
    err = np.linalg.norm(F[:, 1:] - target, axis=1) * 1000
    print(f'Error to the clipped target: mean {err.mean():.1f} mm, '
          f'p95 {np.percentile(err, 95):.1f} mm, max {err.max():.1f} mm')

    dt = 0.005
    t = np.arange(max(F[0, 0], G[0, 0]), min(F[-1, 0], G[-1, 0]), dt)
    for k, axis in zip((1, 2, 3), 'xyz'):
        g = np.clip(np.interp(t, G[:, 0], G[:, k]), c[k - 1] - a.half, c[k - 1] + a.half)
        f = np.interp(t, F[:, 0], F[:, k])
        if np.ptp(g) < 0.01:
            print(f'{axis}: target range < 1 cm, lag not computed')
            continue
        lag = min(range(0, int(3.0 / dt)),
                  key=lambda n: np.mean((f[n:] - g[:len(g) - n]) ** 2))
        print(f'{axis}: target range {np.ptp(g) * 100:.1f} cm, lag {lag * dt:.2f} s')

    v_robot = np.linalg.norm(np.diff(F[:, 1:], axis=0), axis=1) / np.diff(F[:, 0])
    v_touch = (np.linalg.norm(np.diff(T[:, 1:], axis=0), axis=1) /
               np.maximum(np.diff(T[:, 0]), 1e-3))
    print(f'Robot speed max {v_robot.max() * 100:.1f} cm/s; '
          f'stylus speed p95 {np.percentile(v_touch, 95) * 100:.1f} cm/s')

    if a.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        t0 = G[0, 0]
        fig, axes = plt.subplots(3, 1, figsize=(8, 7), sharex=True)
        labels = ('x (operator right) [cm]', 'y (away) [cm]', 'z (up) [cm]')
        for k, ax, label in zip((1, 2, 3), axes, labels):
            ax.plot(G[:, 0] - t0, (G[:, k] - c[k - 1]) * 100, color='#888', lw=1.2,
                    label='target from Touch')
            ax.plot(F[:, 0] - t0, (F[:, k] - c[k - 1]) * 100, color='#1f5fbf', lw=1.6,
                    label='FR3 end effector')
            for s in (-1, 1):
                ax.axhline(s * a.half * 100, ls=':', color='#c33', lw=1)
            ax.set_ylabel(label)
            ax.grid(alpha=.3)
        axes[0].legend(loc='upper right', fontsize=8)
        axes[2].set_xlabel('time [s]')
        fig.suptitle(f'Touch -> FR3 position teleoperation ({a.title}, '
                     f'box ±{a.half * 100:.0f} cm dotted)', fontsize=10)
        fig.tight_layout()
        fig.savefig(a.plot, dpi=150)
        print(f'Plot saved to {a.plot}')


if __name__ == '__main__':
    main()
