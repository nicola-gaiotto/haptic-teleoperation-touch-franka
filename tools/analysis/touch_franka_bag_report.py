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
Then (one or more bags; with several, the plot has one column per bag):
    python3 tools/analysis/touch_franka_bag_report.py a/a_0.db3 b/b_0.db3 \
        --centre 0.307 0.0 0.487 --labels "max 2 cm/s" "max 5 cm/s" --plot out.pdf
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


def report(path, c, half):
    """Print the numbers of one recording and return its arrays."""
    d = read_bag(path)
    T, F, G = d['/touch/pose'], d['/franka/pose'], d['/franka/target']

    print(f'== {path}')
    print(f'Duration {F[-1, 0] - F[0, 0]:.1f} s')
    for name, arr in (('/touch/pose', T), ('/franka/pose', F), ('/franka/target', G)):
        rate = (len(arr) - 1) / (arr[-1, 0] - arr[0, 0])
        print(f'{name:16s} {len(arr):5d} msgs, {rate:6.1f} Hz')

    off = G[:, 1:] - resample(T, G[:, 0])
    print('Offset target - stylus: std ' +
          ', '.join(f'{v * 1000:.2f}' for v in off.std(0)) + ' mm (x, y, z)')

    target = np.clip(resample(G, F[:, 0]), c - half, c + half)
    err = np.linalg.norm(F[:, 1:] - target, axis=1)
    print(f'Error to the clipped target: mean {err.mean() * 1000:.1f} mm, '
          f'p95 {np.percentile(err, 95) * 1000:.1f} mm, max {err.max() * 1000:.1f} mm')

    dt = 0.005
    t = np.arange(max(F[0, 0], G[0, 0]), min(F[-1, 0], G[-1, 0]), dt)
    for k, axis in zip((1, 2, 3), 'xyz'):
        g = np.clip(np.interp(t, G[:, 0], G[:, k]), c[k - 1] - half, c[k - 1] + half)
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
    return F, G, err


def plot(runs, labels, c, half, path):
    """One column per recording: x, y, z (target and robot) and the tracking error."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch

    # Colours: categorical blue / orange (colour-blind safe pair), recessive greys.
    ink, muted, grid = '#262624', '#73726c', '#e6e5e0'
    robot_c, target_c, out_c, box_c = '#2a78d6', '#eb6834', '#f2f1ec', '#a3a29b'
    plt.rcParams.update({
        'font.size': 10.5, 'axes.titlesize': 12, 'axes.labelsize': 10.5,
        'axes.edgecolor': box_c, 'axes.labelcolor': ink, 'axes.linewidth': 0.8,
        'xtick.color': muted, 'ytick.color': muted, 'xtick.major.size': 0,
        'ytick.major.size': 0, 'axes.spines.top': False, 'axes.spines.right': False,
        'figure.facecolor': 'white', 'axes.facecolor': 'white'})

    n = len(runs)
    fig, axes = plt.subplots(4, n, figsize=(5.6 * n, 9.6), sharey='row', sharex=True,
                             squeeze=False,
                             gridspec_kw={'height_ratios': [1, 1, 1.25, 0.9],
                                          'hspace': 0.18, 'wspace': 0.06})
    names = ('x, operator right', 'y, away', 'z, up')
    ylims = ((-18, 18), (-18, 18), (-14, 27))
    lim = half * 100
    tmax = max(F[-1, 0] - G[0, 0] for F, G, _ in runs)
    for j, ((F, G, err), label) in enumerate(zip(runs, labels)):
        t0 = G[0, 0]
        for i, k in enumerate((1, 2, 3)):
            ax = axes[i, j]
            lo, hi = ylims[i]
            ax.axhspan(lim, hi, color=out_c, lw=0, zorder=0)
            ax.axhspan(lo, -lim, color=out_c, lw=0, zorder=0)
            for y in (-lim, lim):
                ax.axhline(y, color=box_c, lw=1, zorder=1)
            ax.plot(G[:, 0] - t0, (G[:, k] - c[k - 1]) * 100, color=target_c, lw=1.6,
                    zorder=2, solid_capstyle='round')
            ax.plot(F[:, 0] - t0, (F[:, k] - c[k - 1]) * 100, color=robot_c, lw=2.4,
                    zorder=3, solid_capstyle='round')
            ax.set_ylim(lo, hi)
            ax.set_yticks([-10, 0, 10] + ([20] if hi > 20 else []))
            ax.grid(axis='y', color=grid, lw=0.8, zorder=0)
            if j == 0:
                ax.set_ylabel(f'{names[i]} [cm]')
            if i == 0:
                ax.set_title(label, color=ink, fontweight='bold', loc='left', pad=8)
        ax = axes[3, j]
        e = err * 1000
        ax.fill_between(F[:, 0] - t0, 0, e, color=robot_c, alpha=0.15, lw=0)
        ax.plot(F[:, 0] - t0, e, color=robot_c, lw=1.8)
        mean = e.mean()
        ax.axhline(mean, color=ink, lw=1, ls=(0, (4, 3)))
        ax.text(0.985, 0.95, f'mean {mean:.1f} mm', transform=ax.transAxes, color=ink,
                fontsize=10, ha='right', va='top',
                bbox=dict(boxstyle='round,pad=0.25', fc='white', ec='none'))
        ax.set_ylim(0, None)
        ax.grid(axis='y', color=grid, lw=0.8, zorder=0)
        ax.set_xlabel('time [s]')
        if j == 0:
            ax.set_ylabel('tracking error [mm]')
        ax.set_xlim(0, tmax)

    handles = [Line2D([], [], color=target_c, lw=1.6, label='target from the Touch'),
               Line2D([], [], color=robot_c, lw=2.4, label='FR3 end effector'),
               Patch(fc=out_c, ec=box_c, lw=1, label=f'outside the ±{lim:.0f} cm box')]
    fig.legend(handles=handles, loc='upper center', ncol=3, frameon=False,
               bbox_to_anchor=(0.5, 0.995), handlelength=2.2, columnspacing=2.0)
    fig.subplots_adjust(left=0.07, right=0.985, top=0.915, bottom=0.06)
    fig.savefig(path, dpi=200)
    print(f'Plot saved to {path}')


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('bags', nargs='+', help='path of each .db3 file')
    ap.add_argument('--centre', type=float, nargs=3, required=True,
                    help='workspace box centre [m] (robot position at enable time)')
    ap.add_argument('--half', type=float, default=0.10, help='box half size [m]')
    ap.add_argument('--labels', nargs='+', help='one title per bag for the plot')
    ap.add_argument('--plot', help='save a plot to this path (.png or .pdf); '
                    'with several bags, one column per bag')
    a = ap.parse_args()
    c = np.array(a.centre)
    runs = [report(path, c, a.half) for path in a.bags]
    if a.plot:
        labels = a.labels or a.bags
        if len(labels) != len(runs):
            ap.error('give one label per bag')
        plot(runs, labels, c, a.half, a.plot)


if __name__ == '__main__':
    main()
