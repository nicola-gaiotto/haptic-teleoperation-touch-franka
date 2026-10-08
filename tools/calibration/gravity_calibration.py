#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Part of haptic-teleoperation-touch-franka - Copyright (c) 2026 Nicola Gaiotto
"""
gravity_calibration.py - physics-based gravity model for the 3D Systems Touch,
commanding the motors directly over USB (no OpenHaptics).

This version:
  - uses the JOINT ANGLES (from ~/.touch_raw_driver/kinematics.json);
  - fits a physical model without constant terms (gravity of rigid links). On the basis
        phi = [cos q1, sin q1, sin q2, cos q2]
    two variants are fitted for each motor:
        decoupled: motor 1 uses (cos q1, sin q1) only, motor 2 uses (sin q2, cos q2) only
        coupled:   each motor uses all four terms (covers a coupled transmission)
    The coupled variant is kept only if there are enough poses and it reduces the error
    by more than 25 %; otherwise the more robust decoupled variant is used.
  - chooses the poses among the configurations actually visited by hand in phase 1
    (pulled 15 % towards the centre). Joints 1 and 2 have COUPLED limits (the links
    cannot fold past each other), so the rectangular grid of v2 contained unreachable
    poses where a motor only pushed against a mechanical stop;
  - reaches every pose twice, once from each side, and averages the two holding
    commands, so that static friction (which acts against the last motion) cancels out.
Gravity does not depend on the base angle q0: the base is simply held still.

PROCEDURE (about 3-5 minutes):
  - put the Touch near the table edge, so the stylus hangs freely and touches nothing;
  - PHASE 1 (25 s, motors OFF): move the arm by hand through the whole workspace,
    slowly, combining up/down with near/far;
  - PHASE 2: after the countdown let go (hand close). The arm visits up to 16 poses.
  - Before saving, the model is evaluated over the whole visited workspace: if it
    predicts commands above a safe limit, it is NOT saved.
Output: ~/.touch_raw_driver/gravity.json  (used by touch_raw_driver)

Usage: python3 gravity_calibration.py
Emergency stop: Ctrl+C (zero current is always sent), or unplug the Touch power supply.
"""
import json
import os
import struct
import sys
import time
from collections import deque

import numpy as np
import usb.core
import usb.util

VID, PID = 0x2988, 0x0304
EP_OUT, EP_IN = 0x02, 0x81
ZERO = 0x0800
FLAGS = 0x03
MOTOR_SIGN = (-1, -1, +1)             # measured with characterize_motors.py
RAW_TO_OH_SIGN = (-1, -1, 1)          # raw USB encoders vs OpenHaptics encoders
MAX_DELTA = (400, 600, 600)
SATURATION_FRACTION = 0.95
MAX_POSE_ERROR = 150                  # counts

KP = (0.8, 0.8, 0.8)
KI = (1.0, 3.0, 3.0)
KD = (0.03, 0.03, 0.03)
INTEGRAL_LIMIT = 500.0
REF_SPEED = 800.0                     # counts / s
VEL_ALPHA = 0.2

N_POSES = 16
SHRINK = 0.15                         # pull the poses 15 % towards the centre
MIN_POSE_DIST = 300                   # counts: minimum distance between chosen poses
APPROACH = 150                        # counts: approach offset on each side
SAFE_PREDICTION = 550                 # do not save a model predicting more than this
SETTLE_TIMEOUT = 10.0
SETTLE_WINDOW = 0.5
SETTLE_ENC_SPREAD = 20
SETTLE_CMD_SPREAD = 25

KIN_PATH = os.path.expanduser('~/.touch_raw_driver/kinematics.json')
OUT_PATH = os.path.expanduser('~/.touch_raw_driver/gravity.json')


class TouchUSB:
    def __init__(self):
        dev = usb.core.find(idVendor=VID, idProduct=PID)
        if dev is None:
            sys.exit('Touch not found.')
        try:
            if dev.is_kernel_driver_active(0):
                dev.detach_kernel_driver(0)
        except (usb.core.USBError, NotImplementedError):
            pass
        usb.util.claim_interface(dev, 0)
        self.dev = dev
        self.counter = 0

    def exchange(self, deltas):
        cmd = [ZERO + int(d) for d in deltas]
        pkt = struct.pack('<BIHHHB', 0x0A, self.counter & 0xFFFFFFFF,
                          cmd[0], cmd[1], cmd[2], FLAGS)
        self.dev.write(EP_OUT, pkt, timeout=100)
        self.counter += 1
        try:
            data = bytes(self.dev.read(EP_IN, 64, timeout=20))
        except usb.core.USBTimeoutError:
            return None
        if len(data) >= 35 and data[0] == 0x0B:
            return struct.unpack('<13h', data[9:35])[:3]
        return None

    def send_zero(self, n=50):
        for _ in range(n):
            try:
                self.exchange([0, 0, 0])
            except usb.core.USBError:
                pass
            time.sleep(0.002)

    def close(self):
        usb.util.release_interface(self.dev, 0)
        usb.util.dispose_resources(self.dev)


def load_kinematics():
    with open(KIN_PATH) as f:
        m = json.load(f)
    W = np.array(m['enc_to_q'], dtype=float)
    A, b = W[:3, :].T, W[3, :]

    def raw_to_q(raw):
        e = np.array([raw[i] * RAW_TO_OH_SIGN[i] for i in range(3)], dtype=float)
        return A @ e + b
    return raw_to_q


class JointPID:
    def __init__(self, enc):
        self.ref = [float(e) for e in enc]
        self.target = list(self.ref)
        self.integ = [0.0, 0.0, 0.0]
        self.vel = [0.0, 0.0, 0.0]
        self.prev = list(enc)
        self.deltas = [0, 0, 0]

    def step(self, enc, dt):
        raw_v = [(e - p) / dt for e, p in zip(enc, self.prev)]
        self.vel = [VEL_ALPHA * r + (1 - VEL_ALPHA) * v for r, v in zip(raw_v, self.vel)]
        self.prev = list(enc)
        max_step = REF_SPEED * dt
        for i in range(3):
            step = max(-max_step, min(max_step, self.target[i] - self.ref[i]))
            self.ref[i] += step
            err = self.ref[i] - enc[i]
            self.integ[i] = max(-INTEGRAL_LIMIT,
                                min(INTEGRAL_LIMIT, self.integ[i] + KI[i] * err * dt))
            u = KP[i] * err + self.integ[i] - KD[i] * self.vel[i]
            self.deltas[i] = int(max(-MAX_DELTA[i], min(MAX_DELTA[i], MOTOR_SIGN[i] * u)))
        return self.deltas


class Runner:
    """Keeps the USB loop running and moves the PID between targets."""

    def __init__(self, touch):
        self.touch = touch
        enc = None
        while enc is None:
            enc = touch.exchange([0, 0, 0])
        self.pid = JointPID(enc)
        self.last_t = time.monotonic()

    def _tick(self):
        now = time.monotonic()
        dt = max(now - self.last_t, 1e-4)
        self.last_t = now
        enc = self.touch.exchange(self.pid.deltas)
        if enc is None:
            self.pid.deltas = [0, 0, 0]
            return now, None
        self.pid.step(enc, dt)
        time.sleep(0.001)
        return now, enc

    def move(self, target, wait=0.8):
        """Go to target (no measurement), then wait a moment."""
        self.pid.target = [float(x) for x in target]
        t_end = None
        while True:
            now, enc = self._tick()
            if all(abs(self.pid.ref[i] - self.pid.target[i]) < 1.0 for i in range(3)):
                t_end = t_end or now + wait
                if now >= t_end:
                    return

    def hold(self, target):
        """Go to target and measure the steady holding command. Returns (ok, cmd, enc, why)."""
        self.pid.target = [float(x) for x in target]
        t_start = time.monotonic()
        window = deque()
        while True:
            now, enc = self._tick()
            if enc is None:
                continue
            window.append((now, list(enc), list(self.pid.deltas)))
            while window and now - window[0][0] > SETTLE_WINDOW:
                window.popleft()
            reached = all(abs(self.pid.ref[i] - self.pid.target[i]) < 1.0 for i in range(3))
            encs = np.array([w[1] for w in window], dtype=float)
            cmds = np.array([w[2] for w in window], dtype=float)
            full = now - window[0][0] >= 0.9 * SETTLE_WINDOW
            enc_spread = (encs[:, 1:3].max(axis=0) - encs[:, 1:3].min(axis=0)).max()
            cmd_spread = (cmds[:, 1:3].max(axis=0) - cmds[:, 1:3].min(axis=0)).max()
            if reached and full and enc_spread <= SETTLE_ENC_SPREAD and cmd_spread <= SETTLE_CMD_SPREAD:
                cmd_mean, enc_mean = cmds.mean(axis=0), encs.mean(axis=0)
                for i in (1, 2):
                    if abs(cmd_mean[i]) >= SATURATION_FRACTION * MAX_DELTA[i]:
                        return False, None, None, f'motor {i} saturated ({cmd_mean[i]:+.0f})'
                    if abs(self.pid.target[i] - enc_mean[i]) > MAX_POSE_ERROR:
                        return False, None, None, f'joint {i} blocked ({self.pid.target[i] - enc_mean[i]:+.0f} counts)'
                return True, cmd_mean, enc_mean, None
            if now - t_start > SETTLE_TIMEOUT:
                return False, None, None, 'did not settle'


def phase1_ranges(touch, duration=25.0):
    print(f'\nPHASE 1 ({duration:.0f} s, motors OFF): move the arm by hand through the WHOLE '
          'workspace (up/down, near/far), without touching the table with the stylus.')
    lo, hi = [None] * 3, [None] * 3
    visited = []
    t0 = time.monotonic()
    next_print = 0.0
    while time.monotonic() - t0 < duration:
        enc = touch.exchange([0, 0, 0])
        if enc is not None:
            visited.append(enc)
            for i in range(3):
                lo[i] = enc[i] if lo[i] is None else min(lo[i], enc[i])
                hi[i] = enc[i] if hi[i] is None else max(hi[i], enc[i])
        t = time.monotonic() - t0
        if t >= next_print:
            print(f'  {duration - t:4.1f} s left | enc1 [{lo[1]}, {hi[1]}] | enc2 [{lo[2]}, {hi[2]}]',
                  flush=True)
            next_print += 1.0
        time.sleep(0.002)
    return lo, hi, np.array(visited, dtype=float)


def choose_poses(visited):
    """Pick well-spread, reachable (enc1, enc2) poses among those visited by hand."""
    pts = visited[:, 1:3]
    centre = pts.mean(axis=0)
    cand = centre + (1.0 - SHRINK) * (pts - centre)      # pulled towards the centre
    cand = cand[::10]                                    # thin out
    chosen = [cand[np.argmin(np.linalg.norm(cand - centre, axis=1))]]
    d = np.linalg.norm(cand - chosen[0], axis=1)
    while len(chosen) < N_POSES:
        i = int(np.argmax(d))                            # farthest-point sampling
        if d[i] < MIN_POSE_DIST:
            break
        chosen.append(cand[i])
        d = np.minimum(d, np.linalg.norm(cand - cand[i], axis=1))
    # visit order: nearest neighbour, to keep the moves short
    order, rest = [chosen[0]], chosen[1:]
    while rest:
        j = int(np.argmin([np.linalg.norm(r - order[-1]) for r in rest]))
        order.append(rest.pop(j))
    return order


def main():
    try:
        raw_to_q = load_kinematics()
    except (OSError, KeyError, ValueError) as e:
        sys.exit(f'Cannot load {KIN_PATH}: {e}. Run kin_logger + kin_fit.py first.')

    touch = TouchUSB()
    try:
        lo, hi, visited = phase1_ranges(touch)
        span1, span2 = hi[1] - lo[1], hi[2] - lo[2]
        if span1 < 500 or span2 < 500:
            print('Recorded range too small: move the arm more during phase 1. Aborting.')
            return
        poses = choose_poses(visited)
        print(f'\n{len(poses)} poses chosen among the visited configurations.')

        print('\nPHASE 2: hold the stylus near the centre. Motors ON in:')
        for k in (3, 2, 1):
            print(f'  {k}...', flush=True)
            t_end = time.monotonic() + 1.0
            while time.monotonic() < t_end:
                touch.exchange([0, 0, 0])
                time.sleep(0.002)
        run = Runner(touch)
        base = run.pid.ref[0]
        print('Motors ON. Let go of the stylus now (keep your hand close).')

        samples = []
        n = len(poses)
        for k, (e1, e2) in enumerate(poses, start=1):
            results = []
            for side in (+1, -1):
                run.move((base, e1 + side * APPROACH, e2 + side * APPROACH))
                ok, cmd, enc, why = run.hold((base, e1, e2))
                if not ok:
                    results = None
                    break
                results.append((cmd, enc))
            if results is None:
                print(f'  pose {k:2d}/{n}: skipped ({why})')
                continue
            cmd = (results[0][0] + results[1][0]) / 2.0
            enc = (results[0][1] + results[1][1]) / 2.0
            friction = np.abs(results[0][0] - results[1][0]) / 2.0
            q = raw_to_q(enc)
            samples.append((q[1], q[2], cmd[1], cmd[2]))
            print(f'  pose {k:2d}/{n}: q1={np.degrees(q[1]):6.1f} deg '
                  f'q2={np.degrees(q[2]):6.1f} deg -> hold m1={cmd[1]:+5.0f} m2={cmd[2]:+5.0f} '
                  f'(friction band +/-{friction[1]:.0f}, +/-{friction[2]:.0f})')

        touch.send_zero()
        if len(samples) < 6:
            print(f'\nOnly {len(samples)} valid poses: not enough. Nothing saved.')
            return

        s = np.array(samples)
        q1, q2, m1, m2 = s[:, 0], s[:, 1], s[:, 2], s[:, 3]
        Phi = np.column_stack([np.cos(q1), np.sin(q1), np.sin(q2), np.cos(q2)])

        def fit(cols, y):
            c, *_ = np.linalg.lstsq(Phi[:, cols], y, rcond=None)
            full = np.zeros(4)
            full[cols] = c
            return full, float(np.sqrt(np.mean((Phi @ full - y) ** 2)))

        d1, rd1 = fit([0, 1], m1)
        d2, rd2 = fit([2, 3], m2)
        c1, rc1 = fit([0, 1, 2, 3], m1)
        c2, rc2 = fit([0, 1, 2, 3], m2)
        print(f'\nFit RMS  decoupled: motor1 {rd1:.1f}, motor2 {rd2:.1f} | '
              f'coupled: motor1 {rc1:.1f}, motor2 {rc2:.1f}')
        use_coupled = len(samples) >= 10 and (rc1 + rc2) < 0.75 * (rd1 + rd2)
        if use_coupled:
            p1, p2, rms1, rms2, variant = c1, c2, rc1, rc2, 'coupled'
        else:
            p1, p2, rms1, rms2, variant = d1, d2, rd1, rd2, 'decoupled'
        print(f'Using the {variant} model.')

        # Safety check over the whole workspace visited in phase 1.
        qv = np.array([raw_to_q(e) for e in visited[::20]])
        Phv = np.column_stack([np.cos(qv[:, 1]), np.sin(qv[:, 1]),
                               np.sin(qv[:, 2]), np.cos(qv[:, 2])])
        g1v, g2v = Phv @ p1, Phv @ p2
        worst = max(np.abs(g1v).max(), np.abs(g2v).max())
        print(f'\nPredicted command over the visited workspace: motor1 [{g1v.min():+.0f}, '
              f'{g1v.max():+.0f}], motor2 [{g2v.min():+.0f}, {g2v.max():+.0f}]')
        if worst > SAFE_PREDICTION:
            print(f'The model predicts commands up to {worst:.0f} (> {SAFE_PREDICTION}): '
                  'NOT saved. Send this output for analysis.')
            return

        model = {
            'model': 'physics_v2',
            'basis': ['cos_q1', 'sin_q1', 'sin_q2', 'cos_q2'],
            'variant': variant,
            'description': 'Motor command (offset from 2048) holding the arm: '
                           'g_i = motor_i . [cos q1, sin q1, sin q2, cos q2]  '
                           '(q in rad, from kinematics.json)',
            'motor1': p1.tolist(),
            'motor2': p2.tolist(),
            'rms_fit_error': [rms1, rms2],
            'n_poses': len(samples),
            'samples_q1_q2_m1_m2': s.tolist(),
        }
        os.makedirs(os.path.dirname(OUT_PATH), exist_ok=True)
        with open(OUT_PATH, 'w') as f:
            json.dump(model, f, indent=2)
        print(f'\nModel saved to {OUT_PATH}')
        print(f'  valid poses: {len(samples)}')
        names = ('cos q1', 'sin q1', 'sin q2', 'cos q2')
        print('  motor 1: ' + ' '.join(f'{c:+.1f} {n}' for c, n in zip(p1, names) if c != 0.0)
              + f' | RMS {rms1:.1f}')
        print('  motor 2: ' + ' '.join(f'{c:+.1f} {n}' for c, n in zip(p2, names) if c != 0.0)
              + f' | RMS {rms2:.1f}')
    except KeyboardInterrupt:
        print('\nInterrupted.')
    finally:
        touch.send_zero()
        touch.close()
        print('Zero current sent, interface released.')


if __name__ == '__main__':
    main()
