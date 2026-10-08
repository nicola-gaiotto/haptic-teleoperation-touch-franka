#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Part of haptic-teleoperation-touch-franka - Copyright (c) 2026 Nicola Gaiotto
"""
kin_fit.py - kinematic calibration of the 3D Systems Touch for touch_raw_driver.

Input: the CSV written by kin_logger (OpenHaptics, read-only).
Steps:
  1. INKWELL CHECK: encoder values with the stylus in the inkwell (first 4 s). They must
     match what touch_raw_driver reads over USB in the same pose, otherwise the mapping
     below cannot be used with the raw driver.
  2. ENCODERS -> JOINT ANGLES: affine least-squares fit, q = A * enc[0:3] + b
     (a full 3x3 matrix, so it also captures any coupling between joints).
     Same for the gimbal: g = G * enc[3:6] + c (for orientation, later).
  3. FORWARD KINEMATICS: two candidate structures for the Phantom/Touch arm are fitted
     by linear least squares (unknowns: link lengths L1, L2 and offsets x0, y0, z0):
        A) q2 absolute:  r = L1 cos q1 + L2 sin q2
                         x = -sin(q0) r + x0,  y = L1 sin q1 - L2 cos q2 + y0,
                         z =  cos(q0) r + z0
        B) q2 relative:  same with q2 replaced by (q1 + q2)
     The structure with the smaller residual is kept.
  4. END-TO-END CHECK: encoders -> angles -> position, compared with OpenHaptics.
Output: ~/.touch_raw_driver/kinematics.json

Usage: python3 kin_fit.py [kin_log.csv]
"""
import json
import os
import sys

import numpy as np

OUT_PATH = os.path.expanduser('~/.touch_raw_driver/kinematics.json')
RAW_USB_INKWELL = [0, 659, -910]   # enc0..enc2 read by our USB scripts in the inkwell


def fk_terms(q0, q1, q2, structure):
    """Return the design-matrix columns of the FK model (linear in L1, L2, x0, y0, z0)."""
    q2e = q2 if structure == 'A' else q1 + q2
    s0, c0 = np.sin(q0), np.cos(q0)
    # x = -s0 (L1 c1 + L2 s2e) + x0 ; y = L1 s1 - L2 c2e + y0 ; z = c0 (L1 c1 + L2 s2e) + z0
    rows_x = np.column_stack([-s0 * np.cos(q1), -s0 * np.sin(q2e),
                              np.ones_like(q0), np.zeros_like(q0), np.zeros_like(q0)])
    rows_y = np.column_stack([np.sin(q1), -np.cos(q2e),
                              np.zeros_like(q0), np.ones_like(q0), np.zeros_like(q0)])
    rows_z = np.column_stack([c0 * np.cos(q1), c0 * np.sin(q2e),
                              np.zeros_like(q0), np.zeros_like(q0), np.ones_like(q0)])
    return rows_x, rows_y, rows_z


def fit_fk(q, pos, structure):
    rx, ry, rz = fk_terms(q[:, 0], q[:, 1], q[:, 2], structure)
    M = np.vstack([rx, ry, rz])
    y = np.concatenate([pos[:, 0], pos[:, 1], pos[:, 2]])
    p, *_ = np.linalg.lstsq(M, y, rcond=None)
    pred = (M @ p).reshape(3, -1).T
    rms = np.sqrt(np.mean((pred - pos) ** 2, axis=0))
    return p, pred, rms


def fk_eval(q, p, structure):
    rx, ry, rz = fk_terms(q[:, 0], q[:, 1], q[:, 2], structure)
    return np.column_stack([rx @ p, ry @ p, rz @ p])


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else 'kin_log.csv'
    d = np.genfromtxt(path, delimiter=',', names=True)
    if d.size < 500:
        sys.exit(f'Only {d.size} samples: log at least 60 s.')

    t = d['t']
    enc = np.column_stack([d[f'enc{i}'] for i in range(6)]).astype(float)
    q = np.column_stack([d['q0'], d['q1'], d['q2']])
    g = np.column_stack([d['g0'], d['g1'], d['g2']])
    pos = np.column_stack([d['x'], d['y'], d['z']])

    # 1. Inkwell check
    ink = t < 4.0
    print('1. INKWELL (first 4 s)')
    print(f'   OpenHaptics encoders : {np.round(enc[ink, :3].mean(axis=0)).astype(int).tolist()}'
          f'   (spread {np.ptp(enc[ink, :3], axis=0).astype(int).tolist()})')
    print(f'   raw USB encoders     : {RAW_USB_INKWELL}')
    print(f'   OpenHaptics position : {np.round(pos[ink].mean(axis=0), 1).tolist()} mm')

    moving = t >= 5.0
    e3, qm, gm, pm = enc[moving, :3], q[moving], g[moving], pos[moving]
    print(f'\n   samples used for the fit: {moving.sum()}')
    print(f'   encoder range: min {e3.min(axis=0).astype(int).tolist()} '
          f'max {e3.max(axis=0).astype(int).tolist()}')

    # 2. Encoders -> joint angles (affine)
    X = np.column_stack([e3, np.ones(len(e3))])
    W, *_ = np.linalg.lstsq(X, qm, rcond=None)           # (4 x 3)
    q_hat = X @ W
    rms_q = np.degrees(np.sqrt(np.mean((q_hat - qm) ** 2, axis=0)))
    print('\n2. ENCODERS -> JOINT ANGLES (affine fit)')
    for i in range(3):
        print(f'   q{i}: coeffs (rad/count) {np.round(W[:3, i] * 1e4, 4).tolist()} x1e-4, '
              f'offset {W[3, i]:+.4f} rad | RMS error {rms_q[i]:.3f} deg')

    Xg = np.column_stack([enc[moving, 3:6], np.ones(moving.sum())])
    Wg, *_ = np.linalg.lstsq(Xg, gm, rcond=None)
    rms_g = np.degrees(np.sqrt(np.mean((Xg @ Wg - gm) ** 2, axis=0)))
    print(f'   gimbal RMS error: {np.round(rms_g, 2).tolist()} deg')

    # 3. Forward kinematics
    print('\n3. FORWARD KINEMATICS (fitted on OpenHaptics angles)')
    best = None
    for structure in ('A', 'B'):
        p, _, rms = fit_fk(qm, pm, structure)
        print(f'   structure {structure}: L1={p[0]:.2f} mm L2={p[1]:.2f} mm '
              f'offset=({p[2]:.1f}, {p[3]:.1f}, {p[4]:.1f}) mm | RMS {np.round(rms, 2).tolist()} mm')
        if best is None or rms.mean() < best[2].mean():
            best = (structure, p, rms)
    structure, p, rms = best
    print(f'   -> using structure {structure}')

    # 4. End-to-end check: raw encoders -> angles -> position
    pos_hat = fk_eval(q_hat, p, structure)
    err = np.linalg.norm(pos_hat - pm, axis=1)
    print('\n4. END-TO-END (encoders -> position)')
    print(f'   position error: RMS {np.sqrt(np.mean(err ** 2)):.2f} mm, max {err.max():.2f} mm')

    model = {
        'description': 'Touch kinematics for touch_raw_driver. q = A^T [enc0, enc1, enc2, 1]; '
                       'position (OpenHaptics frame, mm) from q with the given structure.',
        'enc_to_q': W.tolist(),
        'enc_to_gimbal': Wg.tolist(),
        'fk_structure': structure,
        'L1_mm': float(p[0]), 'L2_mm': float(p[1]),
        'offset_mm': [float(p[2]), float(p[3]), float(p[4])],
        'rms_joint_deg': rms_q.tolist(),
        'rms_fk_mm': rms.tolist(),
        'rms_end_to_end_mm': float(np.sqrt(np.mean(err ** 2))),
        'encoder_range_min': e3.min(axis=0).tolist(),
        'encoder_range_max': e3.max(axis=0).tolist(),
        'inkwell_encoders_openhaptics': enc[ink, :3].mean(axis=0).tolist(),
    }
    os.makedirs(os.path.dirname(OUT_PATH), exist_ok=True)
    with open(OUT_PATH, 'w') as f:
        json.dump(model, f, indent=2)
    print(f'\nSaved to {OUT_PATH}')


if __name__ == '__main__':
    main()
