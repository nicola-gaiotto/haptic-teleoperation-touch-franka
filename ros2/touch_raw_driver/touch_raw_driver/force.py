# SPDX-License-Identifier: MIT
# Part of haptic-teleoperation-touch-franka - Copyright (c) 2026 Nicola Gaiotto
"""
Force rendering on the Touch: Cartesian force [N] -> motor commands (pure math, no ROS, no USB).

    tau_raw = J_raw(enc)^T F            J_raw = d p / d enc   [m / count], p in touch_base
    tau_i   = tau_raw_i / |dq_i/denc_i| joint torque [N m] along the direction of increasing enc_i
    u_i     = force_gain_i * tau_i      command (before MOTOR_SIGN) that makes enc_i increase

force_gain [command per N m] comes from tools/calibration/force_calibration.py. The Jacobian is
computed numerically from the identified forward kinematics, so it stays consistent with
/touch/pose by construction.
"""
import numpy as np


def jacobian_raw(position_of_raw, raw, step=5.0):
    """3 x 3 Jacobian d p / d raw [m / count] by central differences.

    position_of_raw: function raw [enc0, enc1, enc2] -> position [x, y, z] in metres.
    """
    raw = np.asarray(raw, dtype=float)
    J = np.zeros((3, 3))
    for i in range(3):
        d = np.zeros(3)
        d[i] = step
        J[:, i] = (np.asarray(position_of_raw(raw + d)) -
                   np.asarray(position_of_raw(raw - d))) / (2.0 * step)
    return J


def limit_norm(f, max_norm):
    """Scale the vector f down so that its norm is at most max_norm."""
    f = np.asarray(f, dtype=float)
    n = float(np.linalg.norm(f))
    if n <= max_norm or n == 0.0:
        return f
    return f * (max_norm / n)


def slew(current, target, max_step):
    """Move current towards target by at most max_step (vector norm)."""
    current = np.asarray(current, dtype=float)
    return current + limit_norm(np.asarray(target, dtype=float) - current, max_step)


def force_to_commands(J, force, rad_per_count, force_gain):
    """Commands u_i (positive = makes enc_i increase) that render `force` [N] at the tip.

    J: Jacobian d p / d raw [m / count]; rad_per_count: |dq_i / denc_i| for each joint;
    force_gain: command per N m for each motor.
    """
    tau_raw = J.T @ np.asarray(force, dtype=float)            # N m per count
    tau = tau_raw / np.asarray(rad_per_count, dtype=float)    # N m
    return np.asarray(force_gain, dtype=float) * tau
