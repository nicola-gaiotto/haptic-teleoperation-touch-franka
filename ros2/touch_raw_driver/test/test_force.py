# SPDX-License-Identifier: MIT
# Part of haptic-teleoperation-touch-franka - Copyright (c) 2026 Nicola Gaiotto
"""Unit tests of the force rendering math (no hardware needed)."""
import numpy as np

from test_kinematics import _import_driver, _model_file
from touch_raw_driver.force import force_to_commands, jacobian_raw, limit_norm, slew

MOTOR_SIGN = (-1, -1, +1)
POSES = ([0, 659, -910], [800, -1500, 1200], [-1200, -3000, 3000], [0, -2000, 1800])


def test_jacobian_matches_small_motion(tmp_path):
    drv = _import_driver()
    kin = drv.Kinematics(_model_file(tmp_path))
    for raw in POSES:
        J = jacobian_raw(kin.position_of_raw, raw)
        d = np.array([3.0, -2.0, 4.0])
        dp = kin.position_of_raw(np.array(raw, float) + d) - kin.position_of_raw(raw)
        assert np.allclose(J @ d, dp, atol=1e-6)


def test_commands_push_along_the_force(tmp_path):
    """A small encoder motion in the direction of u moves the tip along F (positive work)."""
    drv = _import_driver()
    kin = drv.Kinematics(_model_file(tmp_path))
    rpc = kin.rad_per_count()
    for raw in POSES:
        J = jacobian_raw(kin.position_of_raw, raw)
        for f in np.eye(3).tolist() + (-np.eye(3)).tolist():
            u = force_to_commands(J, f, rpc, [2000.0] * 3)
            dp = kin.position_of_raw(np.array(raw, float) + 1e-3 * u) - \
                kin.position_of_raw(raw)
            assert np.dot(dp, f) > 0.0


def test_vertical_force_magnitude(tmp_path):
    """1 N straight down with the arm horizontal: joint torque about L * 1 N."""
    drv = _import_driver()
    kin = drv.Kinematics(_model_file(tmp_path))
    raw = [0, 659, -910]
    J = jacobian_raw(kin.position_of_raw, raw)
    u = force_to_commands(J, [0.0, 0.0, -1.0], kin.rad_per_count(), [1.0, 1.0, 1.0])
    assert abs(u[0]) < 1e-3                       # base joint: vertical axis, no torque
    assert np.all(np.abs(u[1:]) < 0.2)            # |tau| <= L1 + L2 = 0.27 N m, roughly
    assert np.linalg.norm(u[1:]) > 0.05


def test_limit_norm_and_slew():
    assert np.allclose(limit_norm([3.0, 4.0, 0.0], 1.0), [0.6, 0.8, 0.0])
    assert np.allclose(limit_norm([0.1, 0.0, 0.0], 1.0), [0.1, 0.0, 0.0])
    f = np.zeros(3)
    for _ in range(10):
        f = slew(f, [1.0, 0.0, 0.0], 0.02)
    assert np.allclose(f, [0.2, 0.0, 0.0])
    assert np.allclose(slew(f, [0.21, 0.0, 0.0], 0.02), [0.21, 0.0, 0.0])
