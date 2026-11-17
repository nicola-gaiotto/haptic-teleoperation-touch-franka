# SPDX-License-Identifier: MIT
from franka_teleop.mapping import ClutchMapping


def _close(a, b):
    return all(abs(x - y) < 1e-12 for x, y in zip(a, b))


def test_released_gives_no_target():
    m = ClutchMapping()
    assert not m.engaged and m.target([0.0, 0.0, 0.0]) is None


def test_engage_does_not_jump():
    m = ClutchMapping(scale=2.0)
    m.engage([0.01, 0.02, -0.03], [0.31, 0.0, 0.49])
    assert _close(m.target([0.01, 0.02, -0.03]), [0.31, 0.0, 0.49])


def test_scaled_displacement_same_axes():
    m = ClutchMapping(scale=1.5)
    m.engage([0.0, 0.0, 0.0], [0.31, 0.0, 0.49])
    # stylus 2 cm right, 1 cm forward, 1 cm down -> robot 3 cm +x, 1.5 cm +y, 1.5 cm -z
    assert _close(m.target([0.02, 0.01, -0.01]), [0.34, 0.015, 0.475])


def test_reclutch_moves_origin():
    m = ClutchMapping()
    m.engage([0.0, 0.0, 0.0], [0.3, 0.0, 0.5])
    m.release()
    assert m.target([0.05, 0.0, 0.0]) is None
    m.engage([-0.05, 0.0, 0.0], [0.35, 0.0, 0.5])      # stylus back left, robot stayed right
    assert _close(m.target([-0.03, 0.0, 0.0]), [0.37, 0.0, 0.5])


def test_yaw():
    m = ClutchMapping(yaw_deg=90.0)
    m.engage([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
    assert _close(m.target([0.01, 0.0, 0.0]), [0.0, 0.01, 0.0])


def test_wall_inside_box_is_zero():
    from franka_teleop.mapping import wall_force
    f = wall_force([0.35, 0.05, 0.45], [0.307, 0.0, 0.487], [0.1] * 3, 100.0)
    assert _close(f, [0.0, 0.0, 0.0])


def test_wall_pushes_back_inside():
    from franka_teleop.mapping import wall_force
    # 5 mm beyond +x and 3 mm below -z, K = 100 N/m -> (-0.5, 0, +0.3) N
    f = wall_force([0.412, 0.0, 0.384], [0.307, 0.0, 0.487], [0.1] * 3, 100.0)
    assert all(abs(a - b) < 1e-9 for a, b in zip(f, [-0.5, 0.0, 0.3]))


def test_wall_force_limited_and_rotated():
    from franka_teleop.mapping import ClutchMapping, wall_force
    f = wall_force([0.0, 0.2, 0.0], [0.0, 0.0, 0.0], [0.1] * 3, 100.0, max_force=1.0)
    assert _close(f, [0.0, -1.0, 0.0])
    # With yaw 90 deg, stylus +x maps to robot +y: a wall at robot +y pushes stylus -x.
    m = ClutchMapping(yaw_deg=90.0)
    m.engage([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
    t = m.target([0.12, 0.0, 0.0])
    f = wall_force(t, [0.0, 0.0, 0.0], [0.1] * 3, 10.0, yaw_deg=90.0)
    assert f[0] < 0 and abs(f[1]) < 1e-9


def test_force_feedback_sign_bias_deadband():
    from franka_teleop.mapping import ForceFeedback
    ff = ForceFeedback(gain=0.1, deadband=0.5, alpha=1.0, max_force=1.0)
    rest = [-0.9, 0.6, 0.6]
    assert ff.update(rest) == [0.0, 0.0, 0.0]          # no bias yet -> no force
    ff.set_bias(rest)
    assert _close(ff.update([-0.8, 0.5, 0.7]), [0.0, 0.0, 0.0])   # noise inside dead zone
    # Hand pushing the end effector DOWN (lab: reading Fz = bias + 8.5 N) -> Touch pushed DOWN
    f = ff.update([-0.9, 0.6, 0.6 + 8.5])
    assert abs(f[0]) < 1e-12 and abs(f[1]) < 1e-12 and abs(f[2] + 0.8) < 1e-9
    # Robot moving down onto a table (reading Fz = bias - 8.5 N) -> Touch pushed UP
    ff.set_bias(rest)
    assert ff.update([-0.9, 0.6, 0.6 - 8.5])[2] > 0.79
    # Large force is limited
    assert abs(ff.update([-0.9, 0.6, 0.6 + 50.0])[2] + 1.0) < 1e-9
