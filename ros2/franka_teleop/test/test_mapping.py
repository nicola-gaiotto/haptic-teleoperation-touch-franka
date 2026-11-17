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
