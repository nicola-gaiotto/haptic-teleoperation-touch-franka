# SPDX-License-Identifier: MIT
from franka_teleop.limits import apply_workspace, clamp_speed, is_zero, twist_to_velocity


def test_twist_mapping():
    assert twist_to_velocity([1, 0, 0], 0.0, 0.02, True, 0.02) == [0.02, 0.0, 0.0]
    assert twist_to_velocity([0, 0, 0], 1.0, 0.02, True, 0.02) == [0.0, 0.02, 0.0]
    assert twist_to_velocity([0, 0, 0], 1.0, 0.02, False, 0.02) == [0.0, 0.0, 0.0]


def _close(a, b):
    return all(abs(x - y) < 1e-12 for x, y in zip(a, b))


def test_operator_yaw_90_lab_setup():
    # i (forward) -> away from the operator = base +y; j (left) -> base -x; t (up) -> +z
    assert _close(twist_to_velocity([1, 0, 0], 0.0, 0.02, True, 0.02, 90.0), [0.0, 0.02, 0.0])
    assert _close(twist_to_velocity([0, 0, 0], 1.0, 0.02, True, 0.02, 90.0), [-0.02, 0.0, 0.0])
    assert _close(twist_to_velocity([0, 0, 1], 0.0, 0.02, True, 0.02, 90.0), [0.0, 0.0, 0.02])


def test_clamp_speed():
    v = clamp_speed([0.03, 0.04, 0.0], 0.02)      # norm 0.05 -> 0.02
    assert abs(v[0] - 0.012) < 1e-12 and abs(v[1] - 0.016) < 1e-12
    assert clamp_speed([0.01, 0.0, 0.0], 0.02) == [0.01, 0.0, 0.0]


def test_workspace_blocks_only_outward_motion():
    centre, half = [0.5, 0.0, 0.5], [0.1, 0.1, 0.1]
    at_x_max = [0.605, 0.0, 0.5]   # just outside the box
    assert apply_workspace([0.02, 0.01, 0.0], at_x_max, centre, half) == [0.0, 0.01, 0.0]
    assert apply_workspace([-0.02, 0.0, 0.0], at_x_max, centre, half) == [-0.02, 0.0, 0.0]
    assert is_zero(apply_workspace([0.0, 0.0, -0.02], [0.5, 0.0, 0.395], centre, half))


def test_clip_to_box():
    from franka_teleop.limits import clip_to_box
    assert _close(clip_to_box([0.7, 0.0, 0.3], [0.5, 0.0, 0.5], [0.1, 0.1, 0.1]),
                  [0.6, 0.0, 0.4])


def test_position_to_velocity():
    from franka_teleop.limits import position_to_velocity
    assert _close(position_to_velocity([0.51, 0.0, 0.5], [0.5, 0.0, 0.5], 3.0), [0.03, 0.0, 0.0])
    assert position_to_velocity([0.5005, 0.0, 0.5], [0.5, 0.0, 0.5], 3.0, 0.001) == [0.0] * 3
