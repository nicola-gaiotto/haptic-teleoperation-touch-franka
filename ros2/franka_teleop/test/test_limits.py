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


def test_limit_contact():
    from franka_teleop.limits import limit_contact
    # below the limit: unchanged
    assert limit_contact([0.0, 0.0, -0.05], [0.0, 0.0, 10.0], 15.0) == ([0.0, 0.0, -0.05], False)
    # sponge pushes up with 20 N: moving down is blocked, sideways motion stays
    v, lim = limit_contact([0.02, 0.0, -0.05], [0.0, 0.0, 20.0], 15.0)
    assert lim and abs(v[2]) < 1e-12 and abs(v[0] - 0.02) < 1e-12
    # moving away from the contact is allowed
    assert limit_contact([0.0, 0.0, 0.03], [0.0, 0.0, 20.0], 15.0) == ([0.0, 0.0, 0.03], False)
    # limit 0 = off
    assert limit_contact([0.0, 0.0, -0.05], [0.0, 0.0, 50.0], 0.0)[1] is False
    # back-off: 13 N with a 10 N limit, 1 mm/s per N -> 3 mm/s up, sideways kept
    v, lim = limit_contact([0.02, 0.0, -0.05], [0.0, 0.0, 13.0], 10.0, 0.001, 0.005)
    assert lim and abs(v[2] - 0.003) < 1e-12 and abs(v[0] - 0.02) < 1e-12
    # back-off speed is capped
    v, _ = limit_contact([0.0, 0.0, -0.05], [0.0, 0.0, 40.0], 10.0, 0.001, 0.005)
    assert abs(v[2] - 0.005) < 1e-12
    # back-off along the force direction (force tilted 45 deg in x-z)
    v, _ = limit_contact([0.0, 0.0, -0.02], [5.0, 0.0, 5.0], 5.0, 0.001, 0.005)
    n = (50.0) ** 0.5
    vn = sum(a * b / n for a, b in zip(v, [5.0, 0.0, 5.0]))
    assert abs(vn - 0.001 * (n - 5.0)) < 1e-12
    # hysteresis: 9 N after being limited (limit 10, margin 3) -> still blocked, no back-off
    v, lim = limit_contact([0.0, 0.0, -0.02], [0.0, 0.0, 9.0], 10.0, 0.001, 0.005, 3.0, True)
    assert lim and abs(v[2]) < 1e-12
    # ... but 9 N without having been limited -> free
    assert limit_contact([0.0, 0.0, -0.02], [0.0, 0.0, 9.0], 10.0, 0.001, 0.005, 3.0, False)[1] is False
    # released below limit - margin
    assert limit_contact([0.0, 0.0, -0.02], [0.0, 0.0, 6.5], 10.0, 0.001, 0.005, 3.0, True)[1] is False
    # moving away releases it
    assert limit_contact([0.0, 0.0, 0.01], [0.0, 0.0, 9.0], 10.0, 0.001, 0.005, 3.0, True)[1] is False


def test_workspace_axis_off_and_floor():
    from franka_teleop.limits import apply_floor, apply_workspace, clip_to_box
    c = [0.3, 0.0, 0.5]
    # half size 0 = no limit on that axis
    assert apply_workspace([0.1, 0.1, 0.1], [0.9, 0.9, 0.9], c, [0.0, 0.0, 0.0]) == [0.1, 0.1, 0.1]
    assert clip_to_box([0.9, -0.9, 0.9], c, [0.0, 0.1, 0.0]) == [0.9, -0.1, 0.9]
    # floor
    assert apply_floor([0.01, 0.0, -0.02], [0.3, 0.0, 0.09], 0.10) == [0.01, 0.0, 0.0]
    assert apply_floor([0.0, 0.0, 0.02], [0.3, 0.0, 0.09], 0.10) == [0.0, 0.0, 0.02]
    assert apply_floor([0.0, 0.0, -0.02], [0.3, 0.0, 0.09], None) == [0.0, 0.0, -0.02]


def test_ramp_velocity():
    from franka_teleop.limits import ramp_velocity
    # 0 -> 0.2 m/s with 2 m/s^2 at 50 Hz: 0.04 m/s per tick
    v = ramp_velocity([0.0, 0.0, 0.0], [0.2, 0.0, 0.0], 2.0, 0.02)
    assert abs(v[0] - 0.04) < 1e-12
    # small change: reached at once
    assert ramp_velocity([0.1, 0.0, 0.0], [0.11, 0.0, 0.0], 2.0, 0.02) == [0.11, 0.0, 0.0]
    # off
    assert ramp_velocity([0.0, 0.0, 0.0], [0.5, 0.0, 0.0], 0.0, 0.02) == [0.5, 0.0, 0.0]
