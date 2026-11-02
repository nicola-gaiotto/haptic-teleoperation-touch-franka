# SPDX-License-Identifier: MIT
# Part of haptic-teleoperation-touch-franka - Copyright (c) 2026 Nicola Gaiotto
"""
Pure functions (no ROS, no franky) that turn a velocity command into a safe linear velocity
of the Franka end effector. Kept separate so that they can be unit-tested without a robot.

The Twist is read in the OPERATOR frame (usual ROS convention: x forward = away from the
operator, y left, z up), so that the keys of teleop_twist_keyboard follow the operator's view:
i/, = away/towards, j/l (turn) = left/right, t/b = up/down.
It is then rotated about z by operator_yaw_deg into the robot base frame (fr3_link0).
Measured in the lab (2026-11-02): from the operator's position, base +x = operator's right and
base +y = away from the operator, i.e. operator_yaw_deg = 90.
"""
import math


def twist_to_velocity(linear, angular_z, linear_scale, lateral_from_angular, angular_scale,
                      operator_yaw_deg=0.0):
    """Velocity [m/s] in the robot base frame from the fields of a geometry_msgs/Twist given
    in the operator frame. operator_yaw_deg: angle of the operator's forward direction measured
    from base +x towards base +y."""
    fx = linear[0] * linear_scale       # forward (away from the operator)
    fy = linear[1] * linear_scale       # left
    vz = linear[2] * linear_scale
    if lateral_from_angular:
        fy += angular_z * angular_scale     # j (turn left, +angular.z) -> move left
    c = math.cos(math.radians(operator_yaw_deg))
    s = math.sin(math.radians(operator_yaw_deg))
    return [c * fx - s * fy, s * fx + c * fy, vz]


def clamp_speed(vel, max_speed):
    """Scale the velocity down so that its norm is at most max_speed [m/s]."""
    norm = math.sqrt(sum(v * v for v in vel))
    if norm <= max_speed or norm == 0.0:
        return list(vel)
    k = max_speed / norm
    return [v * k for v in vel]


def apply_workspace(vel, position, centre, half_size):
    """Zero the velocity components that would move the end effector further out of the box
    centre +- half_size (per axis). Motion back towards the inside is always allowed."""
    out = list(vel)
    for i in range(3):
        offset = position[i] - centre[i]
        if offset >= half_size[i] and out[i] > 0.0:
            out[i] = 0.0
        elif offset <= -half_size[i] and out[i] < 0.0:
            out[i] = 0.0
    return out


def is_zero(vel, eps=1e-6):
    return all(abs(v) < eps for v in vel)
