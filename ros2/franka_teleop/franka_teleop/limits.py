# SPDX-License-Identifier: MIT
# Part of haptic-teleoperation-touch-franka - Copyright (c) 2026 Nicola Gaiotto
"""
Pure functions (no ROS, no franky) that turn a velocity command into a safe linear velocity
of the Franka end effector. Kept separate so that they can be unit-tested without a robot.

Frame: robot base frame (fr3_link0): x forward (away from the base), y left, z up, metres.
This is also the usual ROS convention of geometry_msgs/Twist, so the keyboard keys of
teleop_twist_keyboard map directly: i/, = forward/back, t/b = up/down, j/l (turn) = left/right.
"""
import math


def twist_to_velocity(linear, angular_z, linear_scale, lateral_from_angular, angular_scale):
    """Velocity [m/s] in the robot base frame from the fields of a geometry_msgs/Twist."""
    vx = linear[0] * linear_scale
    vy = linear[1] * linear_scale
    vz = linear[2] * linear_scale
    if lateral_from_angular:
        vy += angular_z * angular_scale     # j (turn left, +angular.z) -> move left (+y)
    return [vx, vy, vz]


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
