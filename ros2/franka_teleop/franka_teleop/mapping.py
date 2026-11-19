# SPDX-License-Identifier: MIT
# Part of haptic-teleoperation-touch-franka - Copyright (c) 2026 Nicola Gaiotto
"""
Pure position mapping Touch -> Franka with a clutch (no ROS, no robot; unit-tested).

    p_target = p_franka0 + scale * R(yaw) * (p_touch - p_touch0)

p_touch0 and p_franka0 are stored when the clutch is ENGAGED, so engaging never makes the
robot jump: at that instant the target is exactly where the robot is. While the clutch is
released the robot holds still and the stylus can be repositioned.

touch_base (x right, y forward, z up) has the same orientation as the FR3 base seen by the
operator in the lab (x right, y away, z up; measured 2026-11-02), so yaw_deg = 0 by default.
"""
import math


class ClutchMapping:

    def __init__(self, scale=1.0, yaw_deg=0.0):
        self.scale = float(scale)
        self.yaw_deg = float(yaw_deg)
        self._touch0 = None
        self._franka0 = None

    @property
    def engaged(self):
        return self._touch0 is not None

    def engage(self, touch_position, franka_position):
        self._touch0 = list(touch_position)
        self._franka0 = list(franka_position)

    def release(self):
        self._touch0 = None
        self._franka0 = None

    def target(self, touch_position):
        """Franka target [m] in the robot base frame, or None if the clutch is released."""
        if not self.engaged:
            return None
        d = [touch_position[i] - self._touch0[i] for i in range(3)]
        c = math.cos(math.radians(self.yaw_deg))
        s = math.sin(math.radians(self.yaw_deg))
        d = [c * d[0] - s * d[1], s * d[0] + c * d[1], d[2]]
        return [self._franka0[i] + self.scale * d[i] for i in range(3)]


def wall_force(target, centre, half_size, stiffness, yaw_deg=0.0, max_force=1.0):
    """Virtual-wall force [N] for the Touch, in touch_base.

    target, centre, half_size: robot base frame [m] (the franka_teleop workspace box).
    Each component of the target outside the box is pushed back with a spring,
    F_robot = -stiffness * excess, then rotated back into touch_base (inverse of the yaw
    used by ClutchMapping) and limited to max_force (norm).
    """
    excess = []
    for p, c, h in zip(target, centre, half_size):
        lo, hi = c - h, c + h
        excess.append(p - hi if p > hi else (p - lo if p < lo else 0.0))
    fr = [-stiffness * e for e in excess]
    c = math.cos(math.radians(yaw_deg))
    s = math.sin(math.radians(yaw_deg))
    f = [c * fr[0] + s * fr[1], -s * fr[0] + c * fr[1], fr[2]]
    n = math.sqrt(sum(v * v for v in f))
    if n > max_force > 0.0:
        f = [v * max_force / n for v in f]
    return f


class ForceFeedback:
    """FR3 external force -> Touch force (pure math, unit-tested).

    O_F_ext_hat_K from libfranka/franky is the force the robot applies to the environment
    (lab 2026-11-17: pushing the end effector down by hand gave Fz up to +9 N). The force the
    environment applies to the robot is its opposite, so the Touch renders

        F_touch = R(yaw)^T * gain * dz(LPF(-(F_ext - F_bias)), deadband)

    F_bias is the reading stored when the clutch is engaged (offset of about 1 N measured at
    rest). The dead zone removes the noise (about +-0.1 N measured at rest); the norm is limited
    to max_force.
    """

    def __init__(self, gain=0.1, deadband=0.5, alpha=0.2, max_force=1.0, yaw_deg=0.0):
        self.gain, self.deadband, self.alpha = float(gain), float(deadband), float(alpha)
        self.max_force, self.yaw_deg = float(max_force), float(yaw_deg)
        self._bias = None
        self._filt = [0.0, 0.0, 0.0]

    def set_bias(self, f_ext):
        self._bias = list(f_ext)
        self._filt = [0.0, 0.0, 0.0]

    def reset(self):
        self._bias = None
        self._filt = [0.0, 0.0, 0.0]

    def update(self, f_ext):
        """New wrench sample (force part, base frame) -> Touch force [N] in touch_base."""
        if self._bias is None:
            return [0.0, 0.0, 0.0]
        env = [-(f - b) for f, b in zip(f_ext, self._bias)]
        self._filt = [v + self.alpha * (e - v) for v, e in zip(self._filt, env)]
        n = math.sqrt(sum(v * v for v in self._filt))
        if n <= self.deadband:
            return [0.0, 0.0, 0.0]
        fr = [v * (n - self.deadband) / n * self.gain for v in self._filt]   # smooth dead zone
        c = math.cos(math.radians(self.yaw_deg))
        s = math.sin(math.radians(self.yaw_deg))
        f = [c * fr[0] + s * fr[1], -s * fr[0] + c * fr[1], fr[2]]
        return limit_norm(f, self.max_force)


def limit_norm(f, max_norm):
    n = math.sqrt(sum(v * v for v in f))
    if n > max_norm > 0.0:
        return [v * max_norm / n for v in f]
    return list(f)
