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
