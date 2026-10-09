#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Part of haptic-teleoperation-touch-franka - Copyright (c) 2026 Nicola Gaiotto
"""
twist_to_target - velocity teleoperation of the 3D Systems Touch.

Converts velocity commands (geometry_msgs/Twist, e.g. from teleop_twist_keyboard) into a
Cartesian target that moves continuously, and sends it to touch_raw_driver.

    teleop_twist_keyboard --/touch/cmd_vel (Twist)--> twist_to_target --/touch/target--> touch_raw_driver
                                                            ^
                                                            '---- /touch/pose

Velocity mapping (Twist follows the usual ROS convention: x forward, y left, z up;
the Touch frame is x right, y forward, z up):
    touch x (right)   = -linear.y * linear_scale  (- angular.z * angular_scale, optional)
    touch y (forward) =  linear.x * linear_scale
    touch z (up)      =  linear.z * linear_scale

Behaviour:
  - the target starts from the current stylus position (no jump);
  - while commands arrive, the target moves at the commanded velocity;
  - if no command arrives for cmd_timeout seconds, the motion stops
    (teleop_twist_keyboard sends a message only when a key is pressed or auto-repeated);
  - when a new motion starts, the target continues from where it was; it restarts from
    the actual stylus position only if the two are farther apart than resync_distance
    (stylus moved by hand, motors disabled). Restarting from the actual position after
    every pause would make a small gravity sag accumulate (downward drift);
  - the target stays inside a workspace box; along the commanded axes it is at most
    max_lead metres ahead of the actual stylus position, so it never runs away if the
    arm cannot follow. The other axes are left untouched (otherwise a sag on z would
    drag the z target down while moving left/right).

Topics:
  subscribes  <cmd_topic> (default /touch/cmd_vel)  geometry_msgs/Twist
  subscribes  /touch/pose                           geometry_msgs/PoseStamped
  publishes   /touch/target                         geometry_msgs/Point (only while moving)
  publishes   /touch/teleop/markers                 visualization_msgs/Marker (RViz)
"""
import math
import time

import rclpy
from geometry_msgs.msg import Point, PoseStamped, Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from visualization_msgs.msg import Marker


class TwistToTarget(Node):

    def __init__(self):
        super().__init__('twist_to_target')
        self.declare_parameter('cmd_topic', '/touch/cmd_vel')
        self.declare_parameter('linear_scale', 0.05)
        self.declare_parameter('lateral_from_angular', True)
        self.declare_parameter('angular_scale', 0.025)
        self.declare_parameter('rate', 50.0)
        self.declare_parameter('cmd_timeout', 0.55)
        self.declare_parameter('resync_distance', 0.015)
        self.declare_parameter('max_lead', 0.02)
        self.declare_parameter('workspace_min', [-0.08, -0.06, -0.05])
        self.declare_parameter('workspace_max', [0.08, 0.06, 0.08])

        self._pose = None            # actual stylus position [m]
        self._frame = 'touch_base'
        self._vel = [0.0, 0.0, 0.0]  # commanded velocity in the touch frame [m/s]
        self._last_cmd = None        # monotonic time of the last velocity command
        self._target = None
        self._last_tick = time.monotonic()

        cmd_topic = str(self.get_parameter('cmd_topic').value)
        self.create_subscription(Twist, cmd_topic, self._on_twist, 10)
        self.create_subscription(PoseStamped, '/touch/pose', self._on_pose, 10)
        self._pub_target = self.create_publisher(Point, '/touch/target', 10)
        self._pub_marker = self.create_publisher(Marker, '/touch/teleop/markers', 10)

        rate = float(self.get_parameter('rate').value)
        self.create_timer(1.0 / rate, self._on_timer)
        self.create_timer(1.0, self._publish_workspace)

        self.get_logger().info(
            f'Listening to {cmd_topic}. Remember: touch_raw_driver must be ENABLED and in '
            "'position' mode.")

    # ------------------------------------------------------------------ callbacks
    def _on_pose(self, msg):
        p = msg.pose.position
        self._pose = [p.x, p.y, p.z]
        if msg.header.frame_id:
            self._frame = msg.header.frame_id

    def _on_twist(self, msg):
        ls = float(self.get_parameter('linear_scale').value)
        vx = -msg.linear.y * ls
        if bool(self.get_parameter('lateral_from_angular').value):
            vx += -msg.angular.z * float(self.get_parameter('angular_scale').value)
        self._vel = [vx, msg.linear.x * ls, msg.linear.z * ls]
        self._last_cmd = time.monotonic()

    def _on_timer(self):
        now = time.monotonic()
        dt = min(now - self._last_tick, 0.1)
        self._last_tick = now
        if self._pose is None:
            self.get_logger().warn('Waiting for /touch/pose (is touch_raw_driver running?)',
                                   throttle_duration_sec=5.0)
            return

        timeout = float(self.get_parameter('cmd_timeout').value)
        moving = (self._last_cmd is not None and now - self._last_cmd < timeout
                  and any(abs(v) > 1e-9 for v in self._vel))

        if not moving:
            self._publish_target_marker()
            return

        if self._target is None:
            self._target = list(self._pose)
        else:
            gap = math.sqrt(sum((self._target[i] - self._pose[i]) ** 2 for i in range(3)))
            if gap > float(self.get_parameter('resync_distance').value):
                self._target = list(self._pose)   # stylus moved by hand / motors disabled

        lo = [float(v) for v in self.get_parameter('workspace_min').value]
        hi = [float(v) for v in self.get_parameter('workspace_max').value]
        lead = float(self.get_parameter('max_lead').value)

        # Integrate the velocity, then apply the workspace box.
        t = [min(max(self._target[i] + self._vel[i] * dt, lo[i]), hi[i]) for i in range(3)]
        # Along the commanded axes only, keep the target at most max_lead ahead of the
        # actual stylus position.
        for i in range(3):
            if abs(self._vel[i]) > 1e-9 and lead > 0.0:
                t[i] = min(max(t[i], self._pose[i] - lead), self._pose[i] + lead)
        self._target = t

        self._pub_target.publish(Point(x=t[0], y=t[1], z=t[2]))
        self._publish_target_marker()

    # ------------------------------------------------------------------ RViz markers
    def _marker(self, ns, mid, mtype):
        m = Marker()
        m.header.frame_id = self._frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    def _publish_target_marker(self):
        m = self._marker('teleop_target', 0, Marker.SPHERE)
        if self._target is None:
            m.action = Marker.DELETE
        else:
            m.pose.position.x, m.pose.position.y, m.pose.position.z = self._target
            m.scale.x = m.scale.y = m.scale.z = 0.012
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.5, 0.0, 0.9
        self._pub_marker.publish(m)

    def _publish_workspace(self):
        lo = [float(v) for v in self.get_parameter('workspace_min').value]
        hi = [float(v) for v in self.get_parameter('workspace_max').value]
        m = self._marker('workspace', 0, Marker.CUBE)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = (
            (lo[i] + hi[i]) / 2.0 for i in range(3))
        m.scale.x, m.scale.y, m.scale.z = (hi[i] - lo[i] for i in range(3))
        m.color.r, m.color.g, m.color.b, m.color.a = 0.2, 0.6, 1.0, 0.12
        self._pub_marker.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = TwistToTarget()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
