#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Part of haptic-teleoperation-touch-franka - Copyright (c) 2026 Nicola Gaiotto
"""
touch_to_franka - position teleoperation: the Touch stylus moves the Franka end effector.

    touch_raw_driver --/touch/pose--> touch_to_franka --/franka/target (Point)--> franka_teleop
    franka_teleop ---/franka/pose---^

    p_target = p_franka0 + scale * (p_touch - p_touch0)      (position only, orientation fixed)

Clutch (std_msgs/Bool on clutch_topic):
  - true  = ENGAGE: p_touch0 and p_franka0 are stored, so the robot never jumps;
  - false = RELEASE: one zero Twist is sent on /franka/cmd_vel, the robot stops at once and
    the stylus can be repositioned.
The clutch is released automatically if /touch/pose stops arriving (touch_timeout); it must
then be engaged again by hand. Speed limit, workspace box, timeout and enable stay in
franka_teleop: this node only computes the target.

Topics:
  subscribes  /touch/pose        geometry_msgs/PoseStamped (touch_base: x right, y forward, z up)
  subscribes  /franka/pose       geometry_msgs/PoseStamped (robot base frame)
  subscribes  <clutch_topic>     std_msgs/Bool   (default /touch_to_franka/clutch)
  publishes   <target_topic>     geometry_msgs/Point (default /franka/target), at `rate`
  publishes   /franka/cmd_vel    geometry_msgs/Twist (zero, once, on release)
  publishes   /touch_to_franka/status  std_msgs/String  'released' or 'engaged'

Parameters: see config/touch_to_franka.yaml.
"""
import time

import rclpy
from geometry_msgs.msg import Point, PoseStamped, Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String

from franka_teleop.mapping import ClutchMapping


class TouchToFranka(Node):

    def __init__(self):
        super().__init__('touch_to_franka')
        self.declare_parameter('scale', 1.0)
        self.declare_parameter('yaw_deg', 0.0)
        self.declare_parameter('rate', 50.0)
        self.declare_parameter('touch_timeout', 0.1)
        self.declare_parameter('franka_timeout', 0.5)
        self.declare_parameter('clutch_topic', '/touch_to_franka/clutch')
        self.declare_parameter('target_topic', '/franka/target')

        self._map = ClutchMapping(float(self.get_parameter('scale').value),
                                  float(self.get_parameter('yaw_deg').value))
        self._touch = None           # (position, monotonic time)
        self._franka = None

        self.create_subscription(PoseStamped, '/touch/pose', self._on_touch, 10)
        self.create_subscription(PoseStamped, '/franka/pose', self._on_franka, 10)
        self.create_subscription(Bool, str(self.get_parameter('clutch_topic').value),
                                 self._on_clutch, 10)
        self._pub_target = self.create_publisher(
            Point, str(self.get_parameter('target_topic').value), 10)
        self._pub_stop = self.create_publisher(Twist, '/franka/cmd_vel', 10)
        self._pub_status = self.create_publisher(String, '/touch_to_franka/status', 10)
        self.create_timer(1.0 / float(self.get_parameter('rate').value), self._on_timer)
        self.create_timer(0.2, self._publish_status)
        self.get_logger().info(
            f'Scale {self._map.scale}. Clutch RELEASED: publish true on '
            f"{self.get_parameter('clutch_topic').value} to engage.")

    @staticmethod
    def _position(msg):
        p = msg.pose.position
        return [p.x, p.y, p.z]

    def _on_touch(self, msg):
        self._touch = (self._position(msg), time.monotonic())

    def _on_franka(self, msg):
        self._franka = (self._position(msg), time.monotonic())

    def _fresh(self, sample, timeout_param):
        return (sample is not None and
                time.monotonic() - sample[1] < float(self.get_parameter(timeout_param).value))

    def _on_clutch(self, msg):
        if msg.data and not self._map.engaged:
            if not self._fresh(self._touch, 'touch_timeout'):
                self.get_logger().error('Cannot engage: no recent /touch/pose.')
                return
            if not self._fresh(self._franka, 'franka_timeout'):
                self.get_logger().error('Cannot engage: no recent /franka/pose.')
                return
            self._map.engage(self._touch[0], self._franka[0])
            t = ', '.join(f'{v:.3f}' for v in self._touch[0])
            f = ', '.join(f'{v:.3f}' for v in self._franka[0])
            self.get_logger().info(f'ENGAGED. Touch origin ({t}) m, Franka origin ({f}) m')
        elif not msg.data and self._map.engaged:
            self._release('clutch released')

    def _release(self, reason):
        self._map.release()
        self._pub_stop.publish(Twist())      # newest input -> franka_teleop stops at once
        self.get_logger().warn(f'RELEASED ({reason}).')

    def _on_timer(self):
        if not self._map.engaged:
            return
        if not self._fresh(self._touch, 'touch_timeout'):
            self._release('no recent /touch/pose')
            return
        x, y, z = self._map.target(self._touch[0])
        self._pub_target.publish(Point(x=x, y=y, z=z))

    def _publish_status(self):
        self._pub_status.publish(String(data='engaged' if self._map.engaged else 'released'))

    def shutdown(self):
        if self._map.engaged:
            self._release('node shutting down')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TouchToFranka()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
