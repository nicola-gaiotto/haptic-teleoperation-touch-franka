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
  subscribes  /franka/workspace_centre  geometry_msgs/Point (centre of the franka_teleop box)
  publishes   /touch/force       geometry_msgs/Vector3Stamped, virtual walls (if walls_enabled)

Virtual walls (walls_enabled): while engaged, if the target leaves the franka_teleop box
(centre from /franka/workspace_centre, half size wall_half_size), a spring force
-wall_stiffness * excess is sent to the Touch (touch_raw_driver in 'force' mode), limited to
wall_max_force.
Force feedback (force_feedback_enabled): the FR3 external force (/franka/wrench_ext) minus its
value at engage time, with the sign of the force the environment applies to the robot, low-pass
filtered, with a dead zone and scaled by ff_gain (mapping.ForceFeedback), is added to the wall
force. The total is limited to max_force. fake_franka: no robot needed; the Franka position is wall_centre (for tests
of the walls with the Touch only; nothing listens to /franka/target).

Parameters: see config/touch_to_franka.yaml.
"""
import time

import rclpy
from geometry_msgs.msg import Point, PoseStamped, Twist, Vector3Stamped, WrenchStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String

from franka_teleop.mapping import ClutchMapping, ForceFeedback, limit_norm, wall_force


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
        self.declare_parameter('walls_enabled', False)
        self.declare_parameter('wall_stiffness', 300.0)        # [N/m]
        self.declare_parameter('wall_max_force', 1.5)          # [N]
        self.declare_parameter('wall_half_size', [0.10, 0.10, 0.10])   # [m], as franka_teleop
        self.declare_parameter('fake_franka', False)
        self.declare_parameter('wall_centre', [0.307, 0.0, 0.487])     # [m], ready pose
        self.declare_parameter('force_feedback_enabled', False)
        self.declare_parameter('ff_gain', 0.05)               # Touch N per robot N
        self.declare_parameter('ff_deadband', 0.5)            # [N] robot side
        self.declare_parameter('ff_filter_alpha', 0.2)        # low-pass, per wrench sample
        self.declare_parameter('max_force', 1.5)              # [N] total on the Touch
        self.declare_parameter('wrench_timeout', 0.1)         # [s]

        self._map = ClutchMapping(float(self.get_parameter('scale').value),
                                  float(self.get_parameter('yaw_deg').value))
        self._touch = None           # (position, monotonic time)
        self._franka = None
        self._centre = None          # (position, monotonic time) from franka_teleop
        self._wall_active = False
        self._wrench = None          # (force [N], monotonic time)
        self._ff = ForceFeedback(
            float(self.get_parameter('ff_gain').value),
            float(self.get_parameter('ff_deadband').value),
            float(self.get_parameter('ff_filter_alpha').value),
            float(self.get_parameter('max_force').value),
            self._map.yaw_deg)
        self._ff_force = [0.0, 0.0, 0.0]

        self.create_subscription(PoseStamped, '/touch/pose', self._on_touch, 10)
        self.create_subscription(PoseStamped, '/franka/pose', self._on_franka, 10)
        self.create_subscription(Bool, str(self.get_parameter('clutch_topic').value),
                                 self._on_clutch, 10)
        self._pub_target = self.create_publisher(
            Point, str(self.get_parameter('target_topic').value), 10)
        self._pub_stop = self.create_publisher(Twist, '/franka/cmd_vel', 10)
        self._pub_status = self.create_publisher(String, '/touch_to_franka/status', 10)
        self.create_subscription(Point, '/franka/workspace_centre', self._on_centre, 10)
        self._pub_force = self.create_publisher(Vector3Stamped, '/touch/force', 10)
        self.create_subscription(WrenchStamped, '/franka/wrench_ext', self._on_wrench, 10)
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

    def _on_centre(self, msg):
        self._centre = ([msg.x, msg.y, msg.z], time.monotonic())

    def _on_wrench(self, msg):
        f = msg.wrench.force
        self._wrench = ([f.x, f.y, f.z], time.monotonic())
        # Gain, dead zone and filter can be changed at runtime with `ros2 param set`.
        self._ff.gain = float(self.get_parameter('ff_gain').value)
        self._ff.deadband = float(self.get_parameter('ff_deadband').value)
        self._ff.alpha = float(self.get_parameter('ff_filter_alpha').value)
        self._ff.max_force = float(self.get_parameter('max_force').value)
        if self._map.engaged:
            self._ff_force = self._ff.update(self._wrench[0])

    def _franka_sample(self):
        if bool(self.get_parameter('fake_franka').value):
            c = [float(v) for v in self.get_parameter('wall_centre').value]
            return (c, time.monotonic())
        return self._franka

    def _wall_centre(self):
        if bool(self.get_parameter('fake_franka').value):
            return [float(v) for v in self.get_parameter('wall_centre').value]
        if self._centre is not None and time.monotonic() - self._centre[1] < 1.0:
            return self._centre[0]
        return None

    def _publish_force(self, f):
        m = Vector3Stamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'touch_base'
        m.vector.x, m.vector.y, m.vector.z = (float(v) for v in f)
        self._pub_force.publish(m)

    def _fresh(self, sample, timeout_param):
        return (sample is not None and
                time.monotonic() - sample[1] < float(self.get_parameter(timeout_param).value))

    def _on_clutch(self, msg):
        if msg.data and not self._map.engaged:
            if not self._fresh(self._touch, 'touch_timeout'):
                self.get_logger().error('Cannot engage: no recent /touch/pose.')
                return
            franka = self._franka_sample()
            if not self._fresh(franka, 'franka_timeout'):
                self.get_logger().error('Cannot engage: no recent /franka/pose.')
                return
            self._map.engage(self._touch[0], franka[0])
            if bool(self.get_parameter('force_feedback_enabled').value):
                if not self._fresh(self._wrench, 'wrench_timeout'):
                    self._map.release()
                    self.get_logger().error('Cannot engage: force feedback on but no recent '
                                            '/franka/wrench_ext.')
                    return
                self._ff.set_bias(self._wrench[0])
                b = ', '.join(f'{v:+.2f}' for v in self._wrench[0])
                self.get_logger().info(f'Force bias ({b}) N')
            t = ', '.join(f'{v:.3f}' for v in self._touch[0])
            f = ', '.join(f'{v:.3f}' for v in franka[0])
            self.get_logger().info(f'ENGAGED. Touch origin ({t}) m, Franka origin ({f}) m')
        elif not msg.data and self._map.engaged:
            self._release('clutch released')

    def _release(self, reason):
        self._map.release()
        self._ff.reset()
        self._ff_force = [0.0, 0.0, 0.0]
        if (bool(self.get_parameter('walls_enabled').value) or
                bool(self.get_parameter('force_feedback_enabled').value)):
            self._publish_force([0.0, 0.0, 0.0])
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
        walls = bool(self.get_parameter('walls_enabled').value)
        ff = bool(self.get_parameter('force_feedback_enabled').value)
        if not (walls or ff):
            return
        f = [0.0, 0.0, 0.0]
        if ff:
            if self._fresh(self._wrench, 'wrench_timeout'):
                f = list(self._ff_force)
            else:
                self.get_logger().warn('No recent /franka/wrench_ext: no force feedback.',
                                       throttle_duration_sec=2.0)
        if walls:
            centre = self._wall_centre()
            if centre is None:
                self.get_logger().warn('Walls: no /franka/workspace_centre (enable franka_teleop).',
                                       throttle_duration_sec=2.0)
                fw = [0.0, 0.0, 0.0]
            else:
                fw = wall_force([x, y, z], centre,
                                [float(v) for v in self.get_parameter('wall_half_size').value],
                                float(self.get_parameter('wall_stiffness').value),
                                self._map.yaw_deg,
                                float(self.get_parameter('wall_max_force').value))
            active = any(v != 0.0 for v in fw)
            if active and not self._wall_active:
                self.get_logger().info('Wall contact: force ('
                                       + ', '.join(f'{v:+.2f}' for v in fw) + ') N')
            self._wall_active = active
            f = [a + b for a, b in zip(f, fw)]
        self._publish_force(limit_norm(f, float(self.get_parameter('max_force').value)))

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
