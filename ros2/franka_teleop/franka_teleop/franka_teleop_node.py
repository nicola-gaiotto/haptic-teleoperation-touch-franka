#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Part of haptic-teleoperation-touch-franka - Copyright (c) 2026 Nicola Gaiotto
"""
franka_teleop - velocity teleoperation of the Franka Research 3 through franky.

    teleop_twist_keyboard --/franka/cmd_vel (Twist)--> franka_teleop --franky / FCI--> FR3
                                                           |
                                                           '--> /franka/pose, /franka/status

Safety behaviour:
  - starts DISABLED: the robot moves only after `/franka/enable` receives true;
  - linear velocity only (orientation fixed), norm limited to max_speed;
  - workspace box of +- workspace_half_size around the pose at enable time: velocity
    components pointing further out are set to zero;
  - commands older than cmd_timeout are ignored; every command is a short velocity motion
    (motion_duration_ms) so the robot stops by itself if commands stop arriving;
  - global franky dynamics factor (default 5 %) and collision thresholds;
  - optional go_home: at start-up, a slow joint motion to home_joints (Franka "ready" pose),
    so that the +- workspace box starts far from singularities. Lab, 2026-11-02: starting
    at (0.63, -0.06, 0.27) m, moving +10 cm right and +9 cm up ended with
    "cannot start at singular pose";
  - on any robot error the node disables itself; it recovers only through the
    `/franka/recover` service (std_srvs/Trigger), never automatically.

Topics and services:
  subscribes  <cmd_topic> (default /franka/cmd_vel)  geometry_msgs/Twist
  subscribes  /franka/enable                        std_msgs/Bool
  publishes   /franka/pose                          geometry_msgs/PoseStamped (frame base_frame)
  publishes   /franka/status                        std_msgs/String
  service     /franka/recover                       std_srvs/Trigger

Parameters: see config/franka_teleop.yaml.
"""
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from franka_teleop.franky_arm import FrankyArm
from franka_teleop.limits import apply_workspace, clamp_speed, is_zero, twist_to_velocity


class FrankaTeleop(Node):

    def __init__(self):
        super().__init__('franka_teleop')
        self.declare_parameter('robot_ip', '')
        self.declare_parameter('simulate', False)
        self.declare_parameter('visualize', True)
        self.declare_parameter('base_frame', 'fr3_link0')
        self.declare_parameter('cmd_topic', '/franka/cmd_vel')
        self.declare_parameter('linear_scale', 0.02)
        self.declare_parameter('lateral_from_angular', True)
        self.declare_parameter('angular_scale', 0.02)
        self.declare_parameter('operator_yaw_deg', 90.0)
        self.declare_parameter('max_speed', 0.02)
        self.declare_parameter('workspace_half_size', [0.10, 0.10, 0.10])
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('cmd_timeout', 0.55)
        self.declare_parameter('motion_duration_ms', 150)
        self.declare_parameter('dynamics_factor', 0.05)
        self.declare_parameter('torque_thresholds', [20.0] * 7)
        self.declare_parameter('force_thresholds', [30.0] * 6)
        self.declare_parameter('go_home', False)
        # Franka "ready" pose: arm well bent, far from singularities.
        self.declare_parameter('home_joints',
                               [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])

        simulate = bool(self.get_parameter('simulate').value)
        robot_ip = str(self.get_parameter('robot_ip').value)
        if not simulate and not robot_ip:
            raise RuntimeError("Set the parameter 'robot_ip' (or simulate:=true).")

        where = 'franky-sim' if simulate else robot_ip
        self.get_logger().info(f'Connecting to the robot ({where}) ...')
        self._arm = FrankyArm(
            robot_ip, simulate=simulate,
            visualize=bool(self.get_parameter('visualize').value),
            dynamics_factor=float(self.get_parameter('dynamics_factor').value),
            torque_thresholds=[float(v) for v in self.get_parameter('torque_thresholds').value],
            force_thresholds=[float(v) for v in self.get_parameter('force_thresholds').value],
            motion_duration_ms=int(self.get_parameter('motion_duration_ms').value))

        if bool(self.get_parameter('go_home').value):
            home = [float(v) for v in self.get_parameter('home_joints').value]
            self.get_logger().warn(
                'go_home: moving to the home joint pose ' + str(home) + ' (slow joint motion)')
            self._arm.move_joints(home)
            t, _ = self._arm.pose()
            self.get_logger().info(
                'Home reached. End effector at (' + ', '.join(f'{v:.3f}' for v in t) + ') m')

        self._enabled = False
        self._error = ''
        self._centre = None          # workspace centre = position at enable time
        self._vel_cmd = [0.0, 0.0, 0.0]
        self._last_cmd = None        # monotonic time of the last Twist
        self._moving = False         # True while velocity motions are being sent

        cmd_topic = str(self.get_parameter('cmd_topic').value)
        self.create_subscription(Twist, cmd_topic, self._on_twist, 10)
        self.create_subscription(Bool, '/franka/enable', self._on_enable, 10)
        self._pub_pose = self.create_publisher(PoseStamped, '/franka/pose', 10)
        self._pub_status = self.create_publisher(String, '/franka/status', 10)
        self.create_service(Trigger, '/franka/recover', self._on_recover)

        self.create_timer(1.0 / float(self.get_parameter('rate').value), self._on_timer)
        self.create_timer(0.1, self._publish_state)

        self.get_logger().info(
            f'Connected. Listening to {cmd_topic}. Robot DISABLED: publish true on '
            '/franka/enable to allow motion.')

    # ------------------------------------------------------------------ callbacks
    def _on_twist(self, msg):
        self._vel_cmd = twist_to_velocity(
            [msg.linear.x, msg.linear.y, msg.linear.z], msg.angular.z,
            float(self.get_parameter('linear_scale').value),
            bool(self.get_parameter('lateral_from_angular').value),
            float(self.get_parameter('angular_scale').value),
            float(self.get_parameter('operator_yaw_deg').value))
        self._last_cmd = time.monotonic()

    def _on_enable(self, msg):
        if msg.data and not self._enabled:
            if self._error:
                self.get_logger().error(
                    f'Cannot enable, robot error: {self._error}. Call /franka/recover first.')
                return
            self._centre, _ = self._arm.pose()
            self._last_cmd = None    # ignore commands sent before enabling
            self._enabled = True
            c = ', '.join(f'{v:.3f}' for v in self._centre)
            self.get_logger().info(f'ENABLED. Workspace centre = ({c}) m')
        elif not msg.data and self._enabled:
            self._disable('disabled by /franka/enable')

    def _on_recover(self, request, response):
        if self._enabled:
            response.success, response.message = False, 'Disable the robot first.'
            return response
        try:
            self._arm.wait()
        except Exception:            # the stored error of the failed motion
            pass
        try:
            self._arm.recover()
            self._error = ''
            response.success, response.message = True, 'Errors cleared. Robot still disabled.'
        except Exception as exc:     # noqa: BLE001 - report any franky error
            response.success, response.message = False, f'Recovery failed: {exc}'
        self.get_logger().info(response.message)
        return response

    def _on_timer(self):
        if not self._enabled:
            return
        now = time.monotonic()
        timeout = float(self.get_parameter('cmd_timeout').value)
        fresh = self._last_cmd is not None and now - self._last_cmd < timeout
        vel = self._vel_cmd if fresh else [0.0, 0.0, 0.0]
        vel = clamp_speed(vel, float(self.get_parameter('max_speed').value))
        try:
            position, _ = self._arm.pose()
            half = [float(v) for v in self.get_parameter('workspace_half_size').value]
            vel = apply_workspace(vel, position, self._centre, half)
            if not is_zero(vel):
                self._arm.send_velocity(vel)
                self._moving = True
            elif self._moving:
                self._arm.stop()     # one explicit zero-velocity motion, then nothing
                self._moving = False
        except Exception as exc:     # noqa: BLE001 - any franky error stops teleoperation
            self._error = str(exc)
            self._disable(f'robot error: {exc}')

    # ------------------------------------------------------------------ helpers
    def _disable(self, reason):
        self._enabled = False
        self._moving = False
        self._vel_cmd = [0.0, 0.0, 0.0]
        if not self._error:
            try:
                self._arm.stop()
            except Exception as exc:  # noqa: BLE001
                self._error = str(exc)
        self.get_logger().warn(f'DISABLED ({reason}).')

    def _publish_state(self):
        try:
            t, q = self._arm.pose()
        except Exception:            # noqa: BLE001 - state not available
            return
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter('base_frame').value)
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = t
        (msg.pose.orientation.x, msg.pose.orientation.y,
         msg.pose.orientation.z, msg.pose.orientation.w) = q
        self._pub_pose.publish(msg)
        state = 'ERROR: ' + self._error if self._error else (
            'enabled' if self._enabled else 'disabled')
        self._pub_status.publish(String(data=state))

    def shutdown(self):
        if self._enabled:
            self._disable('node shutting down')
        self._arm.close()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FrankaTeleop()
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
