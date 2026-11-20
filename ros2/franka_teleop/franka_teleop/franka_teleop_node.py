#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Part of haptic-teleoperation-touch-franka - Copyright (c) 2026 Nicola Gaiotto
"""
franka_teleop - velocity teleoperation of the Franka Research 3 through franky.

    teleop_twist_keyboard --/franka/cmd_vel (Twist)--> franka_teleop --franky / FCI--> FR3
    touch_to_franka ------/franka/target (Point)---->      |
                                                           '--> /franka/pose, /franka/status

Two inputs, the most recent fresh one is used:
  - velocity: Twist in the operator frame (keyboard);
  - position: Point in the robot base frame; a proportional loop turns it into a velocity,
    v = position_gain * (target - position), clipped to the workspace box. The loop runs
    here, at `rate`, with the robot pose read at the same tick.

Safety behaviour:
  - starts DISABLED: the robot moves only after `/franka/enable` receives true;
  - linear velocity only (orientation fixed), norm limited to max_speed;
  - workspace box of +- workspace_half_size around the pose at enable time: velocity
    components pointing further out are set to zero;
  - commands older than cmd_timeout are ignored; every command is a short velocity motion
    (motion_duration_ms) so the robot stops by itself if commands stop arriving;
  - global franky dynamics factor (default 5 %) and collision thresholds;
  - contact force limit: above contact_force_limit [N] (environment force on the end effector,
    O_F_ext_hat_K minus its value at enable), no motion further into the contact, and a
    small motion back proportional to the excess (contact_backoff_gain);
  - optional go_home: at start-up, a slow joint motion to home_joints (Franka "ready" pose),
    so that the +- workspace box starts far from singularities. Lab, 2026-11-02: starting
    at (0.63, -0.06, 0.27) m, moving +10 cm right and +9 cm up ended with
    "cannot start at singular pose";
  - on any robot error the node disables itself; it recovers only through the
    `/franka/recover` service (std_srvs/Trigger), never automatically.

Topics and services:
  subscribes  <cmd_topic> (default /franka/cmd_vel)  geometry_msgs/Twist
  subscribes  <target_topic> (default /franka/target) geometry_msgs/Point (frame base_frame)
  subscribes  /franka/enable                        std_msgs/Bool
  publishes   /franka/pose                          geometry_msgs/PoseStamped (frame base_frame)
  publishes   /franka/status                        std_msgs/String
  publishes   /franka/workspace_centre              geometry_msgs/Point (box centre, when enabled)
  publishes   /franka/wrench_ext                    geometry_msgs/WrenchStamped, estimated external
                                                    wrench O_F_ext_hat_K (base frame), at wrench_rate
  service     /franka/recover                       std_srvs/Trigger

Parameters: see config/franka_teleop.yaml.
"""
import time

import rclpy
from geometry_msgs.msg import Point, PoseStamped, Twist, WrenchStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from franka_teleop.franky_arm import FrankyArm
from franka_teleop.limits import (apply_workspace, clamp_speed, clip_to_box, is_zero,
                                  limit_contact, position_to_velocity, twist_to_velocity)


class FrankaTeleop(Node):

    def __init__(self):
        super().__init__('franka_teleop')
        self.declare_parameter('robot_ip', '')
        self.declare_parameter('simulate', False)
        self.declare_parameter('visualize', True)
        self.declare_parameter('base_frame', 'fr3_link0')
        self.declare_parameter('cmd_topic', '/franka/cmd_vel')
        self.declare_parameter('target_topic', '/franka/target')
        self.declare_parameter('position_gain', 3.0)      # [1/s] position loop
        self.declare_parameter('position_deadband', 0.001)  # [m] hold still this close
        self.declare_parameter('state_rate', 10.0)        # [Hz] /franka/pose and /status
        self.declare_parameter('wrench_rate', 100.0)      # [Hz] /franka/wrench_ext (0 = off)
        # Contact force limit [N] (0 = off): above it, motion further into the contact is
        # blocked. Lab 2026-11-20: a contact went from 1 to 30 N in 0.5 s (collision reflex);
        # tested at 10 N.
        self.declare_parameter('contact_force_limit', 10.0)
        # Above the limit, move back at contact_backoff_gain [m/s per N of excess], at most
        # contact_max_backoff [m/s] (0 = only stop). Lab 2026-11-20 (bag contact5_1629):
        # without it the force stayed above the limit, up to 14.7 N.
        self.declare_parameter('contact_backoff_gain', 0.001)
        self.declare_parameter('contact_max_backoff', 0.005)
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
        self._target = None          # position target [m], base frame
        self._last_target = None     # monotonic time of the last target
        self._moving = False         # True while velocity motions are being sent
        self._wrench_bias = None     # O_F_ext_hat_K force at enable time

        cmd_topic = str(self.get_parameter('cmd_topic').value)
        self.create_subscription(Twist, cmd_topic, self._on_twist, 10)
        target_topic = str(self.get_parameter('target_topic').value)
        self.create_subscription(Point, target_topic, self._on_target, 10)
        self.create_subscription(Bool, '/franka/enable', self._on_enable, 10)
        self._pub_pose = self.create_publisher(PoseStamped, '/franka/pose', 10)
        self._pub_status = self.create_publisher(String, '/franka/status', 10)
        self._pub_centre = self.create_publisher(Point, '/franka/workspace_centre', 10)
        self._pub_wrench = self.create_publisher(WrenchStamped, '/franka/wrench_ext', 10)
        self._wrench_warned = False
        self.create_service(Trigger, '/franka/recover', self._on_recover)

        self.create_timer(1.0 / float(self.get_parameter('rate').value), self._on_timer)
        self.create_timer(1.0 / float(self.get_parameter('state_rate').value),
                          self._publish_state)
        wrench_rate = float(self.get_parameter('wrench_rate').value)
        if wrench_rate > 0.0:
            self.create_timer(1.0 / wrench_rate, self._publish_wrench)

        self.get_logger().info(
            f'Connected. Listening to {cmd_topic} and {target_topic}. Robot DISABLED: publish true on '
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

    def _on_target(self, msg):
        self._target = [msg.x, msg.y, msg.z]
        self._last_target = time.monotonic()

    def _on_enable(self, msg):
        if msg.data and not self._enabled:
            if self._error:
                self.get_logger().error(
                    f'Cannot enable, robot error: {self._error}. Call /franka/recover first.')
                return
            self._centre, _ = self._arm.pose()
            self._wrench_bias = None
            try:
                self._wrench_bias = self._arm.external_wrench()[:3]
            except Exception as exc:  # noqa: BLE001 - contact limit off without the wrench
                self.get_logger().warn(f'No external wrench ({exc}): contact limit OFF.')
            self._last_cmd = None    # ignore commands sent before enabling
            self._last_target = None
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
        fresh_vel = self._last_cmd is not None and now - self._last_cmd < timeout
        fresh_target = self._last_target is not None and now - self._last_target < timeout
        if fresh_target and fresh_vel:   # both inputs active: the most recent one wins
            fresh_target = self._last_target > self._last_cmd
        try:
            position, _ = self._arm.pose()
            half = [float(v) for v in self.get_parameter('workspace_half_size').value]
            if fresh_target:
                target = clip_to_box(self._target, self._centre, half)
                vel = position_to_velocity(
                    target, position, float(self.get_parameter('position_gain').value),
                    float(self.get_parameter('position_deadband').value))
            else:
                vel = self._vel_cmd if fresh_vel else [0.0, 0.0, 0.0]
            vel = clamp_speed(vel, float(self.get_parameter('max_speed').value))
            vel = apply_workspace(vel, position, self._centre, half)
            limit = float(self.get_parameter('contact_force_limit').value)
            if limit > 0.0 and self._wrench_bias is not None and not is_zero(vel):
                w = self._arm.external_wrench()[:3]
                f_env = [-(a - b) for a, b in zip(w, self._wrench_bias)]
                vel, limited = limit_contact(
                    vel, f_env, limit,
                    float(self.get_parameter('contact_backoff_gain').value),
                    float(self.get_parameter('contact_max_backoff').value))
                if limited:
                    self.get_logger().warn(
                        'Contact force above the limit ('
                        + ', '.join(f'{v:+.1f}' for v in f_env) + ' N): not pushing further.',
                        throttle_duration_sec=1.0)
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
        self._target = None
        self._last_target = None
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
        if self._enabled and self._centre is not None:
            c = self._centre
            self._pub_centre.publish(Point(x=float(c[0]), y=float(c[1]), z=float(c[2])))

    def _publish_wrench(self):
        try:
            w = self._arm.external_wrench()
        except Exception as exc:     # noqa: BLE001 - not available (e.g. old franky, sim)
            if not self._wrench_warned:
                self.get_logger().warn(f'External wrench not available: {exc}')
                self._wrench_warned = True
            return
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter('base_frame').value)
        msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z = w[:3]
        msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z = w[3:]
        self._pub_wrench.publish(msg)

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
