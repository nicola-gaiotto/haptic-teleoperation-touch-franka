#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Part of haptic-teleoperation-touch-franka - Copyright (c) 2026 Nicola Gaiotto
"""
touch_model_bridge - drives the 3D model of the Touch (omni_description URDF) in RViz.

Converts the joint angles published by touch_raw_driver (OpenHaptics convention) into
the joint states of the URDF model, with the same conversion used by the third-party
Deliverable 1 driver (Geomagic_Touch_ROS2, omni_state.cpp):

    waist    = -q0
    shoulder =  q1
    elbow    =  q2 - q1 - ELBOW_OFFSET   (the URDF elbow is relative to the upper arm)
    yaw      = -g0 + pi
    pitch    = -g1 - 3 pi / 4
    roll     = -g2 - pi

ELBOW_OFFSET = pi/2 - 1.5 corrects the URDF, whose elbow origin is rotated by -1.5 rad
instead of -pi/2 (not done in the Deliverable 1 driver). With it, and with the base
transform touch_base -> touch/base used in teleop.launch.py, the model places the gimbal
centre within 1 mm of the measured kinematics (about 1 cm without it).

If the gimbal angles are not available, the stylus is shown in a fixed neutral pose.

Topics:
  subscribes  /touch/joint_states        sensor_msgs/JointState (touch_raw_driver)
  publishes   /touch/model_joint_states  sensor_msgs/JointState (robot_state_publisher)
"""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

URDF_JOINTS = ['waist', 'shoulder', 'elbow', 'yaw', 'pitch', 'roll']
NEUTRAL_GIMBAL = (0.0, 0.0, 0.0)
ELBOW_OFFSET = math.pi / 2.0 - 1.5


class TouchModelBridge(Node):

    def __init__(self):
        super().__init__('touch_model_bridge')
        self.create_subscription(JointState, '/touch/joint_states', self._on_joints, 10)
        self._pub = self.create_publisher(JointState, '/touch/model_joint_states', 10)

    def _on_joints(self, msg):
        values = dict(zip(msg.name, msg.position))
        try:
            q = [values[f'touch_joint{i}'] for i in range(3)]
        except KeyError:
            return
        g = [values.get(f'touch_gimbal{i}', NEUTRAL_GIMBAL[i]) for i in range(3)]
        out = JointState()
        out.header.stamp = msg.header.stamp
        out.name = list(URDF_JOINTS)
        out.position = [
            -q[0],
            q[1],
            q[2] - q[1] - ELBOW_OFFSET,
            -g[0] + math.pi,
            -g[1] - 3.0 * math.pi / 4.0,
            -g[2] - math.pi,
        ]
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TouchModelBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
