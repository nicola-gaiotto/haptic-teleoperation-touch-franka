# SPDX-License-Identifier: MIT
# Part of haptic-teleoperation-touch-franka - Copyright (c) 2026 Nicola Gaiotto
"""Start franka_teleop with config/franka_teleop.yaml.

    ros2 launch franka_teleop franka_teleop.launch.py simulate:=true
    ros2 launch franka_teleop franka_teleop.launch.py robot_ip:=<FCI IP>

The keyboard needs its own terminal:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/franka/cmd_vel
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(get_package_share_directory('franka_teleop'),
                          'config', 'franka_teleop.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', default_value=''),
        DeclareLaunchArgument('simulate', default_value='false'),
        Node(
            package='franka_teleop', executable='franka_teleop', output='screen',
            parameters=[config, {
                'robot_ip': ParameterValue(LaunchConfiguration('robot_ip'), value_type=str),
                'simulate': ParameterValue(LaunchConfiguration('simulate'), value_type=bool),
            }]),
    ])
