# SPDX-License-Identifier: MIT
# Part of haptic-teleoperation-touch-franka - Copyright (c) 2026 Nicola Gaiotto
"""Touch -> Franka position teleoperation: franka_teleop + touch_to_franka.

    ros2 launch franka_teleop touch_to_franka.launch.py simulate:=true
    ros2 launch franka_teleop touch_to_franka.launch.py robot_ip:=<FCI IP> go_home:=true

The Touch driver runs separately (its own terminal), in float mode:
    ros2 launch touch_raw_driver touch_raw_driver.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('franka_teleop')
    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', default_value=''),
        DeclareLaunchArgument('simulate', default_value='false'),
        DeclareLaunchArgument('go_home', default_value='false'),
        Node(
            package='franka_teleop', executable='franka_teleop', output='screen',
            parameters=[os.path.join(share, 'config', 'franka_teleop.yaml'), {
                'robot_ip': ParameterValue(LaunchConfiguration('robot_ip'), value_type=str),
                'simulate': ParameterValue(LaunchConfiguration('simulate'), value_type=bool),
                'go_home': ParameterValue(LaunchConfiguration('go_home'), value_type=bool),
            }]),
        Node(
            package='franka_teleop', executable='touch_to_franka', output='screen',
            parameters=[os.path.join(share, 'config', 'touch_to_franka.yaml')]),
    ])
