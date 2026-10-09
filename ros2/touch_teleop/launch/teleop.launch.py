# SPDX-License-Identifier: MIT
# Part of haptic-teleoperation-touch-franka - Copyright (c) 2026 Nicola Gaiotto
"""Keyboard teleoperation of the Touch: driver + velocity bridge + GUI.

    ros2 launch touch_teleop teleop.launch.py                 # dashboard (default)
    ros2 launch touch_teleop teleop.launch.py rviz:=true      # also RViz (3D view)
    ros2 launch touch_teleop teleop.launch.py dashboard:=false

The keyboard node needs its own terminal (it reads the keys from it):
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/touch/cmd_vel
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    driver_params = os.path.join(get_package_share_directory('touch_raw_driver'),
                                 'config', 'touch_raw_driver.yaml')
    teleop_share = get_package_share_directory('touch_teleop')
    teleop_params = os.path.join(teleop_share, 'config', 'teleop.yaml')
    rviz_config = os.path.join(teleop_share, 'rviz', 'touch_teleop.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('dashboard', default_value='true',
                              description='Start the 2D dashboard (arrow-key control)'),
        DeclareLaunchArgument('rviz', default_value='false', description='Start RViz'),

        Node(package='touch_raw_driver', executable='touch_raw_driver',
             name='touch_raw_driver', output='screen', parameters=[driver_params]),

        Node(package='touch_teleop', executable='twist_to_target',
             name='twist_to_target', output='screen', parameters=[teleop_params]),

        # Fixed 'world' frame with the Touch base at its origin (the Franka will be added
        # to the same 'world' later).
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='world_to_touch_base',
             arguments=['--frame-id', 'world', '--child-frame-id', 'touch_base']),

        Node(package='touch_teleop', executable='touch_dashboard', name='touch_dashboard',
             output='screen', parameters=[teleop_params],
             condition=IfCondition(LaunchConfiguration('dashboard'))),

        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', rviz_config], condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
