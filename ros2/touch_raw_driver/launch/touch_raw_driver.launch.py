# SPDX-License-Identifier: MIT
# Part of haptic-teleoperation-touch-franka - Copyright (c) 2026 Nicola Gaiotto
"""Start touch_raw_driver with the parameters in config/touch_raw_driver.yaml."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('touch_raw_driver'),
                          'config', 'touch_raw_driver.yaml')
    return LaunchDescription([
        Node(
            package='touch_raw_driver',
            executable='touch_raw_driver',
            name='touch_raw_driver',
            output='screen',
            parameters=[params],
        ),
    ])
