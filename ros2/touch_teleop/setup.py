# SPDX-License-Identifier: MIT
# Part of haptic-teleoperation-touch-franka - Copyright (c) 2026 Nicola Gaiotto
from glob import glob

from setuptools import find_packages, setup

package_name = 'touch_teleop'

setup(
    name=package_name,
    version='0.3.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nicola Gaiotto',
    maintainer_email='your.email@example.com',
    description='Velocity (keyboard) teleoperation of the 3D Systems Touch.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'twist_to_target = touch_teleop.twist_to_target:main',
            'touch_dashboard = touch_teleop.touch_dashboard:main',
            'touch_model_bridge = touch_teleop.touch_model_bridge:main',
        ],
    },
)
