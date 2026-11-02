# SPDX-License-Identifier: MIT
# Part of haptic-teleoperation-touch-franka - Copyright (c) 2026 Nicola Gaiotto
from glob import glob

from setuptools import find_packages, setup

package_name = 'franka_teleop'

setup(
    name=package_name,
    version='0.4.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nicola Gaiotto',
    maintainer_email='your.email@example.com',
    description='Velocity teleoperation of the Franka Research 3 through franky.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'franka_teleop = franka_teleop.franka_teleop_node:main',
        ],
    },
)
