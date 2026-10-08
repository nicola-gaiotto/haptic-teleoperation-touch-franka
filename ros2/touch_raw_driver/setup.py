# SPDX-License-Identifier: MIT
# Part of haptic-teleoperation-touch-franka - Copyright (c) 2026 Nicola Gaiotto
from glob import glob

from setuptools import find_packages, setup

package_name = 'touch_raw_driver'

setup(
    name=package_name,
    version='0.2.0',
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
    description='ROS 2 driver for the 3D Systems Touch commanding the motors directly over USB.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'touch_raw_driver = touch_raw_driver.touch_raw_driver:main',
        ],
    },
)
