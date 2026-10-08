# SPDX-License-Identifier: MIT
# Part of haptic-teleoperation-touch-franka - Copyright (c) 2026 Nicola Gaiotto
"""Unit tests of the kinematic model (no hardware needed).

Run from the workspace root after building:
    colcon test --packages-select touch_raw_driver && colcon test-result --verbose
"""
import json
import sys
import types

import numpy as np


def _import_driver():
    try:
        from touch_raw_driver import touch_raw_driver
        return touch_raw_driver
    except ImportError:
        pass
    # Outside a ROS 2 environment: stub the ROS / USB modules, so that the pure-math
    # classes can be imported without them.
    stubs = ['rclpy', 'rclpy.node', 'rclpy.signals', 'geometry_msgs', 'geometry_msgs.msg',
             'sensor_msgs', 'sensor_msgs.msg', 'std_msgs', 'std_msgs.msg',
             'usb', 'usb.core', 'usb.util']
    for name in stubs:
        sys.modules.setdefault(name, types.ModuleType(name))
    sys.modules['rclpy.node'].Node = object
    sys.modules['rclpy.signals'].SignalHandlerOptions = None
    for n in ('Point', 'PoseStamped'):
        setattr(sys.modules['geometry_msgs.msg'], n, object)
    sys.modules['sensor_msgs.msg'].JointState = object
    for n in ('Bool', 'Int32MultiArray', 'String'):
        setattr(sys.modules['std_msgs.msg'], n, object)
    from touch_raw_driver import touch_raw_driver
    return touch_raw_driver


def _model_file(tmp_path):
    # Values identified on the reference device (see docs/calibration.md).
    model = {
        'enc_to_q': [[-4.112e-4, 0, 0], [0, -4.0803e-4, 0], [0, 0, 4.0749e-4], [0, 0, 0]],
        'fk_structure': 'A', 'L1_mm': 133.35, 'L2_mm': 133.35,
        'offset_mm': [0.0, 23.3, -168.3],
        'encoder_range_min': [-2397, -4378, -910], 'encoder_range_max': [2355, 11, 4532],
    }
    path = tmp_path / 'kinematics.json'
    path.write_text(json.dumps(model))
    return str(path)


def test_inkwell_position(tmp_path):
    drv = _import_driver()
    kin = drv.Kinematics(_model_file(tmp_path))
    p = drv.Kinematics.oh_to_ros(kin.fk(kin.raw_to_q([0, 659, -910])))
    # Inkwell position measured with the OpenHaptics-based driver (Deliverable 1).
    assert np.allclose(p, [0.0, 0.0881, -0.0655], atol=1e-3)


def test_ik_round_trip(tmp_path):
    drv = _import_driver()
    kin = drv.Kinematics(_model_file(tmp_path))
    for target in ([0.0, 0.0, 0.0], [0.05, 0.0, 0.0], [0.0, 0.03, 0.02], [-0.06, -0.02, -0.03]):
        q, reachable = kin.ik(drv.Kinematics.ros_to_oh(target))
        assert reachable
        back = drv.Kinematics.oh_to_ros(kin.fk(kin.raw_to_q(kin.q_to_raw(q))))
        assert np.allclose(back, target, atol=1e-4)
