#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Part of haptic-teleoperation-touch-franka - Copyright (c) 2026 Nicola Gaiotto
"""
touch_raw_driver - ROS 2 driver for the 3D Systems Touch (HID, USB ID 2988:0304) that
commands the motors DIRECTLY over USB, without OpenHaptics (which, on this system,
always sends zero current to the motors).

PHASE 1 (Deliverable 2): joint-space control, in raw encoder counts.
PHASE 2: Cartesian interface (pose and target in metres, same frame as /phantom/pose),
using the kinematic model identified by kin_logger + kin_fit.py.
Gravity compensation: physical model identified by gravity_calibration.py
(gravity_file), applied with gain gravity_gain.

USB protocol (decoded with usbmon and verified with motor tests):
  PC -> Touch, EP 0x02, report 10:
      0x0a | counter u32 | motor0 u16 | motor1 u16 | motor2 u16 | 0x03
      2048 (0x0800) = zero current
  Touch -> PC, EP 0x81, report 11:
      0x0b | u32 | counter echo u32 | 13 x int16 (enc0..enc5, ...)

Motor/encoder signs, measured with characterize_motors.py:
  motor 0 -> enc0, opposite sign
  motor 1 -> enc1, opposite sign
  motor 2 -> enc2, same sign

Topics:
  publishes   /touch/joint_counts  std_msgs/Int32MultiArray  [enc0, enc1, enc2]
  publishes   /touch/encoders      std_msgs/Int32MultiArray  [enc0 .. enc5] (for phase 2)
  publishes   /touch/motor_cmd     std_msgs/Int32MultiArray  motor command (offset from 2048)
  subscribes  /touch/joint_target  std_msgs/Int32MultiArray  [t0, t1, t2]
  subscribes  /touch/enable        std_msgs/Bool
  subscribes  /touch/mode          std_msgs/String  'position' (default) or 'float'
  with a kinematic model (kinematics_file):
  publishes   /touch/pose          geometry_msgs/PoseStamped  stylus position [m], frame
                                   'touch_base' (x right, y forward, z up, as /phantom/pose);
                                   orientation not yet computed (identity)
  publishes   /touch/joint_states  sensor_msgs/JointState     joint angles q0..q2 and gimbal
                                   angles g0..g2 [rad] (OpenHaptics convention)
  subscribes  /touch/target        geometry_msgs/Point        Cartesian target [m], same frame

Modes (when enabled):
  position : PID towards the joint target + gravity compensation
  float    : gravity compensation only; the stylus can be moved freely by hand
             and stays where it is left
Gravity model: JSON file written by gravity_calibration.py (parameter gravity_file).
If the file is missing, the node works without gravity compensation.

Safety:
  - starts DISABLED (zero current);
  - on enable, the target is set to the current position (no jump);
  - the reference moves towards the target at a limited speed (max_ref_speed);
  - the command is saturated at max_delta (hard limit 600);
  - thermal protection: if the RMS command of a motor over the last rms_time_constant
    seconds exceeds rms_limit, that motor is capped at 90 % of rms_limit until its RMS
    falls below 85 % (hysteresis, no drop-and-recover cycles);
  - anti-windup: the integral is frozen while the command is saturated;
  - zero current when disabled, when no data is received, on USB errors and on Ctrl+C.
  EMERGENCY STOP: Ctrl+C in the node's terminal, or unplug the Touch power supply.
"""
import json
import math
import os
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Point, PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Int32MultiArray, String

import numpy as np

import usb.core
import usb.util

VID, PID = 0x2988, 0x0304
EP_OUT, EP_IN = 0x02, 0x81
ZERO = 0x0800
FLAGS = 0x03
HARD_MAX_DELTA = 600
MOTOR_SIGN = (-1, -1, +1)


class TouchUSB:
    """Layer 1: direct USB communication with the Touch."""

    def __init__(self):
        dev = usb.core.find(idVendor=VID, idProduct=PID)
        if dev is None:
            raise RuntimeError('Touch not found (check cable, power supply and lsusb).')
        try:
            if dev.is_kernel_driver_active(0):
                dev.detach_kernel_driver(0)
        except (usb.core.USBError, NotImplementedError):
            pass
        usb.util.claim_interface(dev, 0)
        self.dev = dev
        self.counter = 0

    def exchange(self, deltas):
        """Send one motor command (offsets from 2048) and read the device state.

        Returns the tuple (enc0..enc5), or None if no valid state report arrived.
        """
        cmd = [ZERO + int(d) for d in deltas]
        pkt = struct.pack('<BIHHHB', 0x0A, self.counter & 0xFFFFFFFF,
                          cmd[0], cmd[1], cmd[2], FLAGS)
        self.dev.write(EP_OUT, pkt, timeout=100)
        self.counter += 1
        try:
            data = bytes(self.dev.read(EP_IN, 64, timeout=20))
        except usb.core.USBTimeoutError:
            return None
        if len(data) >= 35 and data[0] == 0x0B:
            return struct.unpack('<13h', data[9:35])[:6]
        return None

    def send_zero(self, n=50):
        """Send zero current several times, so the device surely ends at rest."""
        for _ in range(n):
            try:
                self.exchange([0, 0, 0])
            except usb.core.USBError:
                pass
            time.sleep(0.002)

    def close(self):
        usb.util.release_interface(self.dev, 0)
        usb.util.dispose_resources(self.dev)


# Raw USB encoders vs OpenHaptics encoders:
#   enc1: raw = -OpenHaptics (verified in the inkwell, -659 vs +659, and on the ranges)
#   enc0: raw = -OpenHaptics (found with the x-direction test: enc0 is 0 in the
#         inkwell, so the inkwell check could not reveal it)
#   enc2: identical (-909 vs -910 in the inkwell)
RAW_TO_OH_SIGN = (-1, -1, 1)
# Gimbal sensors (enc3..enc5): raw USB values are assumed equal to the OpenHaptics ones
# (positive potentiometer readings, about 2000-2700 in the inkwell). To be verified by
# comparing the inkwell values of /touch/encoders with kin_log.csv.
RAW_TO_OH_GIMBAL_SIGN = (1, 1, 1)


class Kinematics:
    """Layer 3: raw encoders <-> joint angles <-> stylus position.

    Model identified by kin_fit.py on OpenHaptics data (structure A):
        q = A e + b                (e = OpenHaptics encoders, q = joint angles [rad])
        r = L1 cos q1 + L2 sin q2
        x = -sin(q0) r + x0,  y = L1 sin q1 - L2 cos q2 + y0,  z = cos(q0) r + z0   [mm]
    in the OpenHaptics frame (x right, y up, z towards the operator).
    The ROS frame is the one of /phantom/pose: x_ros = x, y_ros = -z, z_ros = y [m].
    """

    def __init__(self, path):
        with open(path) as f:
            m = json.load(f)
        if m.get('fk_structure') != 'A':
            raise ValueError('only FK structure A is supported')
        W = np.array(m['enc_to_q'], dtype=float)          # 4 x 3
        self.A = W[:3, :].T                               # q = A e + b
        self.b = W[3, :]
        self.A_inv = np.linalg.inv(self.A)
        self.L1 = float(m['L1_mm'])
        self.L2 = float(m['L2_mm'])
        self.x0, self.y0, self.z0 = (float(v) for v in m['offset_mm'])
        emin = np.array(m['encoder_range_min'], dtype=float)
        emax = np.array(m['encoder_range_max'], dtype=float)
        margin = 0.05 * (emax - emin)                     # stay 5 % inside the recorded range
        self.emin, self.emax = emin + margin, emax - margin
        # Gimbal (stylus orientation sensors), optional: g = G e3 + c
        self.G = self.c = None
        if 'enc_to_gimbal' in m:
            Wg = np.array(m['enc_to_gimbal'], dtype=float)
            self.G, self.c = Wg[:3, :].T, Wg[3, :]

    # --- encoders <-> joint angles
    def raw_to_q(self, raw):
        e = np.array([raw[i] * RAW_TO_OH_SIGN[i] for i in range(3)], dtype=float)
        return self.A @ e + self.b

    def raw_to_gimbal(self, raw6):
        """Gimbal angles [rad] (same convention as HD_CURRENT_GIMBAL_ANGLES), or None."""
        if self.G is None:
            return None
        e = np.array([raw6[3 + i] * RAW_TO_OH_GIMBAL_SIGN[i] for i in range(3)], dtype=float)
        return self.G @ e + self.c

    def q_to_raw(self, q):
        e = self.A_inv @ (np.asarray(q, dtype=float) - self.b)
        e = np.clip(e, self.emin, self.emax)              # never ask for an unreachable joint
        return [float(e[i] * RAW_TO_OH_SIGN[i]) for i in range(3)]

    # --- joint angles <-> position (OpenHaptics frame, mm)
    def fk(self, q):
        q0, q1, q2 = q
        r = self.L1 * np.cos(q1) + self.L2 * np.sin(q2)
        return np.array([-np.sin(q0) * r + self.x0,
                         self.L1 * np.sin(q1) - self.L2 * np.cos(q2) + self.y0,
                         np.cos(q0) * r + self.z0])

    def ik(self, p):
        """Analytic inverse kinematics. Returns (q, reachable)."""
        x, y, z = p[0] - self.x0, p[1] - self.y0, p[2] - self.z0
        q0 = np.arctan2(-x, z)
        r = np.hypot(x, z)
        h = y
        # With beta = q2 - pi/2 the arm is a planar 2-link chain with absolute angles:
        #   r = L1 cos q1 + L2 cos beta,  h = L1 sin q1 + L2 sin beta
        D = max(np.hypot(r, h), 1e-6)
        c1 = (D * D + self.L1 ** 2 - self.L2 ** 2) / (2 * self.L1 * D)
        c2 = (D * D + self.L2 ** 2 - self.L1 ** 2) / (2 * self.L2 * D)
        reachable = abs(c1) <= 1.0 and abs(c2) <= 1.0
        gamma = np.arctan2(h, r)
        q1 = gamma + np.arccos(np.clip(c1, -1.0, 1.0))
        beta = gamma - np.arccos(np.clip(c2, -1.0, 1.0))
        return np.array([q0, q1, beta + np.pi / 2]), reachable

    # --- frames
    @staticmethod
    def oh_to_ros(p):
        return np.array([p[0], -p[2], p[1]]) / 1000.0

    @staticmethod
    def ros_to_oh(p):
        return np.array([p[0], p[2], -p[1]]) * 1000.0


class GravityModel:
    """Physical gravity model identified by gravity_calibration.py.

    Motor command (offset from 2048) that holds the arm:
        g_i = c_i . [cos q1, sin q1, sin q2, cos q2]      (q in rad)
    ('physics_v2'; the older 3-term 'physics_v1' format is also accepted).
    Motor 0 rotates about a vertical axis and needs no compensation.
    """

    def __init__(self, path):
        with open(path) as f:
            m = json.load(f)
        kind = m.get('model')
        if kind == 'physics_v2':
            self.c1 = [float(v) for v in m['motor1']]
            self.c2 = [float(v) for v in m['motor2']]
        elif kind == 'physics_v1':
            a, b = m['motor1'], m['motor2']
            self.c1 = [float(a[0]), float(a[1]), 0.0, 0.0]
            self.c2 = [0.0, 0.0, float(b[0]), float(b[1])]
            if abs(float(a[2])) > 1e-9 or abs(float(b[2])) > 1e-9:
                raise ValueError('physics_v1 model with constant terms: recalibrate with v3')
        else:
            raise ValueError('unsupported gravity model: run gravity_calibration.py')
        self.n_poses = int(m.get('n_poses', 0))

    def __call__(self, q):
        q1, q2 = float(q[1]), float(q[2])
        phi = (np.cos(q1), np.sin(q1), np.sin(q2), np.cos(q2))
        g1 = sum(c * f for c, f in zip(self.c1, phi))
        g2 = sum(c * f for c, f in zip(self.c2, phi))
        return [0.0, float(g1), float(g2)]


class TouchRawDriver(Node):
    """Layers 2 and 4: joint control and ROS 2 interface."""

    def __init__(self):
        super().__init__('touch_raw_driver')

        self.declare_parameter('kp', [0.5, 0.5, 0.5])        # command per count
        self.declare_parameter('ki', [0.5, 1.0, 1.0])        # command per (count * s)
        self.declare_parameter('kd', [0.02, 0.02, 0.02])     # command per (count / s)
        self.declare_parameter('integral_limit', 250.0)      # command units
        # Feedforward (0 = disabled). Values from tools/calibration/friction_identification.py
        self.declare_parameter('kv', [0.0, 0.0, 0.0])        # command per (count / s)
        self.declare_parameter('friction_pos', [0.0, 0.0, 0.0])  # command, u > 0 direction
        self.declare_parameter('friction_neg', [0.0, 0.0, 0.0])  # command, u < 0 direction
        self.declare_parameter('friction_err_scale', 30.0)   # counts
        self.declare_parameter('max_delta', 500)             # command units, hard limit 600
        self.declare_parameter('max_ref_speed', 1000.0)      # counts / s
        self.declare_parameter('vel_filter_alpha', 0.2)
        self.declare_parameter('publish_rate', 100.0)        # Hz
        self.declare_parameter('gravity_comp', True)
        self.declare_parameter('gravity_gain', 0.9)          # fraction of the model applied
        self.declare_parameter('rms_limit', 400.0)           # command units
        self.declare_parameter('rms_time_constant', 10.0)    # s
        self.declare_parameter('gravity_file', '~/.touch_raw_driver/gravity.json')
        self.declare_parameter('kinematics_file', '~/.touch_raw_driver/kinematics.json')

        self._lock = threading.Lock()
        self._p = {}
        self._refresh_params()

        self._enabled = False
        self._enc = None
        self._enc6 = None
        self._vel = [0.0, 0.0, 0.0]
        self._target = None
        self._ref = None
        self._integ = [0.0, 0.0, 0.0]
        self._ref_vel = [0.0, 0.0, 0.0]
        self._deltas = [0, 0, 0]
        self._fault_msg = None
        self._mode = 'position'

        self._kin = None
        kpath = os.path.expanduser(str(self.get_parameter('kinematics_file').value))
        try:
            self._kin = Kinematics(kpath)
            self.get_logger().info(f'Kinematic model loaded from {kpath}.')
        except (OSError, KeyError, ValueError, np.linalg.LinAlgError) as e:
            self.get_logger().warn(f'No kinematic model ({e}): /touch/pose and /touch/target '
                                   'are disabled. Run kin_logger + kin_fit.py first.')

        self._ms = [0.0, 0.0, 0.0]          # running mean square of each motor command
        self._thermal_latched = [False, False, False]
        self._thermal_active = False
        self._gravity = None
        gpath = os.path.expanduser(str(self.get_parameter('gravity_file').value))
        try:
            if self._kin is None:
                raise ValueError('the gravity model needs the kinematic model')
            self._gravity = GravityModel(gpath)
            self.get_logger().info(f'Gravity model loaded from {gpath} '
                                   f'({self._gravity.n_poses} calibration poses).')
        except (OSError, KeyError, ValueError) as e:
            self.get_logger().warn(f'No gravity model ({e}). Running WITHOUT gravity '
                                   'compensation: run gravity_calibration.py first.')

        self._pub_joint = self.create_publisher(Int32MultiArray, '/touch/joint_counts', 10)
        self._pub_enc = self.create_publisher(Int32MultiArray, '/touch/encoders', 10)
        self._pub_cmd = self.create_publisher(Int32MultiArray, '/touch/motor_cmd', 10)
        self.create_subscription(Int32MultiArray, '/touch/joint_target', self._on_target, 10)
        self.create_subscription(Bool, '/touch/enable', self._on_enable, 10)
        self.create_subscription(String, '/touch/mode', self._on_mode, 10)
        self._pub_pose = self.create_publisher(PoseStamped, '/touch/pose', 10)
        self._pub_js = self.create_publisher(JointState, '/touch/joint_states', 10)
        self.create_subscription(Point, '/touch/target', self._on_cartesian_target, 10)

        rate = float(self._p['publish_rate'])
        self.create_timer(1.0 / rate, self._on_publish)
        self.create_timer(0.1, self._refresh_params)

        self._usb = TouchUSB()
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

        self.get_logger().info(
            'Touch found. DISABLED (zero current). Enable with: ros2 topic pub --once -w 1 '
            '/touch/enable std_msgs/msg/Bool "{data: true}"')

    # ------------------------------------------------------------------ parameters
    def _refresh_params(self):
        """Read the ROS parameters into a plain dict used by the control thread."""
        def g(name):
            return self.get_parameter(name).value

        def three(name):
            v = [float(x) for x in g(name)]
            if len(v) != 3:
                self.get_logger().warn(f'Parameter {name} must have 3 values: ignored.',
                                       throttle_duration_sec=5.0)
                return self._p.get(name, [0.0, 0.0, 0.0])
            return [max(0.0, x) for x in v]

        p = {
            'kp': three('kp'),
            'ki': three('ki'),
            'kd': three('kd'),
            'kv': three('kv'),
            'friction_pos': three('friction_pos'),
            'friction_neg': three('friction_neg'),
            'friction_err_scale': max(1.0, float(g('friction_err_scale'))),
            'integral_limit': max(0.0, float(g('integral_limit'))),
            'max_delta': min(abs(int(g('max_delta'))), HARD_MAX_DELTA),
            'max_ref_speed': max(0.0, float(g('max_ref_speed'))),
            'alpha': min(max(float(g('vel_filter_alpha')), 0.0), 1.0),
            'publish_rate': max(1.0, float(g('publish_rate'))),
            'gravity_comp': bool(g('gravity_comp')),
            'gravity_gain': min(max(float(g('gravity_gain')), 0.0), 1.2),
            'rms_limit': max(0.0, float(g('rms_limit'))),
            'rms_tau': max(0.5, float(g('rms_time_constant'))),
        }
        with self._lock:
            self._p = p

    # ------------------------------------------------------------------ ROS callbacks
    def _on_enable(self, msg):
        with self._lock:
            if msg.data and not self._enabled:
                if self._enc is None:
                    self.get_logger().warn('No encoder data yet: not enabling.')
                    return
                # Hold the current position: no jump when the motors are switched on.
                self._ref = [float(e) for e in self._enc]
                self._target = list(self._ref)
                self._integ = [0.0, 0.0, 0.0]
                self._ref_vel = [0.0, 0.0, 0.0]
                self._enabled = True
                self.get_logger().info(f'ENABLED. Holding current position {list(self._enc)}.')
            elif not msg.data and self._enabled:
                self._enabled = False
                self._integ = [0.0, 0.0, 0.0]
                self.get_logger().info('DISABLED. Zero current.')

    def _on_mode(self, msg):
        mode = msg.data.strip().lower()
        if mode not in ('position', 'float'):
            self.get_logger().warn("Unknown mode: use 'position' or 'float'.")
            return
        with self._lock:
            if mode == self._mode:
                return
            self._mode = mode
            # Restart the position controller from the current pose: no jump.
            if self._enc is not None:
                self._ref = [float(e) for e in self._enc]
                self._target = list(self._ref)
            self._integ = [0.0, 0.0, 0.0]
            self._ref_vel = [0.0, 0.0, 0.0]
            if mode == 'float' and self._gravity is None:
                self.get_logger().warn('Float mode without a gravity model: zero current.')
        self.get_logger().info(f'Mode: {mode}')

    def _on_target(self, msg):
        if len(msg.data) != 3:
            self.get_logger().warn('The target must have 3 values: [t0, t1, t2].')
            return
        with self._lock:
            if not self._enabled:
                self.get_logger().warn('Target ignored: enable the node first with /touch/enable.')
                return
            if self._mode != 'position':
                self.get_logger().warn("Target ignored: switch to 'position' mode first.")
                return
            self._target = [float(x) for x in msg.data]
        self.get_logger().info(f'New target: {list(msg.data)}')

    def _on_cartesian_target(self, msg):
        if self._kin is None:
            self.get_logger().warn('Cartesian target ignored: no kinematic model loaded.')
            return
        p_oh = Kinematics.ros_to_oh([msg.x, msg.y, msg.z])
        q, reachable = self._kin.ik(p_oh)
        raw = self._kin.q_to_raw(q)
        # Where the arm will actually go (after the joint-range limits), in the ROS frame.
        p_check = Kinematics.oh_to_ros(self._kin.fk(self._kin.raw_to_q(raw)))
        # Targets may arrive as a continuous stream (keyboard teleoperation, Franka bridge):
        # all messages below are throttled to avoid flooding the terminal.
        if not reachable:
            self.get_logger().warn('Target outside the reachable workspace: moving to the '
                                   'closest reachable configuration.',
                                   throttle_duration_sec=1.0)
        with self._lock:
            if not self._enabled:
                self.get_logger().warn('Target ignored: enable the node first with /touch/enable.',
                                       throttle_duration_sec=2.0)
                return
            if self._mode != 'position':
                self.get_logger().warn("Target ignored: switch to 'position' mode first.",
                                       throttle_duration_sec=2.0)
                return
            self._target = raw
        self.get_logger().info(
            f'New Cartesian target ({msg.x:+.3f}, {msg.y:+.3f}, {msg.z:+.3f}) m -> joints '
            f'{[int(v) for v in raw]} -> expected ({p_check[0]:+.3f}, {p_check[1]:+.3f}, '
            f'{p_check[2]:+.3f}) m', throttle_duration_sec=1.0)

    def _on_publish(self):
        with self._lock:
            enc, enc6, deltas = self._enc, self._enc6, list(self._deltas)
            enabled, target, fault = self._enabled, self._target, self._fault_msg
            mode = self._mode
            self._fault_msg = None
        if fault:
            self.get_logger().error(fault)
        if self._thermal_active:
            self.get_logger().warn('Thermal protection active: motor command reduced.',
                                   throttle_duration_sec=2.0)
        if enc is None:
            self.get_logger().warn('Waiting for encoder data...', throttle_duration_sec=2.0)
            return
        self._pub_joint.publish(Int32MultiArray(data=[int(e) for e in enc]))
        self._pub_enc.publish(Int32MultiArray(data=[int(e) for e in enc6]))
        self._pub_cmd.publish(Int32MultiArray(data=[int(d) for d in deltas]))
        if self._kin is not None:
            q = self._kin.raw_to_q(enc)
            p = Kinematics.oh_to_ros(self._kin.fk(q))
            stamp = self.get_clock().now().to_msg()
            js = JointState()
            js.header.stamp = stamp
            names = ['touch_joint0', 'touch_joint1', 'touch_joint2']
            positions = [float(v) for v in q]
            g = self._kin.raw_to_gimbal(enc6)
            if g is not None:
                names += ['touch_gimbal0', 'touch_gimbal1', 'touch_gimbal2']
                positions += [float(v) for v in g]
            # Assign once: message array fields cannot be extended with a list.
            js.name = names
            js.position = positions
            self._pub_js.publish(js)
            pose = PoseStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = 'touch_base'
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = (float(v) for v in p)
            pose.pose.orientation.w = 1.0
            self._pub_pose.publish(pose)
        state = f'ENABLED/{mode}' if enabled else 'disabled'
        tgt = [int(t) for t in target] if (enabled and target and mode == 'position') else '-'
        self.get_logger().info(f'[{state}] joints {list(enc)} | target {tgt} | command {deltas}',
                               throttle_duration_sec=1.0)

    # ------------------------------------------------------------------ control loop
    def _loop(self):
        """Control thread: USB exchange, velocity estimate and per-joint PID."""
        deltas = [0, 0, 0]
        last_t = time.monotonic()
        while self._running:
            try:
                enc6 = self._usb.exchange(deltas)
            except usb.core.USBError as e:
                with self._lock:
                    self._enabled = False
                    self._fault_msg = f'USB error: {e}. Disabled.'
                deltas = [0, 0, 0]
                time.sleep(0.1)
                continue

            now = time.monotonic()
            dt = max(now - last_t, 1e-4)
            last_t = now

            if enc6 is None:
                # No valid state: never apply current without fresh data.
                deltas = [0, 0, 0]
                continue

            enc = enc6[:3]
            new_deltas = [0, 0, 0]
            with self._lock:
                p = self._p
                # Low-pass filtered velocity estimate (counts / s).
                if self._enc is not None:
                    a = p['alpha']
                    raw_v = [(e - pe) / dt for e, pe in zip(enc, self._enc)]
                    self._vel = [a * r + (1.0 - a) * v for r, v in zip(raw_v, self._vel)]
                self._enc = enc
                self._enc6 = enc6

                m = p['max_delta']
                # Thermal protection with hysteresis: a motor whose RMS command (over
                # rms_time_constant) exceeds rms_limit is capped at 90 % of the limit,
                # and released only when its RMS drops below 85 % of the limit.
                # A constant cap (instead of scaling the command down) avoids the
                # drop-and-recover cycles of the previous version.
                caps = [float(m)] * 3
                if p['rms_limit'] > 0.0:
                    for i in range(3):
                        rms = self._ms[i] ** 0.5
                        if rms > p['rms_limit']:
                            self._thermal_latched[i] = True
                        elif rms < 0.85 * p['rms_limit']:
                            self._thermal_latched[i] = False
                        if self._thermal_latched[i]:
                            caps[i] = min(caps[i], 0.9 * p['rms_limit'])
                else:
                    self._thermal_latched = [False, False, False]

                grav = [0.0, 0.0, 0.0]
                if self._enabled and p['gravity_comp'] and self._gravity is not None:
                    gq = self._gravity(self._kin.raw_to_q(enc))
                    grav = [p['gravity_gain'] * v for v in gq]

                if self._enabled and self._mode == 'float':
                    # Gravity compensation only: the stylus feels weightless.
                    new_deltas = [int(max(-c, min(c, g))) for g, c in zip(grav, caps)]
                elif self._enabled and self._target is not None:
                    max_step = p['max_ref_speed'] * dt
                    lim = p['integral_limit']
                    for i in range(3):
                        # Move the reference towards the target at limited speed.
                        step = self._target[i] - self._ref[i]
                        step = max(-max_step, min(max_step, step))
                        self._ref[i] += step
                        # Filtered velocity of the reference [counts/s].
                        self._ref_vel[i] += 0.2 * (step / dt - self._ref_vel[i])
                        # PID on the joint error, with a clamped integral.
                        err = self._ref[i] - enc[i]
                        prev_integ = self._integ[i]
                        self._integ[i] += p['ki'][i] * err * dt
                        self._integ[i] = max(-lim, min(lim, self._integ[i]))
                        u = p['kp'][i] * err + self._integ[i] - p['kd'][i] * self._vel[i]
                        # Feedforward (u > 0 makes the encoder count increase):
                        #  - velocity: push already in the direction the reference moves;
                        #  - friction: overcome the static friction, with a different value
                        #    for each direction; when the reference moves, its direction is
                        #    used, otherwise a smooth function of the error (helps the last
                        #    millimetres without chattering around the target).
                        u += p['kv'][i] * self._ref_vel[i]
                        if abs(self._ref_vel[i]) > 50.0:
                            direction = 1.0 if self._ref_vel[i] > 0 else -1.0
                        else:
                            direction = math.tanh(err / p['friction_err_scale'])
                        fc = p['friction_pos'][i] if direction > 0 else p['friction_neg'][i]
                        u += fc * direction
                        # Apply the measured motor sign, add the gravity feedforward
                        # (already in motor-command units), then saturate.
                        d = MOTOR_SIGN[i] * u + grav[i]
                        if abs(d) > caps[i]:
                            # Anti-windup: while the output is saturated, freeze the
                            # integral, so it does not keep growing and overshoot later.
                            self._integ[i] = prev_integ
                        new_deltas[i] = int(max(-caps[i], min(caps[i], d)))
                # Update the running mean square with the command actually applied.
                a = min(dt / p['rms_tau'], 1.0)
                for i in range(3):
                    self._ms[i] += a * (new_deltas[i] ** 2 - self._ms[i])
                self._thermal_active = any(self._thermal_latched)
                self._deltas = list(new_deltas)
            deltas = new_deltas
            time.sleep(0.001)

    def shutdown(self):
        """Stop the control thread and leave the device at zero current."""
        self._running = False
        self._thread.join(timeout=1.0)
        self._usb.send_zero()
        self._usb.close()


def main(args=None):
    # Without the default signal handler, Ctrl+C raises KeyboardInterrupt and the
    # node can still send zero current before shutting down.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = TouchRawDriver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as e:
        print(f'ERROR: {e}')
    finally:
        if node is not None:
            node.get_logger().info('Shutting down: sending zero current.')
            node.shutdown()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
