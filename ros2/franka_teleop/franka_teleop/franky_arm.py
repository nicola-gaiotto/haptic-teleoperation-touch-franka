# SPDX-License-Identifier: MIT
# Part of haptic-teleoperation-touch-franka - Copyright (c) 2026 Nicola Gaiotto
"""
Thin wrapper around franky (no ROS here), used by the franka_teleop node.

Commanding principle: every call to send_velocity() starts a short CartesianVelocityMotion
(asynchronous) that replaces the previous one. franky ramps to the commanded velocity, holds
it for `motion_duration_ms` and then decelerates to zero. If the caller stops sending (key
released, node stuck, ...) the robot therefore stops by itself shortly after.
Measured in franky-sim (2026-11-01): 2 cm/s, renewed every 50 ms with 150 ms motions ->
smooth motion; after the last command vx goes to 0 within ~0.2 s.

simulate=True starts franky-sim (MuJoCo) in this process instead of connecting to a robot.
"""
import contextlib

import franky


class FrankyArm:

    def __init__(self, robot_ip, simulate=False, visualize=True, dynamics_factor=0.05,
                 torque_thresholds=None, force_thresholds=None, motion_duration_ms=150):
        self._stack = contextlib.ExitStack()
        self._duration = franky.Duration(int(motion_duration_ms))
        if simulate:
            # Imported here so that the real-robot setup does not need franky-sim.
            from franky_sim import SimulationServer
            from franky_sim.mujoco_simulator import MujocoSimulator
            sim = self._stack.enter_context(MujocoSimulator(enable_visualization=visualize))
            model = sim.add_robot()
            server = self._stack.enter_context(SimulationServer(sim))
            server.run_async()
            self.robot = franky.Robot(model.hostname,
                                      realtime_config=franky.RealtimeConfig.Ignore)
        else:
            # RealtimeConfig.Enforce (default): fails if the PC has no real-time kernel.
            self.robot = franky.Robot(robot_ip)
        self.robot.relative_dynamics_factor = float(dynamics_factor)
        if torque_thresholds is not None and force_thresholds is not None:
            self.robot.set_collision_behavior(list(torque_thresholds), list(force_thresholds))

    # ------------------------------------------------------------------ commands
    def send_velocity(self, linear):
        """Start (or replace) a short Cartesian velocity motion, orientation kept fixed.
        Raises the exception of a previous asynchronous motion if it failed."""
        motion = franky.CartesianVelocityMotion(
            franky.Twist(list(linear), [0.0, 0.0, 0.0]), duration=self._duration)
        self.robot.move(motion, asynchronous=True)

    def stop(self):
        """Command zero velocity (smooth stop). Raises a stored motion exception, if any."""
        self.send_velocity([0.0, 0.0, 0.0])

    def wait(self):
        """Wait for the current motion to end; raises its exception if it failed."""
        self.robot.join_motion()

    def recover(self):
        self.robot.recover_from_errors()

    # ------------------------------------------------------------------ state
    def pose(self):
        """(translation [x, y, z] in metres, quaternion [x, y, z, w]) of the end effector."""
        p = self.robot.current_cartesian_state.pose.end_effector_pose
        return [float(v) for v in p.translation], [float(v) for v in p.quaternion]

    def has_errors(self):
        return bool(self.robot.has_errors)

    def close(self):
        with contextlib.suppress(Exception):
            self.robot.join_motion()
        self._stack.close()
