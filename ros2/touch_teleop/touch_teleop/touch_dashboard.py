#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Part of haptic-teleoperation-touch-franka - Copyright (c) 2026 Nicola Gaiotto
"""
touch_dashboard - simple control panel for the 3D Systems Touch (the 3D view is in RViz).

  - position read-out in millimetres;
  - buttons: motors ON / OFF, mode position / float, STOP;
  - speed slider;
  - keyboard teleoperation with the ARROW keys while the window has the focus:
        Left / Right   -> left / right
        Up / Down      -> forward / backward (away from / towards you)
        W / S          -> up / down
        Space          -> stop
    The motion lasts exactly while the key is held (key release = stop).

It publishes the same velocity command as teleop_twist_keyboard (geometry_msgs/Twist on
/touch/cmd_vel), so twist_to_target must be running. Both can be used together.

Topics:
  subscribes  /touch/pose     geometry_msgs/PoseStamped
  publishes   /touch/cmd_vel  geometry_msgs/Twist
  publishes   /touch/enable   std_msgs/Bool
  publishes   /touch/mode     std_msgs/String
"""
import threading
import tkinter as tk

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String

# Key -> velocity direction in the Twist convention (x forward, y left, z up).
KEY_DIRECTIONS = {
    'Up': (1, 0, 0), 'Down': (-1, 0, 0),
    'Left': (0, 1, 0), 'Right': (0, -1, 0),
    'w': (0, 0, 1), 's': (0, 0, -1), 'W': (0, 0, 1), 'S': (0, 0, -1),
}


class DashboardNode(Node):

    def __init__(self):
        super().__init__('touch_dashboard')
        self.declare_parameter('linear_scale', 0.05)   # must match twist_to_target
        self.lock = threading.Lock()
        self.pose = None
        self.create_subscription(PoseStamped, '/touch/pose', self._on_pose, 10)
        self.pub_cmd = self.create_publisher(Twist, '/touch/cmd_vel', 10)
        self.pub_enable = self.create_publisher(Bool, '/touch/enable', 10)
        self.pub_mode = self.create_publisher(String, '/touch/mode', 10)

    def _on_pose(self, msg):
        p = msg.pose.position
        with self.lock:
            self.pose = (p.x, p.y, p.z)

    def send_velocity(self, direction, speed_m_s):
        """direction in the Twist convention; speed in m/s of the stylus."""
        scale = float(self.get_parameter('linear_scale').value)
        k = speed_m_s / scale if scale > 0 else 0.0
        t = Twist()
        t.linear.x, t.linear.y, t.linear.z = (float(d * k) for d in direction)
        self.pub_cmd.publish(t)


class DashboardApp:

    def __init__(self, node):
        self.node = node
        self.pressed = set()
        self.pending_release = {}
        self.was_moving = False

        self.root = tk.Tk()
        self.root.title('Touch control panel')
        self.root.protocol('WM_DELETE_WINDOW', self.close)

        panel = tk.Frame(self.root, padx=14, pady=12)
        panel.pack()
        tk.Label(panel, text='Stylus position', font=('Sans', 11, 'bold')).pack()
        self.lbl_pos = tk.Label(panel, text='x = ---\ny = ---\nz = ---',
                                font=('Monospace', 16), justify='left')
        self.lbl_pos.pack(pady=(2, 12))

        row = tk.Frame(panel)
        row.pack(pady=2)
        tk.Button(row, text='Motors ON', bg='#7bc67b', width=12,
                  command=lambda: self._enable(True)).pack(side='left', padx=2)
        tk.Button(row, text='Motors OFF', bg='#e57373', width=12,
                  command=lambda: self._enable(False)).pack(side='left', padx=2)
        row = tk.Frame(panel)
        row.pack(pady=2)
        tk.Button(row, text='Mode: position', width=12,
                  command=lambda: self._mode('position')).pack(side='left', padx=2)
        tk.Button(row, text='Mode: float', width=12,
                  command=lambda: self._mode('float')).pack(side='left', padx=2)
        tk.Button(panel, text='STOP  (Space)', width=26,
                  command=self._stop).pack(pady=(8, 2))

        tk.Label(panel, text='Speed [cm/s]').pack(pady=(10, 0))
        # takefocus=0 and focus back to the window: otherwise the arrow keys would move the
        # slider instead of the stylus.
        self.speed = tk.Scale(panel, from_=0.5, to=5.0, resolution=0.5, orient='horizontal',
                              length=220, takefocus=0, command=lambda v: self.root.focus_set())
        self.speed.set(2.5)
        self.speed.pack()

        tk.Label(panel, justify='left', font=('Sans', 10),
                 text='Click on this window, then:\n'
                      '   \u2190 \u2192    left / right\n'
                      '   \u2191 \u2193    forward / backward\n'
                      '   W  S    up / down\n'
                      '   Space   stop').pack(pady=(10, 0))
        self.lbl_state = tk.Label(panel, text='motors OFF', fg='gray')
        self.lbl_state.pack(pady=(8, 0))

        # ---- keyboard
        self.root.bind('<KeyPress>', self._on_key_press)
        self.root.bind('<KeyRelease>', self._on_key_release)
        self.root.bind('<space>', lambda e: self._stop())
        self.root.bind('<FocusOut>', lambda e: self._stop())

        self.root.focus_set()
        self.root.after(33, self._command_loop)
        self.root.after(50, self._redraw)

    # ---------------------------------------------------------------- buttons
    def _enable(self, on):
        self._stop()
        self.node.pub_enable.publish(Bool(data=on))
        self.lbl_state.config(text='motors ON' if on else 'motors OFF (hold the stylus!)')

    def _mode(self, mode):
        self._stop()
        self.node.pub_mode.publish(String(data=mode))
        self.lbl_state.config(text=f'mode: {mode}')

    def _stop(self):
        self.pressed.clear()
        self.node.send_velocity((0, 0, 0), 0.0)

    # ---------------------------------------------------------------- keys
    # With key auto-repeat, X11 sends release+press pairs while a key is held: a release
    # is confirmed only if no new press arrives within 40 ms.
    def _on_key_press(self, event):
        key = event.keysym
        if key in self.pending_release:
            self.root.after_cancel(self.pending_release.pop(key))
        if key in KEY_DIRECTIONS:
            self.pressed.add(key)

    def _on_key_release(self, event):
        key = event.keysym
        if key in KEY_DIRECTIONS:
            self.pending_release[key] = self.root.after(40, self._confirm_release, key)

    def _confirm_release(self, key):
        self.pending_release.pop(key, None)
        self.pressed.discard(key)

    def _command_loop(self):
        if self.pressed:
            d = [0, 0, 0]
            for k in self.pressed:
                for i in range(3):
                    d[i] += KEY_DIRECTIONS[k][i]
            d = [max(-1, min(1, v)) for v in d]
            self.node.send_velocity(d, self.speed.get() / 100.0)
            self.was_moving = True
        elif self.was_moving:
            self.node.send_velocity((0, 0, 0), 0.0)   # immediate stop on release
            self.was_moving = False
        self.root.after(33, self._command_loop)

    # ---------------------------------------------------------------- drawing
    def _redraw(self):
        with self.node.lock:
            pose = self.node.pose
        if pose is not None:
            self.lbl_pos.config(text=f'x = {pose[0] * 1000:+7.1f} mm  (right)\n'
                                     f'y = {pose[1] * 1000:+7.1f} mm  (forward)\n'
                                     f'z = {pose[2] * 1000:+7.1f} mm  (up)')
        self.root.after(100, self._redraw)

    def close(self):
        self._stop()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    try:
        DashboardApp(node).run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
