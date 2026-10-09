#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Part of haptic-teleoperation-touch-franka - Copyright (c) 2026 Nicola Gaiotto
"""
touch_dashboard - simple 2D control panel for the 3D Systems Touch.

A single window, easier to read than RViz:
  - TOP VIEW (seen from above) and FRONT VIEW (seen by the operator) of the workspace,
    with the stylus position (blue dot), its recent trail, the last target (orange
    cross) and the workspace box;
  - position read-out in millimetres;
  - buttons: motors ON / OFF, mode position / float, STOP;
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
  subscribes  /touch/target   geometry_msgs/Point
  publishes   /touch/cmd_vel  geometry_msgs/Twist
  publishes   /touch/enable   std_msgs/Bool
  publishes   /touch/mode     std_msgs/String
"""
import collections
import threading
import time
import tkinter as tk

import matplotlib
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg  # noqa: E402
from matplotlib.figure import Figure  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402

import rclpy  # noqa: E402
from geometry_msgs.msg import Point, PoseStamped, Twist  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import Bool, String  # noqa: E402

# Key -> velocity direction in the Twist convention (x forward, y left, z up).
KEY_DIRECTIONS = {
    'Up': (1, 0, 0), 'Down': (-1, 0, 0),
    'Left': (0, 1, 0), 'Right': (0, -1, 0),
    'w': (0, 0, 1), 's': (0, 0, -1), 'W': (0, 0, 1), 'S': (0, 0, -1),
}
TRAIL_SECONDS = 4.0


class DashboardNode(Node):

    def __init__(self):
        super().__init__('touch_dashboard')
        self.declare_parameter('linear_scale', 0.05)   # must match twist_to_target
        self.declare_parameter('workspace_min', [-0.08, -0.06, -0.05])
        self.declare_parameter('workspace_max', [0.08, 0.06, 0.08])
        self.lock = threading.Lock()
        self.pose = None
        self.target = None
        self.trail = collections.deque()
        self.create_subscription(PoseStamped, '/touch/pose', self._on_pose, 10)
        self.create_subscription(Point, '/touch/target', self._on_target, 10)
        self.pub_cmd = self.create_publisher(Twist, '/touch/cmd_vel', 10)
        self.pub_enable = self.create_publisher(Bool, '/touch/enable', 10)
        self.pub_mode = self.create_publisher(String, '/touch/mode', 10)

    def _on_pose(self, msg):
        p = msg.pose.position
        now = time.monotonic()
        with self.lock:
            self.pose = (p.x, p.y, p.z)
            self.trail.append((now, p.x, p.y, p.z))
            while self.trail and now - self.trail[0][0] > TRAIL_SECONDS:
                self.trail.popleft()

    def _on_target(self, msg):
        with self.lock:
            self.target = (msg.x, msg.y, msg.z)

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
        self.root.title('Touch dashboard')
        self.root.protocol('WM_DELETE_WINDOW', self.close)

        # ---- plots
        lo = [float(v) for v in node.get_parameter('workspace_min').value]
        hi = [float(v) for v in node.get_parameter('workspace_max').value]
        self.fig = Figure(figsize=(9, 4.4), dpi=100)
        self.ax_top = self.fig.add_subplot(1, 2, 1)
        self.ax_front = self.fig.add_subplot(1, 2, 2)
        views = ((self.ax_top, 'TOP VIEW (from above)', 'y forward [mm]', 1),
                 (self.ax_front, 'FRONT VIEW (operator)', 'z up [mm]', 2))
        self.artists = {}
        for ax, title, ylabel, j in views:
            ax.set_title(title)
            ax.set_xlabel('x right [mm]')
            ax.set_ylabel(ylabel)
            ax.set_aspect('equal')
            m = 20.0
            ax.set_xlim(lo[0] * 1000 - m, hi[0] * 1000 + m)
            ax.set_ylim(lo[j] * 1000 - m, hi[j] * 1000 + m)
            ax.grid(True, alpha=0.3)
            ax.add_patch(Rectangle((lo[0] * 1000, lo[j] * 1000), (hi[0] - lo[0]) * 1000,
                                   (hi[j] - lo[j]) * 1000, fill=False, ls='--',
                                   ec='tab:blue', alpha=0.6, label='workspace'))
            trail, = ax.plot([], [], '-', color='tab:blue', alpha=0.3, lw=1.5)
            target, = ax.plot([], [], 'x', color='tab:orange', ms=12, mew=2.5, label='target')
            stylus, = ax.plot([], [], 'o', color='tab:blue', ms=11, label='stylus')
            self.artists[j] = (trail, target, stylus)
        self.ax_top.legend(loc='upper right', fontsize=8)
        self.fig.tight_layout()
        canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        canvas.get_tk_widget().grid(row=0, column=0, rowspan=8, padx=4, pady=4)
        self.canvas = canvas

        # ---- side panel
        panel = tk.Frame(self.root)
        panel.grid(row=0, column=1, sticky='n', padx=10, pady=10)
        self.lbl_pos = tk.Label(panel, text='x = ---\ny = ---\nz = ---',
                                font=('Monospace', 14), justify='left')
        self.lbl_pos.pack(pady=(0, 12))
        tk.Button(panel, text='Motors ON', bg='#7bc67b', width=16,
                  command=lambda: self._enable(True)).pack(pady=2)
        tk.Button(panel, text='Motors OFF', bg='#e57373', width=16,
                  command=lambda: self._enable(False)).pack(pady=2)
        tk.Button(panel, text='Mode: position', width=16,
                  command=lambda: self._mode('position')).pack(pady=(12, 2))
        tk.Button(panel, text='Mode: float', width=16,
                  command=lambda: self._mode('float')).pack(pady=2)
        tk.Button(panel, text='STOP (Space)', width=16,
                  command=self._stop).pack(pady=(12, 2))
        tk.Label(panel, text='Speed [cm/s]').pack(pady=(12, 0))
        # takefocus=0 and focus back to the window: otherwise the arrow keys would move the
        # slider instead of the stylus.
        self.speed = tk.Scale(panel, from_=0.5, to=5.0, resolution=0.5, orient='horizontal',
                              takefocus=0, command=lambda v: self.root.focus_set())
        self.speed.set(2.5)
        self.speed.pack()
        tk.Label(panel, justify='left', font=('Sans', 9),
                 text='Keys (click on this window first):\n'
                      '  arrows  left / right / fwd / back\n'
                      '  W / S   up / down\n'
                      '  Space   stop').pack(pady=(12, 0))
        self.lbl_state = tk.Label(panel, text='', fg='gray')
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
            pose, target, trail = self.node.pose, self.node.target, list(self.node.trail)
        if pose is not None:
            self.lbl_pos.config(text=f'x = {pose[0] * 1000:+7.1f} mm\n'
                                     f'y = {pose[1] * 1000:+7.1f} mm\n'
                                     f'z = {pose[2] * 1000:+7.1f} mm')
            for j in (1, 2):
                tr, tg, st = self.artists[j]
                tr.set_data([p[1] * 1000 for p in trail], [p[1 + j] * 1000 for p in trail])
                st.set_data([pose[0] * 1000], [pose[j] * 1000])
                if target is not None:
                    tg.set_data([target[0] * 1000], [target[j] * 1000])
            self.canvas.draw_idle()
        self.root.after(50, self._redraw)

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
