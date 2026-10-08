import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


def quat_to_rpy_deg(q):
    # Conversione quaternione -> roll, pitch, yaw (convenzione ZYX), in gradi
    roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y))
    s = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
    pitch = math.asin(s)
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


class PoseMonitor(Node):
    def __init__(self):
        super().__init__('touch_pose_monitor')
        self.last = None
        self.create_subscription(PoseStamped, '/phantom/pose', self.on_msg, 10)
        self.create_timer(0.1, self.show)  # aggiorna la riga 10 volte al secondo

    def on_msg(self, msg):
        self.last = msg.pose

    def show(self):
        if self.last is None:
            print('\rIn attesa di /phantom/pose ...', end='', flush=True)
            return
        p = self.last.position
        r, pi, y = quat_to_rpy_deg(self.last.orientation)
        print(f'\rPOS [mm]  x={p.x * 1000:7.1f}  y={p.y * 1000:7.1f}  z={p.z * 1000:7.1f}   '
              f'ORIENT [deg]  roll={r:7.1f}  pitch={pi:7.1f}  yaw={y:7.1f}   ',
              end='', flush=True)


def main():
    rclpy.init()
    node = PoseMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print()
        node.destroy_node()
        rclpy.try_shutdown()


main()
