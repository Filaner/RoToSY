#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from std_msgs.msg import Float64MultiArray
import math

class TcpMonitor(Node):
    def __init__(self):
        super().__init__('tcp_monitor')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._pub = self.create_publisher(Float64MultiArray, '/arm/tcp_pose', 10)
        # 10Hz로 주기적으로 조회
        self.timer = self.create_timer(0.1, self.get_tcp)

    def get_tcp(self):
        try:
            t = self.tf_buffer.lookup_transform('base_link', 'link_6', rclpy.time.Time())
            tr = t.transform.translation
            q = t.transform.rotation

            x = tr.x * 1000  # m → mm
            y = tr.y * 1000
            z = tr.z * 1000

            # 쿼터니언 → RPY (degree)
            rx, ry, rz = self.quat_to_rpy_deg(q.x, q.y, q.z, q.w)

            msg = Float64MultiArray()
            msg.data = [x, y, z, rx, ry, rz]
            self._pub.publish(msg)
        except Exception:
            pass  # 아직 TF 없을 때 무시

    def quat_to_rpy_deg(self, qx, qy, qz, qw):
        # Roll
        sinr = 2*(qw*qx + qy*qz)
        cosr = 1 - 2*(qx*qx + qy*qy)
        rx = math.atan2(sinr, cosr)
        # Pitch
        sinp = 2*(qw*qy - qz*qx)
        sinp = max(-1.0, min(1.0, sinp))
        ry = math.asin(sinp)
        # Yaw
        siny = 2*(qw*qz + qx*qy)
        cosy = 1 - 2*(qy*qy + qz*qz)
        rz = math.atan2(siny, cosy)

        return math.degrees(rx), math.degrees(ry), math.degrees(rz)

def main():
    rclpy.init()
    node = TcpMonitor()
    rclpy.spin(node)

if __name__ == '__main__':
    main()