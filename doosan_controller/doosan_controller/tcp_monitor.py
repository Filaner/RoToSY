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

        self.declare_parameter('max_position_jump_mm', 250.0)
        self.declare_parameter('max_orientation_jump_deg', 120.0)
        self.declare_parameter('position_filter_alpha', 0.35)
        self.declare_parameter('orientation_filter_alpha', 0.25)
        self.declare_parameter('singular_pitch_threshold_deg', 175.0)

        self._max_position_jump_mm = float(
            self.get_parameter('max_position_jump_mm').value
        )
        self._max_orientation_jump_deg = float(
            self.get_parameter('max_orientation_jump_deg').value
        )
        self._position_alpha = self._clamp01(
            float(self.get_parameter('position_filter_alpha').value)
        )
        self._orientation_alpha = self._clamp01(
            float(self.get_parameter('orientation_filter_alpha').value)
        )
        self._singular_pitch_threshold_deg = float(
            self.get_parameter('singular_pitch_threshold_deg').value
        )

        self._last_quat = None
        self._filtered_pose = None
        self._rejected_count = 0

        # 10Hz로 주기적으로 조회
        self.timer = self.create_timer(0.1, self.get_tcp)
        self.get_logger().info(
            'tcp_monitor filter enabled: '
            f'pos_jump>{self._max_position_jump_mm:.1f}mm, '
            f'ori_jump>{self._max_orientation_jump_deg:.1f}deg, '
            f'alpha=({self._position_alpha:.2f}, {self._orientation_alpha:.2f}), '
            f'singular_pitch>{self._singular_pitch_threshold_deg:.1f}deg'
        )

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
            quat = self._normalize_quat((q.x, q.y, q.z, q.w))
            raw_pose = [x, y, z, rx, ry, rz]

            if self._is_outlier(raw_pose, quat):
                self._rejected_count += 1
                if self._rejected_count == 1 or self._rejected_count % 20 == 0:
                    self.get_logger().warning(
                        f'TCP outlier ignored ({self._rejected_count} rejected)'
                    )
                return

            filtered_pose = self._filter_pose(raw_pose)
            self._last_quat = quat
            self._filtered_pose = filtered_pose
            self._rejected_count = 0

            msg = Float64MultiArray()
            msg.data = filtered_pose
            self._pub.publish(msg)
        except Exception:
            pass  # 아직 TF 없을 때 무시

    def _is_outlier(self, pose, quat):
        if self._filtered_pose is None or self._last_quat is None:
            return False

        position_jump = math.dist(pose[:3], self._filtered_pose[:3])
        orientation_jump = self._quat_angle_deg(quat, self._last_quat)
        return (
            position_jump > self._max_position_jump_mm
            or orientation_jump > self._max_orientation_jump_deg
        )

    def _filter_pose(self, pose):
        if self._filtered_pose is None:
            return list(pose)

        filtered = [0.0] * 6
        for index in range(3):
            filtered[index] = self._lerp(
                self._filtered_pose[index],
                pose[index],
                self._position_alpha,
            )

        continuous_rpy = self._continuous_rpy(pose[3:6], self._filtered_pose[3:6])
        for offset, index in enumerate(range(3, 6)):
            filtered[index] = self._wrap_deg(
                self._lerp(
                    self._filtered_pose[index],
                    continuous_rpy[offset],
                    self._orientation_alpha,
                )
            )

        return filtered

    def _continuous_rpy(self, rpy, reference):
        candidates = [
            list(rpy),
            [rpy[0] + 180.0, 180.0 - rpy[1], rpy[2] + 180.0],
            [rpy[0] - 180.0, 180.0 - rpy[1], rpy[2] - 180.0],
        ]

        best = None
        best_score = float('inf')
        for candidate in candidates:
            unwrapped = [
                self._unwrap_deg(angle, ref)
                for angle, ref in zip(candidate, reference)
            ]
            score = sum((angle - ref) ** 2 for angle, ref in zip(unwrapped, reference))
            if score < best_score:
                best = unwrapped
                best_score = score

        if abs(best[1]) >= self._singular_pitch_threshold_deg:
            # At ry ~= +/-180 deg, rx/rz are not independently observable in a
            # stable way. Keep their previous display values and let quaternion
            # filtering handle outlier rejection.
            best[0] = reference[0]
            best[2] = reference[2]

        return best

    @staticmethod
    def _clamp01(value):
        return max(0.0, min(1.0, value))

    @staticmethod
    def _lerp(previous, current, alpha):
        return previous + alpha * (current - previous)

    @staticmethod
    def _unwrap_deg(angle, reference):
        return reference + ((angle - reference + 180.0) % 360.0 - 180.0)

    @staticmethod
    def _wrap_deg(angle):
        return ((angle + 180.0) % 360.0) - 180.0

    @staticmethod
    def _normalize_quat(quat):
        norm = math.sqrt(sum(v * v for v in quat))
        if norm == 0.0:
            return 0.0, 0.0, 0.0, 1.0
        return tuple(v / norm for v in quat)

    @staticmethod
    def _quat_angle_deg(a, b):
        dot = abs(sum(x * y for x, y in zip(a, b)))
        dot = max(-1.0, min(1.0, dot))
        return math.degrees(2.0 * math.acos(dot))

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
