"""
ros2 run doosan_controller test_moveC [radius_mm] [direction]

현재 TCP 위치를 구의 정점으로 하여 구의 겉면을 따라 90° 호 이동.

  구 중심  = 현재 TCP에서 -Z 방향으로 radius_mm
  Via (45°): YZ 평면 상 45° 지점 — TCP가 구 중심을 향하는 방향 유지
  End (90°): YZ 평면 상 90° 지점 — Y축으로 radius_mm 이동, Z는 -radius_mm

방향 공식: Rx=-90, Ry = 180 - direction * angle_deg, Rz=0
  → TCP 공구 축이 항상 구 중심을 향함

파라미터:
  radius_mm  : 반지름 (mm, 기본값 100)
  direction  :  1 → 구 중심 = 현재 TCP 아래(-Z),  +Y 방향 호, Z 감소 (전진)
               -1 → 구 중심 = 현재 TCP의 -Y,       -Y 방향 호, Z 증가 (복귀)
               * +1 끝 위치에서 -1을 실행하면 원래 위치로 정확히 복귀함

사용 예:
  ros2 run doosan_controller test_moveC              # 100mm, +Y
  ros2 run doosan_controller test_moveC 150          # 150mm, +Y
  ros2 run doosan_controller test_moveC 100 -1       # 100mm, -Y
  ros2 run doosan_controller test_moveC 150 -1       # 150mm, -Y
"""

import math
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from dsr_msgs2.srv import MoveCircle
from robot_arm_interfaces.msg import RobotStatus

ROBOT_NS         = 'dsr01'
DEFAULT_RADIUS    = 100.0   # mm
DEFAULT_DIRECTION = 1       # +1: +Y, -1: -Y
VEL               = 30.0    # mm/s, deg/s
ACC               = 60.0    # mm/s², deg/s²


class TestMoveCNode(Node):

    def __init__(self, radius_mm: float, direction: int):
        super().__init__('test_movec')
        self._radius    = radius_mm
        self._direction = direction   # +1: +Y, -1: -Y
        self._tcp       = None        # [x, y, z, rx, ry, rz]  mm / deg

        self._status_sub = self.create_subscription(
            RobotStatus, '/arm/status', self._status_cb, 10
        )
        self._cli_circle = self.create_client(
            MoveCircle, f'/{ROBOT_NS}/motion/move_circle'
        )

    # ── callbacks ────────────────────────────────────────────────────────────

    def _status_cb(self, msg: RobotStatus):
        if msg.servo_on:
            self._tcp = list(msg.current_tcp)

    # ── helpers ──────────────────────────────────────────────────────────────

    def _wait_tcp(self, timeout: float = 5.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._tcp and any(v != 0.0 for v in self._tcp[:3]):
                return True
        return False

    def _move_circle(self, via: list, end: list) -> bool:
        """DSR move_circle 서비스 호출 (SYNC — 완료까지 블로킹)."""
        if not self._cli_circle.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('move_circle 서비스 없음 — launch 실행 여부 확인')
            return False

        via_msg      = Float64MultiArray()
        via_msg.data = [float(v) for v in via]
        end_msg      = Float64MultiArray()
        end_msg.data = [float(v) for v in end]

        req            = MoveCircle.Request()
        req.pos        = [via_msg, end_msg]
        req.vel        = [VEL, VEL]
        req.acc        = [ACC, ACC]
        req.time       = 0.0
        req.radius     = 0.0
        req.ref        = 0    # DR_BASE
        req.mode       = 0    # ABSOLUTE
        req.angle1     = 0.0
        req.angle2     = 0.0
        req.blend_type = 0
        req.sync_type  = 0    # SYNC

        fut = self._cli_circle.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=60.0)
        resp = fut.result()
        if resp is None:
            self.get_logger().error('move_circle 타임아웃')
            return False
        return bool(resp.success)

    # ── main sequence ────────────────────────────────────────────────────────

    def run(self):
        # 1. 현재 TCP 취득
        self.get_logger().info('현재 TCP 위치 취득 중...')
        if not self._wait_tcp():
            self.get_logger().error('TCP 위치 미수신 — Servo ON 여부 확인')
            return

        r   = self._radius
        d   = self._direction   # +1 or -1
        x0, y0, z0 = self._tcp[0], self._tcp[1], self._tcp[2]
        self.get_logger().info(f'현재 TCP : ({x0:.1f}, {y0:.1f}, {z0:.1f}) mm')

        a45 = math.radians(45)

        if d == 1:
            # ── +1: 구 중심 = 현재 TCP 아래(-Z) ─────────────────────────────
            # P(θ) = (x0, y0 + r·sinθ, (z0-r) + r·cosθ)
            # θ: 0°→90°  →  +Y 방향, Z 감소
            # 방향: Ry = 180 - θ°  (공구가 항상 아래쪽 중심을 향함)
            Cx, Cy_c, Cz_c = x0, y0, z0 - r
            via = [Cx, Cy_c + r * math.sin(a45), Cz_c + r * math.cos(a45),
                   -90.0, 135.0, 0.0]
            end = [Cx, Cy_c + r,                  Cz_c,
                   -90.0,  90.0, 0.0]
            desc = '+Y 방향, Z 감소'
            self.get_logger().info(
                f'구 중심  : ({Cx:.1f}, {Cy_c:.1f}, {Cz_c:.1f}) mm  (현재 TCP 아래)'
            )

        else:
            # ── -1: 구 중심 = 현재 TCP의 -Y 방향 ────────────────────────────
            # 현재 TCP를 90° 지점으로 보고 반대 방향(0°)으로 복귀하는 호.
            # P(θ) = (x0, (y0-r) + r·sinθ, z0 + r·cosθ)
            # θ: 0°→90°  →  -Y 방향, Z 증가  (+1 경로의 역방향)
            # 방향: Ry = 135(via)→180(end)  (공구가 항상 -Y 쪽 중심을 향함)
            Cx, Cy_c, Cz_c = x0, y0 - r, z0
            via = [Cx, Cy_c + r * math.sin(a45), Cz_c + r * math.cos(a45),
                   -90.0, 135.0, 0.0]
            end = [Cx, Cy_c,                      Cz_c + r,
                   -90.0, 180.0, 0.0]
            desc = '-Y 방향, Z 증가 (복귀)'
            self.get_logger().info(
                f'구 중심  : ({Cx:.1f}, {Cy_c:.1f}, {Cz_c:.1f}) mm  (현재 TCP의 -Y)'
            )

        self.get_logger().info(f'경로     : {desc}  r={r:.0f} mm')
        self.get_logger().info(
            f'Via (45°): ({via[0]:.1f}, {via[1]:.1f}, {via[2]:.1f})'
            f'  rx={via[3]:.0f} ry={via[4]:.0f} rz={via[5]:.0f}'
        )
        self.get_logger().info(
            f'End      : ({end[0]:.1f}, {end[1]:.1f}, {end[2]:.1f})'
            f'  rx={end[3]:.0f} ry={end[4]:.0f} rz={end[5]:.0f}'
        )

        # 4. MoveC 실행
        self.get_logger().info('MoveC 실행 중...')
        if self._move_circle(via, end):
            self.get_logger().info('MoveC 완료')
        else:
            self.get_logger().error('MoveC 실패')


def main(args=None):
    radius_mm = DEFAULT_RADIUS
    direction = DEFAULT_DIRECTION

    positional = []
    for arg in sys.argv[1:]:
        if arg == '--ros-args':
            break
        try:
            positional.append(float(arg))
        except ValueError:
            continue

    if len(positional) >= 1:
        radius_mm = positional[0]
    if len(positional) >= 2:
        raw = int(positional[1])
        direction = 1 if raw >= 0 else -1

    dir_str = '+Y' if direction == 1 else '-Y'
    print(f'[test_moveC] 반지름: {radius_mm:.0f} mm  방향: {dir_str}')
    rclpy.init(args=args)
    node = TestMoveCNode(radius_mm, direction)
    try:
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
