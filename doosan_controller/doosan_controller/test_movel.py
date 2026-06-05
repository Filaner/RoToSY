"""
ros2 run doosan_controller test_movel <marker_id>

ArUco 마커 지정 ID 위치로 이동 테스트.

시퀀스:
  1. 홈 포지션 이동 (/arm/home)
  2. rx=90, ry=-90, rz=0 방향 정렬 (MoveL 절대 좌표)
  3. 카메라 API에서 지정 마커 좌표 취득
  4. 마커 위치 + Y +100mm 로 MoveL 이동

사용 예:
  ros2 run doosan_controller test_movel 0
  ros2 run doosan_controller test_movel 4
"""

import json
import sys
import time
import urllib.request

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from robot_arm_interfaces.action import MoveJ, MoveL
from robot_arm_interfaces.msg import RobotStatus
from robot_arm_interfaces.srv import Home

CAMERA_API  = 'http://localhost:8000/camera/markers'
Y_OFFSET_MM =  30.0   # 마커 Y 좌표에서 0cm

VEL_MM  = 30.0
ACC_MM  = 60.0
VEL_DEG = 30.0
ACC_DEG = 60.0


class TestMoveToMarker(Node):
    def __init__(self, marker_id: int):
        super().__init__('test_movel')
        self._marker_id = marker_id

        self._home_cli   = self.create_client(Home, '/arm/home')
        self._movel      = ActionClient(self, MoveL, '/arm/move_l')
        self._movej      = ActionClient(self, MoveJ, '/arm/move_j')
        self._status_sub = self.create_subscription(
            RobotStatus, '/arm/status', self._status_cb, 10
        )
        self._tcp    = None   # [x, y, z, rx, ry, rz] mm / deg
        self._joints = None   # [j1..j6] deg

    # ── callbacks ────────────────────────────────────────────────────────────

    def _status_cb(self, msg: RobotStatus):
        self._tcp    = list(msg.current_tcp)
        self._joints = list(msg.current_joints_deg)

    # ── helpers ──────────────────────────────────────────────────────────────

    def _wait_tcp(self, timeout: float = 3.0):
        """TCP 위치 수신 대기."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._tcp and any(v != 0.0 for v in self._tcp[:3]):
                return True
        return False

    def _home(self) -> bool:
        self.get_logger().info('[1] 홈 이동 중...')
        if not self._home_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Home 서비스 없음')
            return False

        req = Home.Request()
        req.target = 0  # mechanical home [0,0,90,0,90,0]
        fut = self._home_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        resp = fut.result()
        if not resp or not resp.success:
            self.get_logger().error(f'홈 실패: {getattr(resp, "message", "?")}')
            return False
        self.get_logger().info('홈 완료')
        time.sleep(0.5)
        return True

    def _move_l(self, x, y, z, rx, ry, rz) -> bool:
        """절대 좌표 MoveL."""
        self.get_logger().info(
            f'MoveL → ({x:.1f}, {y:.1f}, {z:.1f}) mm  rx={rx} ry={ry} rz={rz}'
        )
        self._movel.wait_for_server()

        goal = MoveL.Goal()
        goal.x                      = float(x)
        goal.y                      = float(y)
        goal.z                      = float(z)
        goal.rx                     = float(rx)
        goal.ry                     = float(ry)
        goal.rz                     = float(rz)
        goal.linear_velocity_mm_s   = VEL_MM
        goal.angular_velocity_deg_s = VEL_DEG
        goal.linear_accel_mm_s2     = ACC_MM
        goal.angular_accel_deg_s2   = ACC_DEG
        goal.blend_radius_mm        = 0.0
        goal.reference_frame        = 0      # BASE 좌표계
        goal.relative               = False

        send_fut = self._movel.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_fut)
        handle = send_fut.result()
        if not handle.accepted:
            self.get_logger().error('목표 거부됨 — Servo ON 여부 확인')
            return False

        res_fut = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        result = res_fut.result().result
        if result.success:
            self.get_logger().info(f'이동 완료 ({result.execution_time_sec:.2f}s)')
            return True
        else:
            self.get_logger().error(f'이동 실패: {result.message}')
            return False

    def _move_j(self, joint_angles: list, blend_radius: float = 0.0) -> bool:
        """절대 관절 각도 MoveJ.

        blend_radius > 0 이면 ASYNC 모드로 전송되어 즉시 반환 후
        바로 MoveL을 전송하면 DSR이 두 동작을 블렌딩합니다.
        blend_radius 단위는 deg (관절 공간).
        """
        self.get_logger().info(
            f'MoveJ → {[round(a, 1) for a in joint_angles]}'
            + (f'  blend={blend_radius}°' if blend_radius > 0 else '')
        )
        self._movej.wait_for_server()

        goal = MoveJ.Goal()
        goal.joint_angles_deg    = [float(a) for a in joint_angles]
        goal.velocity_deg_s      = VEL_DEG
        goal.acceleration_deg_s2 = ACC_DEG
        goal.blend_radius_mm     = float(blend_radius)  # 관절 공간에서는 deg 단위

        send_fut = self._movej.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_fut)
        handle = send_fut.result()
        if not handle.accepted:
            self.get_logger().error('MoveJ 목표 거부됨')
            return False

        res_fut = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        result = res_fut.result().result
        if result.success:
            self.get_logger().info(f'MoveJ 완료 ({result.execution_time_sec:.2f}s)')
            return True
        self.get_logger().error(f'MoveJ 실패: {result.message}')
        return False

    def _get_marker(self, marker_id: int, retries: int = 5):
        """카메라 API에서 마커 좌표 취득. 실패 시 None."""
        for attempt in range(retries):
            try:
                with urllib.request.urlopen(CAMERA_API, timeout=3) as resp:
                    data = json.loads(resp.read())
                for m in data.get('markers', []):
                    if m['id'] == marker_id and m.get('x_mm') is not None:
                        return m['x_mm'], m['y_mm'], m['z_mm']
            except Exception as e:
                self.get_logger().warning(f'마커 취득 시도 {attempt+1}: {e}')
            time.sleep(0.5)
        return None

    # ── main sequence ────────────────────────────────────────────────────────

    def run(self):
        # 1. 홈
        if not self._home():
            return

        # 현재 TCP 취득
        if not self._wait_tcp():
            self.get_logger().error('TCP 위치 미수신')
            return
        x0, y0, z0 = self._tcp[0], self._tcp[1], self._tcp[2]
        self.get_logger().info(f'현재 TCP: ({x0:.1f}, {y0:.1f}, {z0:.1f}) mm')

        # 2. 마커 좌표 취득
        marker_id = self._marker_id
        self.get_logger().info(f'[2] ID {marker_id} 마커 취득 중...')
        pos = self._get_marker(marker_id)
        if pos is None:
            self.get_logger().error(
                f'ID {marker_id} 마커 미감지 — 카메라 시야 확인'
            )
            return
        mx, my, mz = pos
        ty = my + Y_OFFSET_MM
        self.get_logger().info(
            f'ID {marker_id} 마커: ({mx:.1f}, {my:.1f}, {mz:.1f}) mm'
            f' → 목표: ({mx:.1f}, {ty:.1f}, {mz:.1f}) mm  (Y{Y_OFFSET_MM:+.0f}mm)'
        )
        
        #   MoveJ(blend=30°) → 즉시 반환 → MoveL 전송
        #   DSR이 관절 회전과 직선 이동을 동시에 블렌딩
        self.get_logger().info('[3] 목표 위치로 이동...')
        self._move_l(mx, ty, mz, 90.0, -90.0, 0.0)
        self.get_logger().info(f'Y 좌표 차이 : {ty - my:.1f} mm')


def main(args=None):
    # sys.argv에서 ROS2 인자 제외하고 첫 번째 정수값을 marker_id로 사용
    marker_id = None
    for arg in sys.argv[1:]:
        if arg == '--ros-args':
            break
        try:
            marker_id = int(arg)
            break
        except ValueError:
            continue

    if marker_id is None:
        print('=' * 50)
        print('사용법: ros2 run doosan_controller test_movel <marker_id>')
        print('예)    ros2 run doosan_controller test_movel 4')
        print('=' * 50)
        print(f'(현재 sys.argv: {sys.argv})')
        return

    print(f'[test_movel] 목표 마커 ID: {marker_id}')

    rclpy.init(args=args)
    node = TestMoveToMarker(marker_id)
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
