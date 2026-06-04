"""
ros2 run doosan_controller motion_sequence <marker_id> [radius_mm]

아루코 마커 ID를 입력받아 해당 마커로 이동하는 시퀀스.
이후 MoveC 원호 접근/복귀까지 확장 예정.

시퀀스 (현재):
  1. 홈 이동
  2. 카메라 API에서 마커 좌표 취득
  3. 마커로 MoveL (마커 4·5는 J4=90/J5=-90 블렌드 이동)

사용 예:
  ros2 run doosan_controller motion_sequence 1
  ros2 run doosan_controller motion_sequence 4
  ros2 run doosan_controller motion_sequence 5 120
"""

import json
import math
import sys
import time
import urllib.request

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from dsr_msgs2.srv import MoveCircle
from robot_arm_interfaces.action import MoveJ, MoveL
from robot_arm_interfaces.msg import RobotStatus
from robot_arm_interfaces.srv import Home

# ── 공통 파라미터 ────────────────────────────────────────────────────────────

CAMERA_API       = 'http://localhost:8000/camera/markers'
ROBOT_NS         = 'dsr01'

VEL_MM           = 30.0
ACC_MM           = 60.0
VEL_DEG          = 30.0
ACC_DEG          = 60.0

DEFAULT_RADIUS   = 100.0   # MoveC 기본 반지름 (mm)

# 마커 4·5: J1=30, J4=90, J5=-90, blend=70°, rz=-90
SPECIAL_MARKERS  = {4, 5}
SPECIAL_JOINTS   = [30.0, None, None, 90.0, -90.0, None]  # None → 현재 관절값 유지
SPECIAL_BLEND    = 70.0
SPECIAL_RZ       = -90.0

# 일반 마커: rz=0
DEFAULT_RZ       = 0.0


class MotionSequenceNode(Node):
    """test_movel + test_moveC 통합 시퀀스 노드."""

    def __init__(self, marker_id: int, radius_mm: float):
        super().__init__('motion_sequence_node')
        self._marker_id  = marker_id
        self._radius_mm  = radius_mm

        self._home_cli   = self.create_client(Home, '/arm/home')
        self._movel      = ActionClient(self, MoveL, '/arm/move_l')
        self._movej      = ActionClient(self, MoveJ, '/arm/move_j')
        self._cli_circle = self.create_client(
            MoveCircle, f'/{ROBOT_NS}/motion/move_circle'
        )
        self._status_sub = self.create_subscription(
            RobotStatus, '/arm/status', self._status_cb, 10
        )
        self._tcp    = None   # [x, y, z, rx, ry, rz] mm / deg
        self._joints = None   # [j1..j6] deg

    # ── 콜백 ─────────────────────────────────────────────────────────────────

    def _status_cb(self, msg: RobotStatus):
        self._tcp    = list(msg.current_tcp)
        self._joints = list(msg.current_joints_deg)

    # ── 유틸 ─────────────────────────────────────────────────────────────────

    def _wait_tcp(self, timeout: float = 5.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._tcp and any(v != 0.0 for v in self._tcp[:3]):
                return True
        return False

    # ── 동작 헬퍼 ────────────────────────────────────────────────────────────

    def _home(self) -> bool:
        self.get_logger().info('[홈] 이동 중...')
        if not self._home_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Home 서비스 없음')
            return False
        req = Home.Request()
        req.target = 0
        fut = self._home_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        resp = fut.result()
        if not resp or not resp.success:
            self.get_logger().error(f'홈 실패: {getattr(resp, "message", "?")}')
            return False
        self.get_logger().info('홈 완료')
        time.sleep(0.5)
        return True

    def _move_l(self, x, y, z, rx=0.0, ry=0.0, rz=0.0, relative: bool = False) -> bool:
        mode = '상대' if relative else '절대'
        self.get_logger().info(
            f'MoveL({mode}) → ({x:.1f}, {y:.1f}, {z:.1f})  rx={rx} ry={ry} rz={rz}'
        )
        self._movel.wait_for_server()
        goal = MoveL.Goal()
        goal.x, goal.y, goal.z     = float(x), float(y), float(z)
        goal.rx, goal.ry, goal.rz  = float(rx), float(ry), float(rz)
        goal.linear_velocity_mm_s   = VEL_MM
        goal.angular_velocity_deg_s = VEL_DEG
        goal.linear_accel_mm_s2     = ACC_MM
        goal.angular_accel_deg_s2   = ACC_DEG
        goal.blend_radius_mm        = 0.0
        goal.reference_frame        = 0
        goal.relative               = relative

        fut = self._movel.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        handle = fut.result()
        if not handle.accepted:
            self.get_logger().error('MoveL 거부 — Servo ON 확인')
            return False
        res_fut = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        result = res_fut.result().result
        if result.success:
            self.get_logger().info(f'MoveL 완료 ({result.execution_time_sec:.2f}s)')
            return True
        self.get_logger().error(f'MoveL 실패: {result.message}')
        return False

    def _move_j(self, joint_angles: list, blend_radius: float = 0.0) -> bool:
        self.get_logger().info(
            f'MoveJ → {[round(a, 1) for a in joint_angles]}'
            + (f'  blend={blend_radius}°' if blend_radius > 0 else '')
        )
        self._movej.wait_for_server()
        goal = MoveJ.Goal()
        goal.joint_angles_deg    = [float(a) for a in joint_angles]
        goal.velocity_deg_s      = VEL_DEG
        goal.acceleration_deg_s2 = ACC_DEG
        goal.blend_radius_mm     = float(blend_radius)

        fut = self._movej.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        handle = fut.result()
        if not handle.accepted:
            self.get_logger().error('MoveJ 거부')
            return False
        res_fut = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        result = res_fut.result().result
        if result.success:
            self.get_logger().info(f'MoveJ 완료 ({result.execution_time_sec:.2f}s)')
            return True
        self.get_logger().error(f'MoveJ 실패: {result.message}')
        return False

    def _move_circle(self, via: list, end: list) -> bool:
        if not self._cli_circle.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('move_circle 서비스 없음')
            return False
        via_msg      = Float64MultiArray(data=[float(v) for v in via])
        end_msg      = Float64MultiArray(data=[float(v) for v in end])
        req          = MoveCircle.Request()
        req.pos      = [via_msg, end_msg]
        req.vel      = [VEL_MM, VEL_DEG]
        req.acc      = [ACC_MM, ACC_DEG]
        req.time     = 0.0
        req.radius   = 0.0
        req.ref      = 0
        req.mode     = 0
        req.angle1   = 0.0
        req.angle2   = 0.0
        req.blend_type = 0
        req.sync_type  = 0

        fut = self._cli_circle.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=60.0)
        resp = fut.result()
        if resp is None:
            self.get_logger().error('MoveC 타임아웃')
            return False
        return bool(resp.success)

    def _move_c(self, radius: float, direction: int,
                start: tuple = None) -> tuple | None:
        """구면 호(90°) 이동.

        start  : (x, y, z) 시작 위치. None이면 self._tcp에서 읽음.
        반환값 : 호의 끝점 (x, y, z). 실패 시 None.

        direction = +1 : 구 중심이 시작점 아래(-Z), +Y 이동하며 Z 감소
        direction = -1 : 구 중심이 시작점의 -Y,   -Y 이동하며 Z 증가
        """
        if start is not None:
            x0, y0, z0 = start
        else:
            # self._tcp가 최신인지 확인하기 위해 여러 번 spin
            deadline = time.time() + 1.0
            while time.time() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
                if self._tcp and any(v != 0.0 for v in self._tcp[:3]):
                    break
            if not self._tcp:
                self.get_logger().error('MoveC: TCP 위치 미수신')
                return None
            x0, y0, z0 = self._tcp[0], self._tcp[1], self._tcp[2]

        r   = float(radius)
        a45 = math.radians(45)

        if direction >= 0:
            Cx, Cy_c, Cz_c = x0, y0, z0 - r
            via = [Cx, Cy_c + r * math.sin(a45), Cz_c + r * math.cos(a45), -90.0, 135.0, 0.0]
            end = [Cx, Cy_c + r,                  Cz_c,                      -90.0,  90.0, 0.0]
            desc = f'+Y Z감소  r={r:.0f}mm'
        else:
            Cx, Cy_c, Cz_c = x0, y0 - r, z0
            via = [Cx, Cy_c + r * math.sin(a45), Cz_c + r * math.cos(a45), -90.0, 135.0, 0.0]
            end = [Cx, Cy_c,                      Cz_c + r,                  -90.0, 180.0, 0.0]
            desc = f'-Y Z증가  r={r:.0f}mm (복귀)'

        self.get_logger().info(f'MoveC {desc}  시작({x0:.1f},{y0:.1f},{z0:.1f})')
        self.get_logger().info(
            f'  Via: ({via[0]:.1f},{via[1]:.1f},{via[2]:.1f})'
            f'  End: ({end[0]:.1f},{end[1]:.1f},{end[2]:.1f})'
        )

        if not self._move_circle(via, end):
            self.get_logger().error('MoveC 실패')
            return None

        self.get_logger().info('MoveC 완료')
        return (end[0], end[1], end[2])

    def _get_marker(self, marker_id: int, retries: int = 5):
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

    # ── 시퀀스 ───────────────────────────────────────────────────────────────

    def _goto_marker(self, marker_id: int) -> tuple | None:
        """지정 마커로 이동. 성공 시 도착 TCP (x, y, z) 반환, 실패 시 None."""
        self.get_logger().info(f'[마커{marker_id}] 좌표 취득 중...')
        pos = self._get_marker(marker_id)
        if pos is None:
            self.get_logger().error(f'ID {marker_id} 마커 미감지')
            return None
        mx, my, mz = pos
        self.get_logger().info(f'마커 위치: ({mx:.1f}, {my:.1f}, {mz:.1f}) mm')

        if marker_id in SPECIAL_MARKERS:
            # J4=90, J5=-90 블렌드 이동
            rclpy.spin_once(self, timeout_sec=0.3)
            joints = list(self._joints) if self._joints else [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
            for idx, val in enumerate(SPECIAL_JOINTS):
                if val is not None:
                    joints[idx] = val
            if not self._move_j(joints, blend_radius=SPECIAL_BLEND):
                return None
            if not self._move_l(mx, my, mz, 90.0, -90.0, SPECIAL_RZ):
                return None
        else:
            if not self._move_l(mx, my, mz, 90.0, -90.0, DEFAULT_RZ):
                return None

        return mx, my, mz

    def run(self):
        # 1. 홈
        if not self._home():
            return

        if not self._wait_tcp():
            self.get_logger().error('TCP 위치 미수신')
            return

        # 2. 마커로 이동
        result = self._goto_marker(self._marker_id)
        if result is None:
            return
        mx, my, mz = result
        self.get_logger().info(
            f'마커 {self._marker_id} 도착: ({mx:.1f}, {my:.1f}, {mz:.1f}) mm'
        )

        # 위치 추적 (상대이동·MoveC 모두 직접 계산 → self._tcp 타이밍 문제 회피)
        cx, cy, cz = mx, my, mz

        # 3. Y +15cm 상대 이동
        self.get_logger().info('[3] Y +150mm 상대 이동...')
        if not self._move_l(0, 150, 0, relative=True):
            return
        cy += 150

        # 4. Y +3cm 상대 이동
        self.get_logger().info('[4] Y +30mm 상대 이동...')
        if not self._move_l(0, 30, 0, relative=True):
            return
        cy += 30

        # 5. MoveC r=200 direction=-1 (복귀 호)
        self.get_logger().info('[5] MoveC r=200 direction=-1...')
        end_pos = self._move_c(200, -1, start=(cx, cy, cz))
        if end_pos is None:
            return
        cx, cy, cz = end_pos

        # 6. MoveC r=200 direction=+1 (전진 호)
        self.get_logger().info('[6] MoveC r=200 direction=+1...')
        end_pos = self._move_c(200, 1, start=(cx, cy, cz))
        if end_pos is None:
            return
        cx, cy, cz = end_pos

        # 7. Y -3cm 상대 이동
        self.get_logger().info('[7] Y -30mm 상대 이동...')
        if not self._move_l(0, -30, 0, relative=True):
            return
        cy -= 30

        # 8. Y -20cm 상대 이동
        self.get_logger().info('[8] Y -150mm 상대 이동...')
        if not self._move_l(0, -150, 0, relative=True):
            return
        cy -= 150

        self.get_logger().info('시퀀스 완료')


def main(args=None):
    marker_id  = None
    radius_mm  = DEFAULT_RADIUS

    positional = []
    for arg in sys.argv[1:]:
        if arg == '--ros-args':
            break
        try:
            positional.append(arg)
        except ValueError:
            continue

    for tok in positional:
        try:
            v = float(tok)
            if marker_id is None:
                marker_id = int(v)
            else:
                radius_mm = v
        except ValueError:
            continue

    if marker_id is None:
        print('=' * 55)
        print('사용법: ros2 run doosan_controller motion_sequence <marker_id> [radius_mm]')
        print('예)    ros2 run doosan_controller motion_sequence 4')
        print('예)    ros2 run doosan_controller motion_sequence 5 120')
        print('=' * 55)
        return

    print(f'[motion_sequence] 마커 ID: {marker_id}  MoveC 반지름: {radius_mm:.0f} mm')

    rclpy.init(args=args)
    node = MotionSequenceNode(marker_id, radius_mm)
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
