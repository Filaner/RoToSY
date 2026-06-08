"""
Temp Sequence Node with Interactive (Step-by-Step) Control.

motion_sequence와 동일한 시퀀스이며, 토픽 네임스페이스만 /temp_motion/* 으로 분리됨.

Usage:
  1. Persistent Node:
     ros2 run doosan_controller temp_sequence
  2. One-shot CLI:
     ros2 run doosan_controller temp_sequence <marker_id> [--step]
"""

import json
import math
import sys
import threading
import time
import urllib.request

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Empty, String, Int32

from dsr_msgs2.srv import MoveCircle
from robot_arm_interfaces.action import MoveJ, MoveL
from robot_arm_interfaces.msg import RobotStatus
from robot_arm_interfaces.srv import Home
from rotosy_gripper_control.keyboard_electromagnet_gripper import KeyboardElectromagnetGripper

# ── 공통 파라미터 ────────────────────────────────────────────────────────────

CAMERA_API       = 'http://localhost:8000/camera/markers'
ROBOT_NS         = 'dsr01'

VEL_MM           = 30.0
ACC_MM           = 60.0
VEL_DEG          = 30.0
ACC_DEG          = 60.0

DEFAULT_RADIUS   = 180.0   # MoveC 기본 반지름 (mm)

DEFAULT_RZ       = 0.0



class TempSequenceNode(Node):
    """실시간 시퀀스 제어가 가능한 임시 시퀀스 노드 (/temp_motion/* 토픽 사용)."""

    def __init__(self):
        super().__init__('temp_sequence_node')
        self._cb_group = ReentrantCallbackGroup()

        # Clients
        self._home_cli   = self.create_client(Home, '/arm/home', callback_group=self._cb_group)
        self._movel      = ActionClient(self, MoveL, '/arm/move_l', callback_group=self._cb_group)
        self._movej      = ActionClient(self, MoveJ, '/arm/move_j', callback_group=self._cb_group)
        self._cli_circle = self.create_client(
            MoveCircle, f'/{ROBOT_NS}/motion/move_circle', callback_group=self._cb_group
        )
        self._gripper = KeyboardElectromagnetGripper()

        # Subscribers
        self._status_sub = self.create_subscription(
            RobotStatus, '/arm/status', self._status_cb, 10
        )
        self._next_sub = self.create_subscription(
            Empty, '/temp_motion/next_step', self._next_step_cb, 10, callback_group=self._cb_group
        )
        self._start_sub = self.create_subscription(
            Int32, '/temp_motion/start', self._start_cb, 10, callback_group=self._cb_group
        )
        self._stop_sub = self.create_subscription(
            Empty, '/temp_motion/stop', self._stop_cb, 10, callback_group=self._cb_group
        )

        # Publishers
        self._step_info_pub = self.create_publisher(String, '/temp_motion/step_info', 10)

        # State
        self._tcp    = None   # [x, y, z, rx, ry, rz] mm / deg
        self._joints = None   # [j1..j6] deg
        
        self._step_mode = False
        self._next_step_event = threading.Event()
        self._stop_requested = False
        self._current_sequence_thread = None
        self._is_running = False

    # ── 콜백 ─────────────────────────────────────────────────────────────────

    def _status_cb(self, msg: RobotStatus):
        self._tcp    = list(msg.current_tcp)
        self._joints = list(msg.current_joints_deg)

    def _next_step_cb(self, msg: Empty):
        self.get_logger().info('Next step signal received')
        self._next_step_event.set()

    def _stop_cb(self, msg: Empty):
        self.get_logger().info('Stop signal received')
        self._stop_requested = True
        self._next_step_event.set() # Release wait if stuck in _wait_for_step

    def _start_cb(self, msg: Int32):
        if self._is_running:
            self.get_logger().warn('Sequence is already running. Stopping previous one...')
            self._stop_requested = True
            self._next_step_event.set()
            if self._current_sequence_thread:
                self._current_sequence_thread.join(timeout=2.0)
        
        marker_id = msg.data
        self._step_mode = True 
        self._stop_requested = False
        
        self.get_logger().info(f'Starting sequence for marker {marker_id} (Step Mode: {self._step_mode})')
        self._current_sequence_thread = threading.Thread(
            target=self.run_sequence, args=(marker_id, DEFAULT_RADIUS)
        )
        self._current_sequence_thread.start()

    # ── 유틸 ─────────────────────────────────────────────────────────────────

    def _wait_for_step(self, step_name: str):
        """Step 모드일 경우 신호를 기다림."""
        if self._stop_requested:
            return False

        info_msg = String()
        info_msg.data = step_name
        self._step_info_pub.publish(info_msg)

        if not self._step_mode:
            self.get_logger().info(f'Step: {step_name}')
            return True
        
        self.get_logger().info(f'Waiting for Next Step signal to: {step_name}...')
        self._next_step_event.clear()
        self._next_step_event.wait()
        
        if self._stop_requested:
            self.get_logger().info('Step wait cancelled by stop request')
            return False

        self.get_logger().info(f'Proceeding to: {step_name}')
        return True

    def _wait_tcp(self, timeout: float = 5.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._tcp and any(v != 0.0 for v in self._tcp[:3]):
                return True
            time.sleep(0.1)
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
        while rclpy.ok() and not fut.done():
            time.sleep(0.1)
        
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
        while rclpy.ok() and not fut.done():
            time.sleep(0.05)
        handle = fut.result()
        if not handle.accepted:
            self.get_logger().error('MoveL 거부 — Servo ON 확인')
            return False
        
        res_fut = handle.get_result_async()
        while rclpy.ok() and not res_fut.done():
            time.sleep(0.1)
        
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
        while rclpy.ok() and not fut.done():
            time.sleep(0.05)
        handle = fut.result()
        if not handle.accepted:
            self.get_logger().error('MoveJ 거부')
            return False
            
        res_fut = handle.get_result_async()
        while rclpy.ok() and not res_fut.done():
            time.sleep(0.1)
            
        result = res_fut.result().result
        if result.success:
            self.get_logger().info(f'MoveJ 완료 ({result.execution_time_sec:.2f}s)')
            return True
        self.get_logger().error(f'MoveJ 실패: {result.message}')
        return False

    def _set_magnet(self, enabled: bool) -> bool:
        """전자석 ON(enabled=True) / OFF(enabled=False) — rotosy_gripper_control 패키지 사용."""
        return self._gripper.set_gripper(enabled)

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
        req.mode     = 0   # MOVE_MODE_ABSOLUTE: via/end가 절대 좌표
        req.angle1   = 0.0
        req.angle2   = 0.0
        req.blend_type = 0
        req.sync_type  = 0

        fut = self._cli_circle.call_async(req)
        while rclpy.ok() and not fut.done():
            time.sleep(0.1)
        resp = fut.result()
        if resp is None:
            self.get_logger().error('MoveC 타임아웃')
            return False
        return bool(resp.success)

    def _move_c(self, radius: float, direction: int) -> tuple | None:
        """현재 TCP 위치 기준 90° 호 이동 (test_moveC와 동일한 방향값 사용).

        direction= 1: 구 중심 = 현재 TCP 아래(-Z), +Y 방향, Z 감소
        direction=-1: 구 중심 = 현재 TCP의 -Y,     -Y 방향, Z 증가 (복귀)
        """
        if not self._tcp:
            self.get_logger().error('MoveC: TCP 위치 미수신')
            return None

        x0, y0, z0 = self._tcp[0], self._tcp[1], self._tcp[2]
        r   = float(radius)
        a45 = math.radians(45)

        if direction >= 0:
            Cx, Cy_c, Cz_c = x0, y0, z0 - r
            via = [Cx, Cy_c + r * math.sin(a45), Cz_c + r * math.cos(a45),
                   -90.0, 135.0, 0.0]
            end = [Cx, Cy_c + r,                  Cz_c,
                   -90.0,  90.0, 0.0]
            desc = f'+Y Z감소  r={r:.0f}mm'
        else:
            Cx, Cy_c, Cz_c = x0, y0 - r, z0
            via = [Cx, Cy_c + r * math.sin(a45), Cz_c + r * math.cos(a45),
                   -90.0, 135.0, 0.0]
            end = [Cx, Cy_c,                      Cz_c + r,
                   -90.0, 180.0, 0.0]
            desc = f'-Y Z증가  r={r:.0f}mm'

        self.get_logger().info(
            f'MoveC {desc}  시작({x0:.1f},{y0:.1f},{z0:.1f})'
            f'  end({end[0]:.1f},{end[1]:.1f},{end[2]:.1f})'
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
        """지정 마커 위치 + Y +30mm 로 이동 (마커 0~5 공통)."""
        self.get_logger().info(f'[마커{marker_id}] 좌표 취득 중...')
        pos = self._get_marker(marker_id)
        if pos is None:
            self.get_logger().error(f'ID {marker_id} 마커 미감지')
            return None
        mx, my, mz = pos
        ty = my + 30.0
        self.get_logger().info(f'마커 위치: ({mx:.1f}, {my:.1f}, {mz:.1f}) mm → 목표 Y: {ty:.1f} mm')
        if not self._move_l(mx, ty, mz, 90.0, -90.0, 0.0):
            return None
        return mx, ty, mz

    def run_sequence(self, marker_id: int, radius_mm: float):
        self._is_running = True
        try:
            # 1. 홈
            if not self._wait_for_step('홈 이동'): return
            if not self._home(): return

            if not self._wait_tcp():
                self.get_logger().error('TCP 위치 미수신')
                return

            # 2. 마커로 이동
            if not self._wait_for_step(f'마커 {marker_id}로 이동'): return
            result = self._goto_marker(marker_id)
            if result is None: return
            mx, my, mz = result
            cx, cy, cz = mx, my, mz

            # 2-1. Y -2.5cm 상대 이동
            if not self._wait_for_step('Y -25mm 상대 이동'): return
            if not self._move_l(0, -25, 0, relative=True): return
            cy -= 25

            # 2-2. 전자석 ON
            if not self._wait_for_step('전자석 ON'): return
            if not self._set_magnet(True): return

            # 3. Y +15cm 상대 이동
            if not self._wait_for_step('Y +150mm 상대 이동'): return
            if not self._move_l(0, 150, 0, relative=True): return
            cy += 150

            # 3-1. 전자석 OFF
            if not self._wait_for_step('전자석 OFF'): return
            if not self._set_magnet(False): return

            # 4. Y +3cm 상대 이동
            if not self._wait_for_step('Y +30mm 상대 이동'): return
            if not self._move_l(0, 30, 0, relative=True): return
            cy += 30

            # 5번 MoveC 시작 전 실제 TCP 전체(위치+자세) 기억
            if not self._wait_tcp():
                self.get_logger().error('MoveC 전 TCP 위치 미수신')
                return
            pre_movec_pos = tuple(self._tcp[:6])  # (x, y, z, rx, ry, rz)
            cx, cy, cz = pre_movec_pos[0], pre_movec_pos[1], pre_movec_pos[2]

            # 5. MoveC r=200 direction=-1 (복귀 호)
            if not self._wait_for_step('MoveC r=200 복귀 호'): return
            end_pos = self._move_c(200, -1)
            if end_pos is None: return
            cx, cy, cz = end_pos

            # 5-1. MoveL 상대이동 (Z-100)
            if not self._wait_for_step('MoveL 상대 (Z-100)'): return
            if not self._move_l(0, 0, -100, relative=True): return
            cz -= 100


            # 5-2. 전자석 ON
            if not self._wait_for_step('전자석 ON'): return
            if not self._set_magnet(True): return

            # 5-1. MoveL 상대이동 (Z+100)
            if not self._wait_for_step('MoveL 상대 (Z+100)'): return
            if not self._move_l(0, 0, 100, relative=True): return
            cz += 100


            # 5-3. MoveL 상대이동 (Y+50, Z+50)
            if not self._wait_for_step('MoveL 상대 (Y+50, Z+50)'): return
            if not self._move_l(0, 50, 50, relative=True): return
            cy += 50
            cz += 50

            # 5-4. MoveJ 특정 포즈
            if not self._wait_for_step('MoveJ 특정 포즈1'): return
            target_joints = [20.44, 53.45, 49.27, 158.82, 97.28, -205.62]
            if not self._move_j(target_joints): return

            # 5-5. MoveJ 특정 포즈
            if not self._wait_for_step('MoveJ 특정 포즈2'): return
            target_joints = [34.01, 53.55, 29.86, -2.98, 97.34, -238.46]
            if not self._move_j(target_joints): return

            # 5-5. 전자석 OFF (배치)
            if not self._wait_for_step('전자석 OFF'): return
            if not self._set_magnet(False): return


            # 5-5-1. MoveJ 특정 포즈
            if not self._wait_for_step('MoveJ 특정 포즈3'): return
            target_joints = [36.69, 29.17, 87.29, -75.19, 117.19, -59.94]
            if not self._move_j(target_joints): return


            # 5-5. MoveC 복귀 호 전 위치로 복귀 (저장된 위치+자세 그대로 복귀)
            if not self._wait_for_step('5-5. 복귀 전 위치로 이동'): return
            if not self._move_l(pre_movec_pos[0], pre_movec_pos[1], pre_movec_pos[2],
                               rx=pre_movec_pos[3], ry=pre_movec_pos[4], rz=pre_movec_pos[5]): return
            cx, cy, cz = pre_movec_pos[0], pre_movec_pos[1], pre_movec_pos[2]

            # 6. Y -3cm 상대 이동
            if not self._wait_for_step('Y -30mm 상대 이동'): return
            if not self._move_l(0, -30, 0, relative=True): return
            cy -= 30

            if not self._wait_for_step('전자석 ON'): return
            if not self._set_magnet(True): return


            # 7. Y -15cm 상대 이동
            if not self._wait_for_step('Y -150mm 상대 이동'): return
            if not self._move_l(0, -150, 0, relative=True): return
            cy -= 150


            if not self._wait_for_step('전자석 OFF'): return
            if not self._set_magnet(False): return

            if not self._wait_for_step('MoveL 상대 (Y+50)'): return
            if not self._move_l(0, 50, 0, relative=True): return
            cy += 50

            self.get_logger().info('시퀀스 완료')
            self._wait_for_step('시퀀스 완료')
        except Exception as e:
            self.get_logger().error(f'Sequence error: {e}')
        finally:
            self._is_running = False
            self._step_info_pub.publish(String(data='IDLE'))


def main(args=None):
    rclpy.init(args=args)
    node = TempSequenceNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # CLI 지원
    if len(sys.argv) > 1 and sys.argv[1].isdigit():
        marker_id = int(sys.argv[1])
        radius = float(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_RADIUS
        node._step_mode = '--step' in sys.argv
        seq_thread = threading.Thread(target=node.run_sequence, args=(marker_id, radius))
        seq_thread.start()

    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
