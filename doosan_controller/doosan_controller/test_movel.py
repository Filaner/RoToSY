"""
ros2 run doosan_controller test_movel

고정 pose [621.77, 107.18, 508.20, 26.81, -179.66, 112.51] (x,y,z,rx,ry,rz)로
목표 자세를 직접 IK 연산해 MoveJ로 이동한 뒤, 남는 오차만 MoveL로 보정하는 단발성 테스트.

시퀀스:
  1. 현재 TCP/관절 취득 (/arm/status)
  2. 목표 pose 자체를 DSR ikin으로 역기구학 계산
     (8개 solution space 중 현재 관절과 가장 가까운 해 선택)
  3. MoveJ → 그 joint 값 (이미 목표 pose에 도달)
  4. MoveL → 목표 pose (IK/인코더 잔차 보정)
  IK 실패 시에는 MoveJ 없이 목표로 곧장 MoveL (passthrough).
"""

import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from dsr_msgs2.srv import Ikin
from robot_arm_interfaces.action import MoveJ, MoveL
from robot_arm_interfaces.msg import RobotStatus

ROBOT_NS = 'dsr01'

TARGET_POSE = [465.78, 499.20, 656.0, 20.48, 51.86, 49.19]   # x,y,z mm / rx,ry,rz deg

# E0509 실제 관절 한계 (deg) — dsr_moveit_config_e0509/config/joint_limits.yaml 기준,
# 실기 알람(JNT(2) MAX 95 / MIN -95)과 일치 확인됨. URDF의 ±360°는 실제 한계가 아님.
JOINT_LIMITS_DEG = (
    (-180.0, 180.0),
    (-95.0, 95.0),
    (-135.0, 135.0),
    (-180.0, 180.0),
    (-135.0, 135.0),
    (-180.0, 180.0),
)

VEL_MM  = 30.0
ACC_MM  = 60.0
VEL_DEG = 30.0
ACC_DEG = 60.0


def _within_joint_limits(joints: list) -> bool:
    return all(lo <= j <= hi for j, (lo, hi) in zip(joints, JOINT_LIMITS_DEG))


class TestHybridMoveL(Node):
    def __init__(self):
        super().__init__('test_movel')

        self._movel    = ActionClient(self, MoveL, '/arm/move_l')
        self._movej    = ActionClient(self, MoveJ, '/arm/move_j')
        self._ikin_cli = self.create_client(Ikin, f'/{ROBOT_NS}/motion/ikin')
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

    def _wait_status(self, timeout: float = 3.0) -> bool:
        """TCP + 관절 위치 수신 대기."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._tcp and self._joints and any(v != 0.0 for v in self._tcp[:3]):
                return True
        return False

    def _dsr_ik(self, pose6: list) -> tuple:
        """DSR ikin을 8개 solution space에 대해 호출, 현재 관절과 가장 가까운 해 반환.

        Returns:
            (joint_angles_deg, '') 성공 시
            (None, reason_str)    실패 시
        """
        if not self._ikin_cli.wait_for_service(timeout_sec=2.0):
            return None, 'ikin 서비스 없음'

        best_joints = None
        best_dist   = float('inf')

        for sol in range(8):
            req           = Ikin.Request()
            req.pos       = [float(v) for v in pose6]
            req.sol_space = sol
            req.ref       = 0   # DR_BASE

            fut = self._ikin_cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut)
            resp = fut.result()
            if resp is None or not resp.success:
                continue

            joints = list(resp.conv_posj)
            if not _within_joint_limits(joints):
                continue
            dist = sum((a - b) ** 2 for a, b in zip(joints, self._joints))
            if dist < best_dist:
                best_dist   = dist
                best_joints = joints

        if best_joints is None:
            return None, '8개 solution space 모두 실패 또는 관절 한계 초과'
        return best_joints, ''

    def _move_l(self, x, y, z, rx, ry, rz) -> bool:
        """절대 좌표 MoveL."""
        self.get_logger().info(
            f'MoveL → ({x:.1f}, {y:.1f}, {z:.1f}) mm  rx={rx:.1f} ry={ry:.1f} rz={rz:.1f}'
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
            self.get_logger().error('MoveL 거부됨 — Servo ON 여부 확인')
            return False

        res_fut = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        result = res_fut.result().result
        if result.success:
            self.get_logger().info(f'MoveL 완료 ({result.execution_time_sec:.2f}s)')
            return True
        self.get_logger().error(f'MoveL 실패: {result.message}')
        return False

    def _move_j(self, joint_angles: list) -> bool:
        """절대 관절 각도 MoveJ (사전접근 — blend 없음)."""
        self.get_logger().info(
            f'MoveJ → {[round(a, 1) for a in joint_angles]}'
        )
        self._movej.wait_for_server()

        goal = MoveJ.Goal()
        goal.joint_angles_deg    = [float(a) for a in joint_angles]
        goal.velocity_deg_s      = VEL_DEG
        goal.acceleration_deg_s2 = ACC_DEG
        goal.blend_radius_mm     = 0.0

        send_fut = self._movej.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_fut)
        handle = send_fut.result()
        if not handle.accepted:
            self.get_logger().error('MoveJ 거부됨')
            return False

        res_fut = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        result = res_fut.result().result
        if result.success:
            self.get_logger().info(f'MoveJ 완료 ({result.execution_time_sec:.2f}s)')
            return True
        self.get_logger().error(f'MoveJ 실패: {result.message}')
        return False

    # ── main sequence ────────────────────────────────────────────────────────

    def run(self):
        if not self._wait_status():
            self.get_logger().error('TCP/관절 위치 미수신')
            return

        cur = self._tcp[:3]
        tx, ty, tz, rx, ry, rz = TARGET_POSE
        dist = sum((a - b) ** 2 for a, b in zip(cur, (tx, ty, tz))) ** 0.5
        self.get_logger().info(
            f'현재 TCP: ({cur[0]:.1f}, {cur[1]:.1f}, {cur[2]:.1f}) mm  '
            f'목표: ({tx:.1f}, {ty:.1f}, {tz:.1f}) mm  거리={dist:.1f}mm'
        )

        # 목표 pose 자체에 대해 IK 계산 (사전접근 오프셋 없음)
        target_pose6 = [tx, ty, tz, rx, ry, rz]

        self.get_logger().info('[1] 목표 pose IK 계산 중...')
        ik_joints, err_msg = self._dsr_ik(target_pose6)
        if ik_joints is None:
            self.get_logger().warning(f'DSR IK 실패({err_msg}) — MoveL 단독 이동으로 대체')
            self._move_l(tx, ty, tz, rx, ry, rz)
            return

        self.get_logger().info(f'IK → {[round(j, 1) for j in ik_joints]}')

        self.get_logger().info('[2] MoveJ → 목표 joint')
        if not self._move_j(ik_joints):
            return
        time.sleep(0.2)

        rclpy.spin_once(self, timeout_sec=0.2)
        if self._tcp:
            px, py, pz, prx, pry, prz = self._tcp[:6]
            self.get_logger().info(
                f'MoveJ 도달 후 실제 TCP: ({px:.1f}, {py:.1f}, {pz:.1f}) mm '
                f'rx={prx:.1f} ry={pry:.1f} rz={prz:.1f}  '
                f'(목표 대비 오차: dx={tx-px:+.1f} dy={ty-py:+.1f} dz={tz-pz:+.1f} '
                f'drx={rx-prx:+.1f} dry={ry-pry:+.1f} drz={rz-prz:+.1f})'
            )

        self.get_logger().info('[3] MoveL → 잔차 보정')
        self._move_l(tx, ty, tz, rx, ry, rz)


def main(args=None):
    rclpy.init(args=args)
    node = TestHybridMoveL()
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
