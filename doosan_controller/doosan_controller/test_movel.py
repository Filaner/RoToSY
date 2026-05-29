"""
ros2 run doosan_controller test_movel

X축으로 100mm(10cm) 상대 이동 테스트 스크립트.
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from robot_arm_interfaces.action import MoveL


class TestMoveL(Node):
    def __init__(self):
        super().__init__('test_movel')
        self._client = ActionClient(self, MoveL, '/arm/move_l')

    def send_goal(self):
        self.get_logger().info('액션 서버 대기 중...')
        self._client.wait_for_server()

        goal = MoveL.Goal()
        goal.x                      = 100.0   # mm (+X 방향 10cm)
        goal.y                      = 0.0
        goal.z                      = 0.0
        goal.rx                     = 0.0
        goal.ry                     = 0.0
        goal.rz                     = 0.0
        goal.linear_velocity_mm_s   = 30.0
        goal.angular_velocity_deg_s = 30.0
        goal.linear_accel_mm_s2     = 60.0
        goal.angular_accel_deg_s2   = 60.0
        goal.blend_radius_mm        = 0.0
        goal.reference_frame        = 0       # 0 = BASE 좌표계
        goal.relative               = True

        self.get_logger().info('MoveL 목표 전송: X +100mm (상대)')
        future = self._client.send_goal_async(
            goal, feedback_callback=self._feedback_cb
        )
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('목표 거부됨 (Servo ON 상태인지 확인)')
            rclpy.shutdown()
            return
        self.get_logger().info('목표 수락됨 — 이동 시작')
        handle.get_result_async().add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'  TCP: X={fb.current_x:7.2f}  Y={fb.current_y:7.2f}  Z={fb.current_z:7.2f} mm'
            f'  [{fb.robot_state}]'
        )

    def _result_cb(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info(
                f'완료! 소요시간: {result.execution_time_sec:.2f}s'
            )
        else:
            self.get_logger().error(f'실패: {result.message}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = TestMoveL()
    node.send_goal()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
