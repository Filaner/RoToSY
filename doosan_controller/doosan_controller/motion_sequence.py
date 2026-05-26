import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from robot_arm_interfaces.action import MoveL, MoveJ
from robot_arm_interfaces.srv import WaitDuration


class MotionSequenceNode(Node):
    def __init__(self):
        super().__init__('motion_sequence_node')
        self._movel_client = ActionClient(self, MoveL, '/arm/move_l')
        self._movej_client = ActionClient(self, MoveJ, '/arm/move_j')
        self._wait_client = self.create_client(WaitDuration, '/arm/wait')

    def wait_for_servers(self):
        self.get_logger().info('서버 대기 중...')
        self._movel_client.wait_for_server()
        self._movej_client.wait_for_server()
        while not self._wait_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Wait 서비스 대기 중...')

    def movel(self, x, y, z, vel=30.0, relative=True):
        """직선 이동 명령 (상대 좌표)"""
        goal = MoveL.Goal()
        goal.x, goal.y, goal.z = float(x), float(y), float(z)
        goal.linear_velocity_mm_s = vel
        goal.relative = relative
        self.get_logger().info(f'MoveL 전송: {x}, {y}, {z}')
        return self._send_action_goal(self._movel_client, goal)

    def movej(self, joints, vel=20.0):
        """관절 이동 명령 (절대 좌표)"""
        goal = MoveJ.Goal()
        goal.joint_angles_deg = [float(j) for j in joints]
        goal.velocity_deg_s = vel
        self.get_logger().info(f'MoveJ 전송: {joints}')
        return self._send_action_goal(self._movej_client, goal)

    def sleep(self, seconds):
        """로봇 정지 대기"""
        req = WaitDuration.Request()
        req.duration_sec = float(seconds)
        self.get_logger().info(f'{seconds}초 대기...')
        future = self._wait_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def _send_action_goal(self, client, goal):
        """액션을 보내고 완료될 때까지 기다리는 내부 함수"""
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle.accepted:
            return False
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().result.success


def main(args=None):
    rclpy.init(args=args)
    node = MotionSequenceNode()
    node.wait_for_servers()

    try:

        # 1. 오른쪽으로 50mm 이동
        node.movel(50.0, 0.0, 0.0, vel=40.0)
        node.sleep(1.0)

        # 2. 위로 30mm 이동
        node.movel(0.0, 0.0, 30.0, vel=20.0)

        # 3. 왼쪽으로 50mm 이동 (복귀)
        node.movel(-50.0, 0.0, -30.0, vel=40.0)
        
        node.movej([0.0, 0.0, 90.0, 0.0, 90.0, 0.0], vel=30.0)

        node.get_logger().info('전체 시퀀스 완료!')

    except Exception as e:
        node.get_logger().error(f'에러 발생: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()