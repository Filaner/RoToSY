#!/usr/bin/env python3
"""
Localization bootstrap node.

로봇은 항상 약실(spawn 위치)에서 시작하므로 초기 위치는 고정값으로 알고 있다.
이 노드는 AMCL이 위치를 확정할 수 있도록 시작 시점에 /initialpose 를 몇 번
반복 발행한 뒤 조용히 종료한다. (데모 골 전송 책임은 없음 — 골은 웹이 보낸다.)

demo_goal_sender 에서 골 전송 로직을 떼어내고 initialpose 발행만 남긴 버전.
"""

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


def yaw_to_quaternion(yaw):
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('mobile_initial_pose_publisher')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('initial_x', -4.3)
        self.declare_parameter('initial_y', 2.05)
        self.declare_parameter('initial_yaw', -1.5708)
        self.declare_parameter('repetitions', 6)
        self.declare_parameter('period_sec', 1.0)

        self.map_frame    = self.get_parameter('map_frame').value
        self.repetitions  = int(self.get_parameter('repetitions').value)
        self.count        = 0

        self.pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.create_timer(float(self.get_parameter('period_sec').value), self._tick)

    def _tick(self):
        if self.count >= self.repetitions:
            self.get_logger().info('Initial pose published — localization bootstrapped, shutting down')
            raise SystemExit  # 발행 끝나면 노드 종료

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(self.get_parameter('initial_x').value)
        msg.pose.pose.position.y = float(self.get_parameter('initial_y').value)
        qx, qy, qz, qw = yaw_to_quaternion(float(self.get_parameter('initial_yaw').value))
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance[0]  = 0.25
        msg.pose.covariance[7]  = 0.25
        msg.pose.covariance[35] = 0.0685
        self.pub.publish(msg)

        self.count += 1
        self.get_logger().info(f'Published initial pose ({self.count}/{self.repetitions})')


def main():
    rclpy.init()
    node = InitialPosePublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
