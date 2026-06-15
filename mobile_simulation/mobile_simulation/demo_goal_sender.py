#!/usr/bin/env python3
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


def yaw_to_quaternion(yaw):
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class DemoGoalSender(Node):
    def __init__(self):
        super().__init__('mobile_demo_goal_sender')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('initial_x', -4.3)
        self.declare_parameter('initial_y', 2.5)
        self.declare_parameter('initial_yaw', -1.5708)
        self.declare_parameter('goal_x', 4.2)
        self.declare_parameter('goal_y', 2.72)
        self.declare_parameter('goal_yaw', 1.5708)
        self.declare_parameter('initial_pose_repetitions', 6)
        self.declare_parameter('goal_start_delay_sec', 8.0)

        self.map_frame = self.get_parameter('map_frame').value
        self.started_at = time.monotonic()
        self.initial_pose_count = 0
        self.goal_sent = False
        self.result_future = None

        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.create_timer(1.0, self._tick)

    def _tick(self):
        self._publish_initial_pose()
        if self.result_future is not None:
            self._check_result()
            return
        if self.goal_sent:
            return
        if time.monotonic() - self.started_at < float(self.get_parameter('goal_start_delay_sec').value):
            return
        if not self.nav_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().info('Waiting for Nav2 navigate_to_pose action server')
            return
        self._send_goal()

    def _publish_initial_pose(self):
        repetitions = int(self.get_parameter('initial_pose_repetitions').value)
        if self.initial_pose_count >= repetitions:
            return

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(self.get_parameter('initial_x').value)
        msg.pose.pose.position.y = float(self.get_parameter('initial_y').value)
        qx, qy, qz, qw = yaw_to_quaternion(float(self.get_parameter('initial_yaw').value))
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685
        self.initial_pose_pub.publish(msg)
        self.initial_pose_count += 1
        self.get_logger().info(f'Published initial pose ({self.initial_pose_count}/{repetitions})')

    def _send_goal(self):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(self.get_parameter('goal_x').value)
        goal.pose.pose.position.y = float(self.get_parameter('goal_y').value)
        qx, qy, qz, qw = yaw_to_quaternion(float(self.get_parameter('goal_yaw').value))
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.goal_sent = True
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_cb)
        self.get_logger().info(
            f"Sent demo navigation goal: ({goal.pose.pose.position.x:.2f}, {goal.pose.pose.position.y:.2f})"
        )

    def _goal_response_cb(self, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('Nav2 rejected demo navigation goal')
            return
        self.result_future = handle.get_result_async()

    def _check_result(self):
        if not self.result_future.done():
            return
        status = self.result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Demo navigation goal reached')
        else:
            self.get_logger().warning(f'Demo navigation finished with status {status}')
        self.result_future = None


def main():
    rclpy.init()
    node = DemoGoalSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
