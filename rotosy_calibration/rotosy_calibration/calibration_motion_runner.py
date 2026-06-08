#!/usr/bin/env python3
import os
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_srvs.srv import Trigger

from robot_arm_interfaces.action import MoveJ
from robot_arm_interfaces.srv import Home


class CalibrationMotionRunner(Node):
    def __init__(self):
        super().__init__('calibration_motion_runner')

        self.declare_parameter('move_j_action', '/arm/move_j')
        self.declare_parameter(
            'joint_poses_deg',
            [0.0, 0.0, 90.0, 0.0, 90.0, 0.0],
            ParameterDescriptor(description='Flat list of 6 joint values per pose, unit is degree.'),
        )
        self.declare_parameter('velocity_deg_s', 15.0)
        self.declare_parameter('acceleration_deg_s2', 30.0)
        self.declare_parameter('settle_time_sec', 1.2)
        self.declare_parameter('start_delay_sec', 3.0)
        self.declare_parameter('take_sample_service', '/calibration/take_sample')
        self.declare_parameter('sample_service_timeout_sec', 45.0)
        self.declare_parameter('home_service', '/arm/home')
        self.declare_parameter('return_home_on_finish', True)
        self.declare_parameter(
            'optimized_motion_path',
            '/home/cheol/RoToSY_ws/src/rotosy_calibration/config/optimized_calibration_motion.yaml',
        )
        self.declare_parameter('loop', False)
        self.declare_parameter('auto_start', True)

        self.move_j_action = self.get_parameter('move_j_action').value
        self.joint_poses = self._parse_joint_poses(self.get_parameter('joint_poses_deg').value)
        self.velocity = float(self.get_parameter('velocity_deg_s').value)
        self.acceleration = float(self.get_parameter('acceleration_deg_s2').value)
        self.settle_time_sec = float(self.get_parameter('settle_time_sec').value)
        self.start_delay_sec = float(self.get_parameter('start_delay_sec').value)
        self.take_sample_service = self.get_parameter('take_sample_service').value
        self.sample_service_timeout_sec = float(self.get_parameter('sample_service_timeout_sec').value)
        self.home_service = self.get_parameter('home_service').value
        self.return_home_on_finish = bool(self.get_parameter('return_home_on_finish').value)
        self.optimized_motion_path = self.get_parameter('optimized_motion_path').value
        self.loop = bool(self.get_parameter('loop').value)
        self.auto_start = bool(self.get_parameter('auto_start').value)

        self.client = ActionClient(self, MoveJ, self.move_j_action)
        self.sample_client = self.create_client(Trigger, self.take_sample_service)
        self.home_client = self.create_client(Home, self.home_service)
        self.accepted_poses = []
        self.rejected_poses = []

        if not self.auto_start:
            self.get_logger().info('auto_start=false. Motion runner is idle.')

    def _parse_joint_poses(self, flat_values):
        values = [float(v) for v in flat_values]
        if len(values) == 0:
            raise ValueError('joint_poses_deg is empty')
        if len(values) % 6 != 0:
            raise ValueError('joint_poses_deg must contain 6 values per pose')
        return [values[i:i + 6] for i in range(0, len(values), 6)]

    def run_sequence(self):
        self.get_logger().info(f'Waiting {self.start_delay_sec:.1f}s before calibration motion')
        time.sleep(self.start_delay_sec)

        while not self.client.wait_for_server(timeout_sec=2.0):
            self.get_logger().info(f'Waiting for action server: {self.move_j_action}')
        sample_wait_started = time.monotonic()
        while not self.sample_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info(f'Waiting for sample service: {self.take_sample_service}')
            elapsed = time.monotonic() - sample_wait_started
            if elapsed >= self.sample_service_timeout_sec:
                self.get_logger().error(
                    f'Sample service unavailable after {elapsed:.1f}s: {self.take_sample_service}. '
                    'The aruco_camera_calibrator node is probably not running.'
                )
                return False

        while rclpy.ok():
            for index, joints in enumerate(self.joint_poses, start=1):
                ok = self._move_j(index, joints)
                if not ok:
                    self.get_logger().error('Stopping calibration motion sequence')
                    self._save_optimized_motion()
                    self._return_home_if_enabled()
                    return False
                time.sleep(self.settle_time_sec)
                sample_ok, message = self._take_sample(index)
                if sample_ok:
                    self.accepted_poses.append(joints)
                    self.get_logger().info(f'Pose {index} accepted: {message}')
                else:
                    self.rejected_poses.append(joints)
                    self.get_logger().warning(f'Pose {index} excluded: {message}')

            if not self.loop:
                self._save_optimized_motion()
                self._return_home_if_enabled()
                self.get_logger().info('Calibration motion sequence complete')
                return True

    def _move_j(self, index, joints):
        goal = MoveJ.Goal()
        goal.joint_angles_deg = [float(v) for v in joints]
        goal.velocity_deg_s = self.velocity
        goal.acceleration_deg_s2 = self.acceleration
        goal.blend_radius_mm = 0.0
        goal.relative = False

        self.get_logger().info(f'MoveJ calibration pose {index}/{len(self.joint_poses)}: {goal.joint_angles_deg}')
        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('MoveJ goal rejected')
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        if not result.success:
            self.get_logger().error(f'MoveJ failed: {result.message}')
            return False
        return True

    def _take_sample(self, index):
        self.get_logger().info(f'Requesting calibration sample at pose {index}')
        future = self.sample_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is None:
            return False, 'sample service returned no response'
        return bool(response.success), response.message

    def _return_home_if_enabled(self):
        if not self.return_home_on_finish:
            return

        self.get_logger().info(f'Returning robot home through {self.home_service}')
        if not self.home_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(f'Home service unavailable: {self.home_service}')
            return

        request = Home.Request()
        request.target = 1
        future = self.home_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is None:
            self.get_logger().error('Home service returned no response')
        elif response.success:
            self.get_logger().info(f'Home complete: {response.message}')
        else:
            self.get_logger().error(f'Home failed: {response.message}')

    def _save_optimized_motion(self):
        if not self.accepted_poses:
            self.get_logger().warning('No accepted calibration poses; optimized motion file was not written')
            return

        os.makedirs(os.path.dirname(self.optimized_motion_path), exist_ok=True)
        flat_values = []
        for pose in self.accepted_poses:
            flat_values.extend(float(v) for v in pose)

        lines = [
            'calibration_motion_runner:',
            '  ros__parameters:',
            f'    move_j_action: "{self.move_j_action}"',
            f'    velocity_deg_s: {self.velocity:.6f}',
            f'    acceleration_deg_s2: {self.acceleration:.6f}',
            f'    settle_time_sec: {self.settle_time_sec:.6f}',
            f'    start_delay_sec: {self.start_delay_sec:.6f}',
            f'    take_sample_service: "{self.take_sample_service}"',
            f'    sample_service_timeout_sec: {self.sample_service_timeout_sec:.6f}',
            f'    home_service: "{self.home_service}"',
            f'    return_home_on_finish: {str(self.return_home_on_finish).lower()}',
            f'    optimized_motion_path: "{self.optimized_motion_path}"',
            '    loop: false',
            '    auto_start: true',
            '',
            f'    # Accepted poses: {len(self.accepted_poses)}',
            f'    # Rejected poses: {len(self.rejected_poses)}',
            '    joint_poses_deg: [',
        ]
        for pose_index, pose in enumerate(self.accepted_poses):
            formatted = ', '.join(f'{value:.6f}' for value in pose)
            suffix = ',' if pose_index < len(self.accepted_poses) - 1 else ''
            lines.append(f'      {formatted}{suffix}')
        lines.extend([
            '    ]',
        ])

        with open(self.optimized_motion_path, 'w', encoding='utf-8') as f:
            f.write('\n'.join(lines) + '\n')

        self.get_logger().info(
            f'Saved optimized calibration motion: {self.optimized_motion_path} '
            f'({len(self.accepted_poses)} accepted, {len(self.rejected_poses)} rejected)'
        )


def main():
    rclpy.init()
    node = CalibrationMotionRunner()
    try:
        if node.auto_start:
            node.run_sequence()
        else:
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
