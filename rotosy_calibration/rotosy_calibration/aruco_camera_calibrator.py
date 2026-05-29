#!/usr/bin/env python3
import os
import traceback
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from rotosy_calibration.tf_messages import transform_to_msg
from rotosy_calibration.transform_math import (
    average_transforms,
    invert_transform,
    make_transform_from_rotation_translation,
    matrix_to_quaternion,
    rotation_angle,
    transform_from_rvec_tvec,
    transform_from_msg,
)


ARUCO_DICTIONARIES = {
    'DICT_4X4_50': cv2.aruco.DICT_4X4_50,
    'DICT_4X4_100': cv2.aruco.DICT_4X4_100,
    'DICT_5X5_100': cv2.aruco.DICT_5X5_100,
    'DICT_6X6_250': cv2.aruco.DICT_6X6_250,
    'DICT_APRILTAG_36h11': cv2.aruco.DICT_APRILTAG_36h11,
}


class ArucoCameraCalibrator(Node):
    def __init__(self):
        super().__init__('aruco_camera_calibrator')

        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('gripper_frame', 'link_6')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('marker_frame', 'calibration_aruco_marker')
        self.declare_parameter('marker_id', 0)
        self.declare_parameter('marker_size_m', 0.05)
        self.declare_parameter('aruco_dictionary', 'DICT_4X4_50')
        self.declare_parameter('samples_required', 15)
        self.declare_parameter('max_samples', 100)
        self.declare_parameter('min_sample_interval_sec', 0.7)
        self.declare_parameter('min_translation_delta_m', 0.01)
        self.declare_parameter('min_rotation_delta_deg', 5.0)
        self.declare_parameter('sample_mode', 'auto')
        self.declare_parameter('take_sample_service', '/calibration/take_sample')
        self.declare_parameter('max_marker_age_sec', 0.5)
        self.declare_parameter('publish_marker_tf', True)
        self.declare_parameter('publish_static_tf_on_complete', True)
        self.declare_parameter('save_result', True)
        self.declare_parameter(
            'result_path',
            str(Path.home() / 'ros2_ws/src/RoToSY/rotosy_calibration/config/camera_extrinsic.yaml'),
        )

        self.image_topic = self.get_parameter('image_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.gripper_frame = self.get_parameter('gripper_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.marker_frame = self.get_parameter('marker_frame').value
        self.marker_id = int(self.get_parameter('marker_id').value)
        self.marker_size_m = float(self.get_parameter('marker_size_m').value)
        self.samples_required = int(self.get_parameter('samples_required').value)
        self.max_samples = int(self.get_parameter('max_samples').value)
        self.min_sample_interval_sec = float(self.get_parameter('min_sample_interval_sec').value)
        self.min_translation_delta_m = float(self.get_parameter('min_translation_delta_m').value)
        self.min_rotation_delta_rad = np.deg2rad(float(self.get_parameter('min_rotation_delta_deg').value))
        self.sample_mode = self.get_parameter('sample_mode').value
        self.take_sample_service = self.get_parameter('take_sample_service').value
        self.max_marker_age_sec = float(self.get_parameter('max_marker_age_sec').value)
        self.publish_marker_tf = bool(self.get_parameter('publish_marker_tf').value)
        self.publish_static_tf_on_complete = bool(self.get_parameter('publish_static_tf_on_complete').value)
        self.save_result = bool(self.get_parameter('save_result').value)
        self.result_path = self.get_parameter('result_path').value

        self.camera_matrix = None
        self.dist_coeffs = None
        self.base_to_gripper_samples = []
        self.camera_to_marker_samples = []
        self.last_sample_time = None
        self.last_sample_gripper = None
        self.latest_camera_to_marker = None
        self.latest_marker_time = None
        self.calibrated_transform = None
        self.gripper_to_marker = None
        self.result_saved = False

        dictionary_name = self.get_parameter('aruco_dictionary').value
        if dictionary_name not in ARUCO_DICTIONARIES:
            raise ValueError(f'Unsupported aruco_dictionary: {dictionary_name}')
        self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICTIONARIES[dictionary_name])
        if hasattr(cv2.aruco, 'DetectorParameters'):
            self.aruco_params = cv2.aruco.DetectorParameters()
        else:
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.aruco_detector = None
        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dictionary, self.aruco_params)

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        self.create_subscription(CameraInfo, self.camera_info_topic, self._camera_info_cb, 10)
        self.create_subscription(Image, self.image_topic, self._image_cb, 10)
        self.create_service(Trigger, self.take_sample_service, self._take_sample_cb)

        self.get_logger().info(
            f'Hand-eye calibration mode. Move {self.gripper_frame} through many poses while '
            f'marker id={self.marker_id} is visible on {self.image_topic}. sample_mode={self.sample_mode}'
        )

    def _camera_info_cb(self, msg):
        self.camera_matrix = np.array(msg.k, dtype=float).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=float)

    def _image_cb(self, msg):
        if self.camera_matrix is None:
            return

        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warning(f'Failed to convert image: {exc}')
            return

        corners, ids = self._detect_markers(image)
        if ids is None:
            return

        flat_ids = ids.flatten()
        matches = np.where(flat_ids == self.marker_id)[0]
        if len(matches) == 0:
            return

        marker_index = int(matches[0])
        try:
            rvec, tvec = self._estimate_marker_pose(corners[marker_index])
        except Exception as exc:
            self.get_logger().warning(f'Failed to estimate marker pose: {exc}')
            return

        camera_to_marker = transform_from_rvec_tvec(rvec, tvec)
        self.latest_camera_to_marker = camera_to_marker
        self.latest_marker_time = self.get_clock().now()

        if self.publish_marker_tf:
            self.tf_broadcaster.sendTransform(
                transform_to_msg(camera_to_marker, self.camera_frame, self.marker_frame, msg.header.stamp)
            )

        if self.sample_mode == 'service':
            return

        self._try_collect_sample(camera_to_marker)

    def _detect_markers(self, image):
        if self.aruco_detector is not None:
            corners, ids, _ = self.aruco_detector.detectMarkers(image)
            return corners, ids

        if hasattr(cv2.aruco, 'detectMarkers'):
            corners, ids, _ = cv2.aruco.detectMarkers(
                image,
                self.aruco_dictionary,
                parameters=self.aruco_params,
            )
            return corners, ids

        raise RuntimeError('OpenCV aruco marker detection API is unavailable')

    def _estimate_marker_pose(self, marker_corners):
        if hasattr(cv2.aruco, 'estimatePoseSingleMarkers'):
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                [marker_corners],
                self.marker_size_m,
                self.camera_matrix,
                self.dist_coeffs,
            )
            return rvecs[0][0], tvecs[0][0]

        half = self.marker_size_m / 2.0
        object_points = np.array(
            [
                [-half, half, 0.0],
                [half, half, 0.0],
                [half, -half, 0.0],
                [-half, -half, 0.0],
            ],
            dtype=np.float32,
        )
        image_points = np.asarray(marker_corners, dtype=np.float32).reshape(4, 2)
        ok, rvec, tvec = cv2.solvePnP(
            object_points,
            image_points,
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if not ok:
            raise RuntimeError('cv2.solvePnP failed')
        return rvec.reshape(3), tvec.reshape(3)

    def _try_collect_sample(self, camera_to_marker):
        try:
            base_to_gripper_msg = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.gripper_frame,
                rclpy.time.Time(),
            )
        except Exception as exc:
            self.get_logger().warning(f'TF unavailable: {self.base_frame} <- {self.gripper_frame}: {exc}')
            return

        base_to_gripper = transform_from_msg(base_to_gripper_msg)

        if self.calibrated_transform is not None and not self.publish_static_tf_on_complete:
            self.tf_broadcaster.sendTransform(
                transform_to_msg(self.calibrated_transform, self.base_frame, self.camera_frame, self.get_clock().now().to_msg())
            )

        if self.calibrated_transform is None and len(self.base_to_gripper_samples) < self.max_samples:
            if self._should_accept_sample(base_to_gripper):
                self.base_to_gripper_samples.append(base_to_gripper)
                self.camera_to_marker_samples.append(camera_to_marker)
                self.last_sample_time = self.get_clock().now()
                self.last_sample_gripper = base_to_gripper
                self.get_logger().info(
                    f'Collected {len(self.base_to_gripper_samples)}/{self.samples_required} samples'
                )
                accepted = True
            else:
                return False, 'rejected: pose is too close to the previous accepted sample'
        else:
            return False, 'rejected: calibration already complete or max_samples reached'

        if self.calibrated_transform is None and len(self.base_to_gripper_samples) >= self.samples_required:
            self.calibrated_transform = self._solve_hand_eye()
            self._report_result()
            if self.publish_static_tf_on_complete:
                self.static_broadcaster.sendTransform(
                    transform_to_msg(self.calibrated_transform, self.base_frame, self.camera_frame, self.get_clock().now().to_msg())
                )
            if self.save_result and not self.result_saved:
                self._save_result()
                self.result_saved = True
            return accepted, 'accepted; calibration_complete'

        return accepted, 'accepted'

    def _take_sample_cb(self, request, response):
        del request

        if self.camera_matrix is None:
            response.success = False
            response.message = 'rejected: camera_info not received'
            return response

        if self.latest_camera_to_marker is None or self.latest_marker_time is None:
            response.success = False
            response.message = 'rejected: marker not visible'
            return response

        age = (self.get_clock().now() - self.latest_marker_time).nanoseconds * 1e-9
        if age > self.max_marker_age_sec:
            response.success = False
            response.message = f'rejected: latest marker is stale ({age:.2f}s)'
            return response

        success, message = self._try_collect_sample(self.latest_camera_to_marker)
        response.success = bool(success)
        response.message = message
        return response

    def _should_accept_sample(self, base_to_gripper):
        if self.last_sample_time is not None:
            elapsed = (self.get_clock().now() - self.last_sample_time).nanoseconds * 1e-9
            if elapsed < self.min_sample_interval_sec:
                return False

        if self.last_sample_gripper is None:
            return True

        relative = invert_transform(self.last_sample_gripper) @ base_to_gripper
        translation_delta = np.linalg.norm(relative[:3, 3])
        rotation_delta = rotation_angle(relative)
        return (
            translation_delta >= self.min_translation_delta_m
            or rotation_delta >= self.min_rotation_delta_rad
        )

    def _solve_hand_eye(self):
        gripper_to_base = [invert_transform(t) for t in self.base_to_gripper_samples]
        marker_to_camera = self.camera_to_marker_samples

        r_gripper_to_base = [t[:3, :3] for t in gripper_to_base]
        t_gripper_to_base = [t[:3, 3] for t in gripper_to_base]
        r_marker_to_camera = [t[:3, :3] for t in marker_to_camera]
        t_marker_to_camera = [t[:3, 3] for t in marker_to_camera]

        rotation, translation = cv2.calibrateHandEye(
            r_gripper_to_base,
            t_gripper_to_base,
            r_marker_to_camera,
            t_marker_to_camera,
            method=cv2.CALIB_HAND_EYE_PARK,
        )
        base_to_camera = make_transform_from_rotation_translation(rotation, translation.reshape(3))
        self.gripper_to_marker = self._estimate_gripper_to_marker(base_to_camera)
        return base_to_camera

    def _estimate_gripper_to_marker(self, base_to_camera):
        estimates = []
        for base_to_gripper, camera_to_marker in zip(self.base_to_gripper_samples, self.camera_to_marker_samples):
            estimates.append(invert_transform(base_to_gripper) @ base_to_camera @ camera_to_marker)
        return average_transforms(estimates)

    def _position_residuals(self):
        residuals = []
        for base_to_gripper, camera_to_marker in zip(self.base_to_gripper_samples, self.camera_to_marker_samples):
            predicted = base_to_gripper @ self.gripper_to_marker
            observed = self.calibrated_transform @ camera_to_marker
            residuals.append(np.linalg.norm(predicted[:3, 3] - observed[:3, 3]))
        return residuals

    def _report_result(self):
        t = self.calibrated_transform[:3, 3]
        q = matrix_to_quaternion(self.calibrated_transform[:3, :3])
        residuals = self._position_residuals()
        self.get_logger().info(
            'Calibration complete: '
            f'{self.base_frame} -> {self.camera_frame} '
            f'translation=[{t[0]:.5f}, {t[1]:.5f}, {t[2]:.5f}] '
            f'quat_xyzw=[{q[0]:.6f}, {q[1]:.6f}, {q[2]:.6f}, {q[3]:.6f}] '
            f'mean_position_residual={np.mean(residuals):.5f} m'
        )

    def _save_result(self):
        t = self.calibrated_transform[:3, 3]
        q = matrix_to_quaternion(self.calibrated_transform[:3, :3])
        gm_t = self.gripper_to_marker[:3, 3]
        gm_q = matrix_to_quaternion(self.gripper_to_marker[:3, :3])
        residuals = self._position_residuals()
        content = (
            'camera_extrinsic:\n'
            '  ros__parameters:\n'
            f'    base_frame: "{self.base_frame}"\n'
            f'    camera_frame: "{self.camera_frame}"\n'
            f'    translation_xyz: [{t[0]:.8f}, {t[1]:.8f}, {t[2]:.8f}]\n'
            f'    rotation_quat_xyzw: [{q[0]:.8f}, {q[1]:.8f}, {q[2]:.8f}, {q[3]:.8f}]\n'
            '\n'
            'hand_eye_result:\n'
            '  ros__parameters:\n'
            f'    gripper_frame: "{self.gripper_frame}"\n'
            f'    marker_frame: "{self.marker_frame}"\n'
            f'    gripper_to_marker_xyz: [{gm_t[0]:.8f}, {gm_t[1]:.8f}, {gm_t[2]:.8f}]\n'
            f'    gripper_to_marker_quat_xyzw: [{gm_q[0]:.8f}, {gm_q[1]:.8f}, {gm_q[2]:.8f}, {gm_q[3]:.8f}]\n'
            f'    marker_id: {self.marker_id}\n'
            f'    marker_size_m: {self.marker_size_m:.8f}\n'
            f'    samples: {len(self.base_to_gripper_samples)}\n'
            f'    mean_position_residual_m: {float(np.mean(residuals)):.8f}\n'
            f'    max_position_residual_m: {float(np.max(residuals)):.8f}\n'
        )
        os.makedirs(os.path.dirname(self.result_path), exist_ok=True)
        with open(self.result_path, 'w', encoding='utf-8') as f:
            f.write(content)
        self.get_logger().info(f'Saved calibration result: {self.result_path}')


def main():
    rclpy.init()
    node = None
    try:
        node = ArucoCameraCalibrator()
        rclpy.spin(node)
    except Exception as exc:
        if node is not None:
            node.get_logger().error(f'aruco_camera_calibrator crashed: {exc}\n{traceback.format_exc()}')
        else:
            print(f'aruco_camera_calibrator failed before logger was ready: {exc}', flush=True)
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
