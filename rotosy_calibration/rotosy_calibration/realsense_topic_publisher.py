#!/usr/bin/env python3
"""Publish one RealSense camera as ROS2 topics.

Run one instance of this node per physical camera. Other processes should
subscribe to the topics instead of opening the RealSense device directly.
"""

from __future__ import annotations

import time
from typing import Optional

import numpy as np
import pyrealsense2 as rs
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32


class RealSenseTopicPublisher(Node):
    def __init__(self) -> None:
        super().__init__('realsense_topic_publisher')

        self.declare_parameter('camera_name', 'rotosy_camera')
        self.declare_parameter('base_topic', '/rotosy/camera')
        self.declare_parameter('serial_no', '')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('align_depth_to_color', True)
        self.declare_parameter('retry_delay_sec', 3.0)

        self.camera_name = str(self.get_parameter('camera_name').value)
        self.base_topic = str(self.get_parameter('base_topic').value).rstrip('/')
        self.serial_no = str(self.get_parameter('serial_no').value)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fps = int(self.get_parameter('fps').value)
        self.align_depth_to_color = bool(self.get_parameter('align_depth_to_color').value)
        self.retry_delay_sec = float(self.get_parameter('retry_delay_sec').value)

        self.color_frame_id = f'{self.camera_name}_color_optical_frame'
        self.depth_frame_id = (
            self.color_frame_id
            if self.align_depth_to_color
            else f'{self.camera_name}_depth_optical_frame'
        )

        self.color_pub = self.create_publisher(
            Image,
            f'{self.base_topic}/color/image_raw',
            qos_profile_sensor_data,
        )
        self.depth_pub = self.create_publisher(
            Image,
            f'{self.base_topic}/aligned_depth_to_color/image_raw'
            if self.align_depth_to_color
            else f'{self.base_topic}/depth/image_raw',
            qos_profile_sensor_data,
        )
        self.color_info_pub = self.create_publisher(
            CameraInfo,
            f'{self.base_topic}/color/camera_info',
            qos_profile_sensor_data,
        )
        self.depth_info_pub = self.create_publisher(
            CameraInfo,
            f'{self.base_topic}/aligned_depth_to_color/camera_info'
            if self.align_depth_to_color
            else f'{self.base_topic}/depth/camera_info',
            qos_profile_sensor_data,
        )
        self.depth_scale_pub = self.create_publisher(
            Float32,
            f'{self.base_topic}/depth_scale',
            10,
        )

        self.pipeline: Optional[rs.pipeline] = None
        self.align: Optional[rs.align] = None
        self.color_info: Optional[CameraInfo] = None
        self.depth_info: Optional[CameraInfo] = None
        self.depth_scale = 0.001

        self._last_connect_attempt = 0.0
        self.timer = self.create_timer(1.0 / max(1, self.fps), self._publish_frame)
        self.get_logger().info(
            f'Waiting for RealSense. Publishing topics under {self.base_topic} when connected.'
        )

    def destroy_node(self) -> bool:
        self._stop_pipeline()
        return super().destroy_node()

    def _connect(self) -> None:
        now = time.monotonic()
        if now - self._last_connect_attempt < self.retry_delay_sec:
            return
        self._last_connect_attempt = now

        try:
            devices = list(rs.context().query_devices())
        except Exception as exc:
            self.get_logger().warning(
                f'RealSense device query failed: {exc}; retry in {self.retry_delay_sec:.1f}s'
            )
            return

        if not devices:
            self.get_logger().warning(
                f'No RealSense device detected; retry in {self.retry_delay_sec:.1f}s'
            )
            return

        if self.serial_no:
            serials = []
            for dev in devices:
                try:
                    serials.append(dev.get_info(rs.camera_info.serial_number))
                except Exception:
                    pass
            if self.serial_no not in serials:
                self.get_logger().warning(
                    f'RealSense serial {self.serial_no} not found. '
                    f'Available serials={serials}; retry in {self.retry_delay_sec:.1f}s'
                )
                return

        pipeline = rs.pipeline()
        config = rs.config()
        if self.serial_no:
            config.enable_device(self.serial_no)
        config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
        config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)

        try:
            profile = pipeline.start(config)
            self.pipeline = pipeline
            self.align = rs.align(rs.stream.color) if self.align_depth_to_color else None
            self._load_camera_info(profile)
            serial_msg = f' serial={self.serial_no}' if self.serial_no else ''
            self.get_logger().info(
                f'RealSense publishing{serial_msg}: '
                f'{self.base_topic}/color/image_raw, '
                f'{self.base_topic}/aligned_depth_to_color/image_raw'
            )
        except Exception as exc:
            self.get_logger().warning(
                f'RealSense connect failed: {exc}; retry in {self.retry_delay_sec:.1f}s'
            )
            try:
                pipeline.stop()
            except Exception:
                pass

    def _stop_pipeline(self) -> None:
        if self.pipeline is None:
            return
        try:
            self.pipeline.stop()
        except Exception:
            pass
        self.pipeline = None

    def _load_camera_info(self, profile) -> None:
        color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
        depth_profile = profile.get_stream(rs.stream.depth).as_video_stream_profile()
        color_intr = color_profile.get_intrinsics()
        depth_intr = color_intr if self.align_depth_to_color else depth_profile.get_intrinsics()

        self.color_info = self._camera_info_msg(color_intr, self.color_frame_id)
        self.depth_info = self._camera_info_msg(depth_intr, self.depth_frame_id)
        self.depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()

    def _camera_info_msg(self, intrinsics, frame_id: str) -> CameraInfo:
        msg = CameraInfo()
        msg.header.frame_id = frame_id
        msg.width = int(intrinsics.width)
        msg.height = int(intrinsics.height)
        msg.distortion_model = 'plumb_bob'
        msg.d = [float(v) for v in intrinsics.coeffs]
        msg.k = [
            float(intrinsics.fx), 0.0, float(intrinsics.ppx),
            0.0, float(intrinsics.fy), float(intrinsics.ppy),
            0.0, 0.0, 1.0,
        ]
        msg.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]
        msg.p = [
            float(intrinsics.fx), 0.0, float(intrinsics.ppx), 0.0,
            0.0, float(intrinsics.fy), float(intrinsics.ppy), 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        return msg

    def _publish_frame(self) -> None:
        if self.pipeline is None:
            self._connect()
            return

        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
            if self.align is not None:
                frames = self.align.process(frames)

            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                return

            now = self.get_clock().now().to_msg()
            color = np.asanyarray(color_frame.get_data())
            depth = np.asanyarray(depth_frame.get_data())

            self.color_pub.publish(self._image_msg(color, 'bgr8', self.color_frame_id, now))
            self.depth_pub.publish(self._image_msg(depth, '16UC1', self.depth_frame_id, now))

            if self.color_info is not None:
                self.color_info.header.stamp = now
                self.color_info_pub.publish(self.color_info)
            if self.depth_info is not None:
                self.depth_info.header.stamp = now
                self.depth_info_pub.publish(self.depth_info)

            self.depth_scale_pub.publish(Float32(data=float(self.depth_scale)))

        except Exception as exc:
            self.get_logger().warning(f'RealSense frame error: {exc}; reconnecting')
            self._stop_pipeline()

    @staticmethod
    def _image_msg(array: np.ndarray, encoding: str, frame_id: str, stamp) -> Image:
        contiguous = np.ascontiguousarray(array)
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height = int(contiguous.shape[0])
        msg.width = int(contiguous.shape[1])
        msg.encoding = encoding
        msg.is_bigendian = 0
        if contiguous.ndim == 2:
            msg.step = int(contiguous.shape[1] * contiguous.dtype.itemsize)
        else:
            msg.step = int(contiguous.shape[1] * contiguous.shape[2] * contiguous.dtype.itemsize)
        msg.data = contiguous.tobytes()
        return msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RealSenseTopicPublisher()
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
