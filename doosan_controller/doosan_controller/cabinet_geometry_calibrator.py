#!/usr/bin/env python3
"""Collect cabinet contact measurements for marker geometry calibration."""

import json
import time
import urllib.request
from pathlib import Path

import numpy as np
import rclpy
import yaml
from rclpy.node import Node

from robot_arm_interfaces.msg import RobotStatus

from doosan_controller.motion_sequence import (
    CAMERA_API,
    _cabinet_geometry_candidates,
    _load_cabinet_geometry,
)


class CabinetGeometryCalibrator(Node):
    def __init__(self):
        super().__init__('cabinet_geometry_calibrator')
        self._tcp = None
        self.create_subscription(RobotStatus, '/arm/status', self._status_cb, 10)
        self.geometry, loaded_path = _load_cabinet_geometry()
        self.path = loaded_path or _cabinet_geometry_candidates()[-1]

    def _status_cb(self, msg: RobotStatus):
        self._tcp = np.asarray(msg.current_tcp[:3], dtype=float)

    def wait_tcp(self) -> np.ndarray:
        deadline = time.time() + 5.0
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._tcp is not None and np.any(self._tcp):
                return self._tcp.copy()
        raise RuntimeError('TCP pose not received')

    def get_marker(self, count: int = 20) -> dict:
        marker_id = int(self.geometry['marker_id'])
        readings = []
        for _ in range(count):
            with urllib.request.urlopen(CAMERA_API, timeout=2.0) as response:
                markers = json.loads(response.read()).get('markers', [])
            marker = next((item for item in markers if item.get('id') == marker_id), None)
            if marker and marker.get('rotation_matrix_base') is not None:
                readings.append(marker)
            time.sleep(0.08)
        if len(readings) < count // 2:
            raise RuntimeError(f'ArUco marker ID {marker_id} not detected reliably')

        positions = np.asarray(
            [[m['x_mm'], m['y_mm'], m['z_mm']] for m in readings], dtype=float
        )
        result = dict(readings[-1])
        result['x_mm'], result['y_mm'], result['z_mm'] = np.mean(positions, axis=0).tolist()
        return result

    def drawer_grid_delta(self, drawer_number: int) -> np.ndarray:
        slot = drawer_number - 1
        reference_slot = int(self.geometry['reference_drawer']) - 1
        row, col = divmod(slot, 2)
        ref_row, ref_col = divmod(reference_slot, 2)
        return np.array([
            (ref_col - col) * float(self.geometry['column_pitch_mm']),
            0.0,
            (ref_row - row) * float(self.geometry['row_pitch_mm']),
        ])

    def capture(self, drawer_number: int) -> dict:
        tcp = self.wait_tcp()
        marker = self.get_marker()
        marker_pos = np.array([marker['x_mm'], marker['y_mm'], marker['z_mm']], dtype=float)
        pull_dir = np.asarray(self.geometry['pull_direction_base'], dtype=float)
        handle_base = tcp - pull_dir * float(self.geometry['gripper_length_mm'])
        drawer_offset = handle_base - marker_pos
        reference_offset = drawer_offset - self.drawer_grid_delta(drawer_number)

        return {
            'drawer': drawer_number,
            'tcp_contact_mm': np.round(tcp, 3).tolist(),
            'marker_base_mm': np.round(marker_pos, 3).tolist(),
            'marker_camera_m': [marker.get('x_cam_m'), marker.get('y_cam_m'), marker.get('z_cam_m')],
            'marker_rvec': marker.get('rvec'),
            'reference_offset_mm': np.round(reference_offset, 3).tolist(),
        }

    def save(self, sample: dict):
        raw = {}
        if self.path.exists():
            raw = yaml.safe_load(self.path.read_text(encoding='utf-8')) or {}
        config = raw.setdefault('cabinet_geometry', {})
        samples = config.setdefault('samples', [])
        samples.append(sample)

        offsets = np.asarray([item['reference_offset_mm'] for item in samples], dtype=float)
        mean = np.mean(offsets, axis=0)
        stddev = np.std(offsets, axis=0)
        config['marker_to_reference_handle_mm'] = np.round(mean, 3).tolist()
        config['sample_mean_mm'] = np.round(mean, 3).tolist()
        config['sample_stddev_mm'] = np.round(stddev, 3).tolist()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.path.write_text(yaml.safe_dump(raw, sort_keys=False), encoding='utf-8')
        print(f'saved: {self.path}')
        print(f'mean offset: {mean.round(2).tolist()} mm')
        print(f'stddev: {stddev.round(2).tolist()} mm')


def main(args=None):
    rclpy.init(args=args)
    node = CabinetGeometryCalibrator()
    try:
        print('Move TCP to the actual drawer handle contact pose.')
        print('Enter drawer number 1..6, or q to finish.')
        while rclpy.ok():
            value = input('drawer> ').strip().lower()
            if value in ('q', 'quit', 'exit'):
                break
            drawer_number = int(value)
            if drawer_number < 1 or drawer_number > 6:
                print('drawer number must be 1..6')
                continue
            sample = node.capture(drawer_number)
            node.save(sample)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
