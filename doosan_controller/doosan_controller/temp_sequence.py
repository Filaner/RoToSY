#!/usr/bin/env python3
"""Temp sequence wrapper for the latest drawer motion sequence.

This node reuses the drawer calibration and motion logic from
``motion_sequence`` but publishes to /temp_motion/* and uses an independent
process lock so it can run alongside other nodes without duplicate execution.
"""

import sys
import threading

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from std_msgs.msg import String

from doosan_controller.motion_sequence import (
    MotionSequenceNode,
    _acquire_instance_lock,
)

TEMP_INSTANCE_LOCK_PATH = '/tmp/rotosy_temp_sequence.lock'
TEMP_FALLBACK_MEDICINE_OFFSET_MM = (0.0, 120.0, 0.0)


class TempSequenceNode(MotionSequenceNode):
    def __init__(self):
        super().__init__(topic_prefix='temp_motion', node_name='temp_sequence_node')

    def _on_missing_medicine(self, contact, pull_dir):
        dx, dy, dz = TEMP_FALLBACK_MEDICINE_OFFSET_MM
        target = (
            contact[0] + pull_dir[0] * dx,
            contact[1] + pull_dir[1] * dy,
            contact[2] + dz,
        )
        self.get_logger().warning(
            '약품 감지 실패 — temp_sequence 테스트용 고정 좌표로 대체합니다.'
        )
        self._step_info_pub.publish(
            String(
                data=(
                    'Vision fallback '
                    f'(X:{target[0]:.1f}, Y:{target[1]:.1f}, Z:{target[2]:.1f})'
                )
            )
        )
        return target


def main(args=None):
    if not _acquire_instance_lock(TEMP_INSTANCE_LOCK_PATH):
        print(
            'temp_sequence is already running; refusing duplicate instance.',
            file=sys.stderr,
        )
        return 1

    rclpy.init(args=args)
    node = TempSequenceNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # CLI 지원
    if len(sys.argv) > 1 and sys.argv[1].isdigit():
        drawer_index = int(sys.argv[1])
        radius = float(sys.argv[2]) if len(sys.argv) > 2 else 200.0
        node._step_mode = '--step' in sys.argv
        seq_thread = threading.Thread(target=node.run_sequence, args=(drawer_index, radius))
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
