"""
Palletizing Sequence Node — 팔레타이징 전용 진입점(얇은 래퍼).

MotionSequenceNode 를 그대로 쓰되 `enable_palletizing` 파라미터만 켜는 전용 노드.
실제 동작(픽업→OCR→14~15 사이 슬롯 계산·정렬·배치)은 MotionSequenceNode 안에 있고,
적재 좌표 알고리즘은 self-contained 모듈 `palletizing_planner` 가 담당한다.
별도 노드로 둬 독립 실행/관리가 쉽도록 유지한다. (motion_sequence 를
`-p enable_palletizing:=true` 로 켜도 완전히 동일하게 동작한다.)

실행:
  ros2 run doosan_controller palletizing_sequence
  ros2 run doosan_controller palletizing_sequence <drawer_index> [--step]
"""

import sys
import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.parameter import Parameter

from .motion_sequence import (
    MotionSequenceNode,
    DEFAULT_RADIUS,
    _acquire_instance_lock,
)


class PalletizingSequenceNode(MotionSequenceNode):
    """`enable_palletizing` 를 켠 MotionSequenceNode (팔레타이징 전용 진입점)."""

    def __init__(self, **kwargs):
        super().__init__(node_name='palletizing_sequence_node', **kwargs)
        # 전용 노드이므로 팔레타이징을 기본으로 켠다.
        self.set_parameters([Parameter('enable_palletizing', Parameter.Type.BOOL, True)])
        self.get_logger().info('PalletizingSequenceNode ready (palletizing ON).')


def main(args=None):
    if not _acquire_instance_lock('/tmp/rotosy_palletizing_sequence.lock'):
        print('palletizing_sequence is already running; refusing duplicate instance.',
              file=sys.stderr)
        return 1

    rclpy.init(args=args)
    node = PalletizingSequenceNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # CLI 지원: ros2 run ... palletizing_sequence <drawer_index> [--step]
    if len(sys.argv) > 1 and sys.argv[1].isdigit():
        drawer_index = int(sys.argv[1])
        radius = float(sys.argv[2]) if len(sys.argv) > 2 and sys.argv[2].replace('.', '', 1).isdigit() else DEFAULT_RADIUS
        node._step_mode = '--step' in sys.argv
        seq_thread = threading.Thread(target=node.run_sequence, args=(drawer_index, radius))
        seq_thread.start()

    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node._stop_requested = True
        node._next_step_event.set()
        thread = node._current_sequence_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=5.0)
        node._plc_safety_off()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
