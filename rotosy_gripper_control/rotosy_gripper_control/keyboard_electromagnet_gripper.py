import select
import sys
import termios
import time
import tty
from typing import Optional

import rclpy
from rclpy.node import Node

from dsr_msgs2.srv import SetToolDigitalOutput
from robot_arm_interfaces.msg import PlcCommand


DOOSAN_TOOL_DO_ON = 1   # 실제 동작: 1=ON(자성 발생)
DOOSAN_TOOL_DO_OFF = 0  # 실제 동작: 0=OFF(자성 없음)

# 전자석과 연동되는 PLC M 릴레이 코일 주소 (PLC 모델에 따라 조정)
_M23_COIL_ADDR = 0x23  # M23


class KeyboardElectromagnetGripper:
    """Helper for an electromagnet gripper on Doosan flange DO1.
    Can be used as a standalone Node or as a component within another Node.
    """

    def __init__(self, node: Optional[Node] = None) -> None:
        if node is None:
            self._node = Node('keyboard_electromagnet_gripper')
            self._own_node = True
        else:
            self._node = node
            self._own_node = False

        self._node.declare_parameter('robot_ns', 'dsr01')
        self._node.declare_parameter('tool_do_index', 1)
        self._node.declare_parameter('service_timeout_sec', 5.0)
        self._node.declare_parameter('turn_off_on_exit', True)

        self._robot_ns = self._node.get_parameter('robot_ns').value
        self._tool_do_index = int(self._node.get_parameter('tool_do_index').value)
        self._service_timeout_sec = float(
            self._node.get_parameter('service_timeout_sec').value
        )
        self._turn_off_on_exit = bool(self._node.get_parameter('turn_off_on_exit').value)

        self._client = self._node.create_client(
            SetToolDigitalOutput,
            f'{self._robot_ns}/io/set_tool_digital_output',
        )
        self._plc_pub = self._node.create_publisher(PlcCommand, '/plc_command', 10)
        self._is_on = False

        self._node.get_logger().info(
            f'Using service {self._client.srv_name!r}, flange DO{self._tool_do_index}.'
        )
        self._node.get_logger().info(
            'Before running this on hardware, set DART Flange I/O Supply Voltage to 12V.'
        )

    def set_gripper(self, enabled: bool) -> bool:
        if not self._client.wait_for_service(timeout_sec=self._service_timeout_sec):
            self._node.get_logger().error(f'Service {self._client.srv_name!r} unavailable')
            return False

        request = SetToolDigitalOutput.Request()
        request.index = self._tool_do_index
        request.value = DOOSAN_TOOL_DO_ON if enabled else DOOSAN_TOOL_DO_OFF

        future = self._client.call_async(request)

        # 만약 별도의 노드로 동작 중이면 직접 spin, 아니면 future가 완료될 때까지 대기
        if self._own_node:
            rclpy.spin_until_future_complete(
                self._node,
                future,
                timeout_sec=self._service_timeout_sec,
            )
        else:
            # 부모 노드의 executor가 이 service 응답을 처리해줘야 함
            start_t = time.time()
            while not future.done() and (time.time() - start_t) < self._service_timeout_sec:
                time.sleep(0.05)

        if not future.done():
            self._node.get_logger().error('Timed out while setting tool digital output')
            return False

        response = future.result()
        if response is None or not response.success:
            self._node.get_logger().error(
                f'Failed to set DO{self._tool_do_index} to {"ON" if enabled else "OFF"}'
            )
            return False

        self._is_on = enabled
        self._node.get_logger().info(
            f'Electromagnet gripper {"ON" if enabled else "OFF"} '
            f'(DO{self._tool_do_index}, {"12V output" if enabled else "open output"})'
        )

        msg = PlcCommand()
        msg.target   = 'PLC'
        msg.command  = 'COIL'
        msg.address  = _M23_COIL_ADDR
        msg.value    = int(enabled)
        msg.slave_id = 1
        self._plc_pub.publish(msg)
        self._node.get_logger().info(
            f'[PLC] M23(0x{_M23_COIL_ADDR:02X}) → {"ON" if enabled else "OFF"}'
        )

        return True

    def shutdown(self) -> None:
        if self._turn_off_on_exit and self._is_on:
            self._node.get_logger().info('Turning gripper OFF before exit')
            self.set_gripper(False)

    def destroy_node(self):
        if self._own_node:
            self._node.destroy_node()

    @property
    def logger(self):
        return self._node.get_logger()


class RawTerminal:
    def __init__(self) -> None:
        self._settings: Optional[list] = None

    def __enter__(self):
        if sys.stdin.isatty():
            self._settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        if self._settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._settings)

    def read_key(self, timeout_sec: float = 0.1) -> Optional[str]:
        ready, _, _ = select.select([sys.stdin], [], [], timeout_sec)
        if not ready:
            return None
        return sys.stdin.read(1)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KeyboardElectromagnetGripper()

    if not sys.stdin.isatty():
        node.get_logger().error('Keyboard input requires an interactive terminal')
        node.destroy_node()
        rclpy.shutdown()
        return

    try:
        with RawTerminal() as terminal:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.0)
                key = terminal.read_key()
                if key is None:
                    continue
                key = key.lower()

                if key == 'o':
                    node.set_gripper(True)
                elif key == 'f':
                    node.set_gripper(False)
                elif key in ('q', '\x03'):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
