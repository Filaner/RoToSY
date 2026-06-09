"""
XBC-DR10E PLC bridge node (Modbus TCP).

XBC-DR10E I/O 구성:
  DI 6점: P00000~P00005 → Modbus 비트 주소 0~5
  DO 4점: P00040~P00043 → Modbus 비트 주소 64~67
  (P00040 = word4 * 16 + bit0 = 64)

Topics:
  /plc/status  (robot_arm_interfaces/msg/PlcStatus) 10 Hz

Services:
  /plc/set_output  (robot_arm_interfaces/srv/PlcSetOutput)
"""

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool

from pymodbus.client import ModbusTcpClient
from pymodbus.exceptions import ModbusException

from robot_arm_interfaces.msg import PlcStatus
from robot_arm_interfaces.srv import PlcSetOutput


class PlcBridgeNode(Node):

    def __init__(self) -> None:
        super().__init__('plc_bridge')

        # ── 파라미터 ─────────────────────────────────────────────────────────
        self.declare_parameter('plc_ip',            '192.168.1.10')
        self.declare_parameter('plc_port',          502)
        self.declare_parameter('di_start_addr',     0)    # DI 시작 Modbus 주소
        self.declare_parameter('di_count',          6)    # DI 개수
        self.declare_parameter('do_start_addr',     64)   # DO 시작 Modbus 주소 (P00040=64)
        self.declare_parameter('do_count',          4)    # DO 개수
        self.declare_parameter('poll_rate_hz',      10.0)
        self.declare_parameter('reconnect_sec',     3.0)

        self._plc_ip         = self.get_parameter('plc_ip').value
        self._plc_port       = self.get_parameter('plc_port').value
        self._di_start       = self.get_parameter('di_start_addr').value
        self._di_count       = self.get_parameter('di_count').value
        self._do_start       = self.get_parameter('do_start_addr').value
        self._do_count       = self.get_parameter('do_count').value
        self._poll_interval  = 1.0 / self.get_parameter('poll_rate_hz').value
        self._reconnect_sec  = self.get_parameter('reconnect_sec').value

        # ── 상태 캐시 ────────────────────────────────────────────────────────
        self._lock      = threading.Lock()
        self._connected = False
        self._di        = [False] * self._di_count
        self._do        = [False] * self._do_count
        self._client: ModbusTcpClient | None = None

        # ── Publisher ────────────────────────────────────────────────────────
        self._pub_status = self.create_publisher(PlcStatus, '/plc/status', 10)

        # ── Service ──────────────────────────────────────────────────────────
        self._srv_cbg = MutuallyExclusiveCallbackGroup()
        self._srv_set_output = self.create_service(
            PlcSetOutput, '/plc/set_output',
            self._set_output_cb,
            callback_group=self._srv_cbg,
        )

        # ── 폴링 스레드 시작 ─────────────────────────────────────────────────
        self._stop_flag = threading.Event()
        self._poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._poll_thread.start()

        self.get_logger().info(
            f'PlcBridgeNode 시작 — {self._plc_ip}:{self._plc_port}  '
            f'DI[{self._di_start}+{self._di_count}]  '
            f'DO[{self._do_start}+{self._do_count}]'
        )

    # ══════════════════════════════════════════════════════════════════════════
    # Modbus 폴링 루프 (백그라운드 스레드)
    # ══════════════════════════════════════════════════════════════════════════

    def _connect(self) -> bool:
        """XBC에 Modbus TCP 연결 시도. 성공 시 True 반환."""
        try:
            client = ModbusTcpClient(host=self._plc_ip, port=self._plc_port)
            if client.connect():
                with self._lock:
                    self._client    = client
                    self._connected = True
                self.get_logger().info(f'PLC 연결 성공: {self._plc_ip}:{self._plc_port}')
                return True
            client.close()
        except Exception as e:
            self.get_logger().warn(f'PLC 연결 실패: {e}')
        return False

    def _disconnect(self) -> None:
        with self._lock:
            if self._client:
                try:
                    self._client.close()
                except Exception:
                    pass
            self._client    = None
            self._connected = False

    def _poll_loop(self) -> None:
        """DI 읽기 → 상태 발행 반복 루프."""
        while not self._stop_flag.is_set() and rclpy.ok():
            if not self._connected:
                if not self._connect():
                    time.sleep(self._reconnect_sec)
                    continue

            try:
                self._read_di()
                self._publish_status()
            except (ModbusException, ConnectionError) as e:
                self.get_logger().warn(f'Modbus 통신 오류: {e} — 재연결 대기')
                self._disconnect()
                time.sleep(self._reconnect_sec)
                continue
            except Exception as e:
                self.get_logger().error(f'예상치 못한 오류: {e}')

            time.sleep(self._poll_interval)

    def _read_di(self) -> None:
        """DI 값 읽기 (Modbus FC02: Read Discrete Inputs)."""
        with self._lock:
            client = self._client
        if client is None:
            return

        resp = client.read_discrete_inputs(
            address=self._di_start, count=self._di_count
        )
        if resp.isError():
            raise ModbusException(f'read_discrete_inputs 오류: {resp}')

        with self._lock:
            self._di = list(resp.bits[:self._di_count])

    # ══════════════════════════════════════════════════════════════════════════
    # Publisher
    # ══════════════════════════════════════════════════════════════════════════

    def _publish_status(self) -> None:
        with self._lock:
            di      = list(self._di)
            do      = list(self._do)
            connected = self._connected

        msg = PlcStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.connected    = connected

        # bool 배열 크기 맞추기 (부족하면 False로 패딩)
        msg.di       = di
        msg.do_state = do

        msg.di_mask = sum(1 << i for i, v in enumerate(di) if v)
        msg.do_mask = sum(1 << i for i, v in enumerate(do) if v)

        self._pub_status.publish(msg)

    # ══════════════════════════════════════════════════════════════════════════
    # Service: /plc/set_output
    # ══════════════════════════════════════════════════════════════════════════

    def _set_output_cb(self, request: PlcSetOutput.Request,
                       response: PlcSetOutput.Response) -> PlcSetOutput.Response:
        idx = request.index
        if idx >= self._do_count:
            response.success = False
            response.message = f'DO 인덱스 범위 초과: {idx} (최대 {self._do_count - 1})'
            response.do_mask = self._current_do_mask()
            return response

        if not self._connected:
            response.success = False
            response.message = 'PLC 미연결'
            response.do_mask = self._current_do_mask()
            return response

        addr = self._do_start + idx
        with self._lock:
            client = self._client

        try:
            resp = client.write_coil(address=addr, value=request.value)
            if resp.isError():
                raise ModbusException(f'write_coil 오류: {resp}')

            with self._lock:
                self._do[idx] = request.value

            state_str = 'ON' if request.value else 'OFF'
            self.get_logger().info(f'DO[{idx}] (addr={addr}) → {state_str}')
            response.success = True
            response.message = f'DO[{idx}] {state_str}'
        except (ModbusException, ConnectionError) as e:
            self.get_logger().error(f'DO 쓰기 실패: {e}')
            self._disconnect()
            response.success = False
            response.message = str(e)

        response.do_mask = self._current_do_mask()
        return response

    def _current_do_mask(self) -> int:
        with self._lock:
            return sum(1 << i for i, v in enumerate(self._do) if v)

    def destroy_node(self) -> None:
        self._stop_flag.set()
        self._disconnect()
        super().destroy_node()


# ──────────────────────────────────────────────────────────────────────────────
# Entry point
# ──────────────────────────────────────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlcBridgeNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
