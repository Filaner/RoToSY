import time

import rclpy
from rclpy.node import Node
from pymodbus.client import ModbusSerialClient

from robot_arm_interfaces.msg import PlcCommand
from robot_arm_interfaces.srv import PlcSetOutput

_MAX_RETRIES = 3
_INTER_CMD_DELAY = 0.1  # 패킷 충돌 방지 최소 간격 (s)


class PlcControllerNode(Node):

    def __init__(self):
        super().__init__('plc_controller_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('parity', 'N')
        self.declare_parameter('stopbits', 1)
        self.declare_parameter('bytesize', 8)
        self.declare_parameter('do_start_addr', 64)
        self.declare_parameter('slave_id', 1)

        port     = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        parity   = self.get_parameter('parity').value
        stopbits = self.get_parameter('stopbits').value
        bytesize = self.get_parameter('bytesize').value
        self._do_start_addr = self.get_parameter('do_start_addr').value
        self._slave_id      = self.get_parameter('slave_id').value
        self._port          = port

        self._client = ModbusSerialClient(
            port=port,
            baudrate=baudrate,
            parity=parity,
            stopbits=stopbits,
            bytesize=bytesize,
        )
        self._last_cmd_time = 0.0

        if self._client.connect():
            self.get_logger().info(
                f'[PLC] Modbus RTU 연결 성공: {port} '
                f'[{baudrate}-{bytesize}{parity}{stopbits}]'
            )
        else:
            self.get_logger().error(
                f'[PLC] ★ Modbus RTU 연결 실패: {port} ★ — '
                f'배선/포트 상태를 확인하세요. 5초마다 재연결 시도합니다.'
            )

        self.create_subscription(PlcCommand, '/plc_command', self._cmd_callback, 10)
        self.create_service(PlcSetOutput, '/plc/set_output', self._set_output_cb)

        # 5초마다 미연결 시 자동 재연결 + 상태 출력
        self.create_timer(5.0, self._connection_check)
        self.get_logger().info('[PLC] 노드 초기화 완료 — /plc_command 구독 대기 중')

    # ── 내부 유틸 ─────────────────────────────────────────────────────────────

    def _connection_check(self):
        """5초마다 연결 상태를 확인하고, 끊겼으면 재연결을 시도한다."""
        if self._client.connected:
            self.get_logger().debug(f'[PLC] 연결 정상: {self._port}')
            return
        self.get_logger().warning(f'[PLC] 미연결 ({self._port}) — 재연결 시도...')
        if self._client.connect():
            self.get_logger().info(f'[PLC] 재연결 성공: {self._port}')
        else:
            self.get_logger().warning(
                f'[PLC] 재연결 실패: {self._port} — 포트/배선 상태를 확인하세요'
            )

    def _enforce_delay(self):
        elapsed = time.time() - self._last_cmd_time
        if elapsed < _INTER_CMD_DELAY:
            time.sleep(_INTER_CMD_DELAY - elapsed)
        self._last_cmd_time = time.time()

    def _ensure_connected(self) -> bool:
        if self._client.connected:
            return True
        self.get_logger().warning('연결 끊김 — 재연결 시도')
        return self._client.connect()

    def _dispatch(self, msg: PlcCommand) -> bool:
        cmd = msg.command.upper()
        try:
            if cmd == 'COIL':
                resp = self._client.write_coil(
                    address=msg.address,
                    value=bool(msg.value),
                    slave=msg.slave_id,
                )
            elif cmd == 'REGISTER':
                resp = self._client.write_register(
                    address=msg.address,
                    value=msg.value,
                    slave=msg.slave_id,
                )
            else:
                self.get_logger().error(f'알 수 없는 command: {msg.command}')
                return False
        except Exception as exc:
            self.get_logger().error(f'Modbus 통신 예외: {exc}')
            self._client.close()
            return False

        if resp.isError():
            self.get_logger().error(f'Modbus 에러: {resp}')
            return False

        self.get_logger().info(
            f'[{msg.target}] {msg.command} '
            f'addr=0x{msg.address:02X} val={msg.value} slave={msg.slave_id} OK'
        )
        return True

    # ── 콜백 ─────────────────────────────────────────────────────────────────

    def _cmd_callback(self, msg: PlcCommand):
        self.get_logger().info(
            f'[PLC 수신] target={msg.target} cmd={msg.command} '
            f'addr=0x{msg.address:02X} val={msg.value} slave={msg.slave_id}'
        )
        if not self._ensure_connected():
            self.get_logger().error(
                f'[PLC] ★ 연결 불가({self._port}) — 명령 무시: '
                f'{msg.command} addr=0x{msg.address:02X} ★'
            )
            return

        self._enforce_delay()

        for attempt in range(1, _MAX_RETRIES + 1):
            if self._dispatch(msg):
                return
            self.get_logger().warning(f'재시도 {attempt}/{_MAX_RETRIES}')
            if not self._ensure_connected():
                self.get_logger().error('재연결 실패 — 재시도 중단')
                break
            time.sleep(_INTER_CMD_DELAY)

        self.get_logger().error(
            f'명령 실패 ({_MAX_RETRIES}회): '
            f'{msg.target} {msg.command} addr=0x{msg.address:02X} '
            f'val={msg.value} slave={msg.slave_id}'
        )

    def _set_output_cb(
        self,
        req: PlcSetOutput.Request,
        resp: PlcSetOutput.Response,
    ) -> PlcSetOutput.Response:
        if not self._ensure_connected():
            resp.success = False
            resp.message = '연결 불가'
            return resp

        address = self._do_start_addr + req.index
        self._enforce_delay()

        for attempt in range(1, _MAX_RETRIES + 1):
            try:
                result = self._client.write_coil(
                    address=address,
                    value=req.value,
                    slave=self._slave_id,
                )
            except Exception as exc:
                self.get_logger().error(f'Modbus 통신 예외: {exc}')
                self._client.close()
                resp.success = False
                resp.message = f'통신 예외: {exc}'
                return resp
            if not result.isError():
                self.get_logger().info(
                    f'[PlcSetOutput] DO[{req.index}] '
                    f'addr=0x{address:02X} val={req.value} OK'
                )
                resp.success = True
                resp.message = 'OK'
                resp.do_mask = 1 << req.index
                return resp
            self.get_logger().warning(f'재시도 {attempt}/{_MAX_RETRIES}: {result}')
            time.sleep(_INTER_CMD_DELAY)

        resp.success = False
        resp.message = f'Modbus 에러 ({_MAX_RETRIES}회 실패): {result}'
        return resp


def main(args=None):
    rclpy.init(args=args)
    node = PlcControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._client.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
