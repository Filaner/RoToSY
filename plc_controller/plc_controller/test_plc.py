"""
ros2 run plc_controller test_plc

plc_controller_node가 함께 실행 중이어야 실제 하드웨어로 전달된다.
"""

import threading
import time

import rclpy
from rclpy.node import Node

from robot_arm_interfaces.msg import PlcCommand


class TestPlcNode(Node):

    def __init__(self):
        super().__init__('test_plc_node')
        self._pub = self.create_publisher(PlcCommand, '/plc_command', 10)

    def send(self, target: str, command: str, address: int, value: int, slave_id: int):
        msg = PlcCommand()
        msg.target   = target
        msg.command  = command
        msg.address  = address
        msg.value    = value
        msg.slave_id = slave_id
        self._pub.publish(msg)


def _cli_loop(node: TestPlcNode):
    print()
    print("====================================================")
    print("     [LS PLC & LS M100 인버터] 연동 제어 프로그램")
    print("----------------------------------------------------")
    print(" 1. PLC 단독 제어 (국번 1)")
    print("    단축: [주소] [상태]     (예: 27 1   →  코일 27 ON)")
    print("    전체: P [주소] [상태]   (예: P 21 1  →  M21 ON, 주소는 16진수 해석)")
    print()
    print(" 2. M100 인버터 제어 + PLC M21 자동 연동 (국번 2)")
    print("    주파수 설정:  I F [값]    (예: I F 6000  →  60.00 Hz)")
    print("    컨베이어 가동: I RUN 1   →  인버터 RUN + PLC M21 ON")
    print("    컨베이어 정지: I RUN 0   →  인버터 STOP + PLC M21 OFF")
    print()
    print(" 3. 프로그램 종료: q")
    print("====================================================")
    print("[주의] plc_controller_node가 실행 중이어야 하드웨어로 전달됩니다.")
    print()

    while rclpy.ok():
        try:
            user_input = input("명령 입력: ").strip()
        except (EOFError, KeyboardInterrupt):
            break

        if not user_input:
            continue

        if user_input.lower() == 'q':
            print("프로그램을 종료합니다.")
            rclpy.shutdown()
            break

        parts = user_input.split()
        if len(parts) < 2:
            print("  [오류] 입력 형식이 너무 짧습니다. (예: 27 1  또는  I RUN 1)")
            continue

        target = parts[0].upper()

        # ── 숫자 두 개만 입력: PLC 코일 단축 제어 (예: 27 1) ────────────
        if len(parts) == 2 and parts[0].isdigit():
            addr_str, state_str = parts[0], parts[1]
            try:
                addr = int(addr_str)
            except ValueError:
                print(f"  [오류] 올바른 숫자 형식이 아닙니다: '{addr_str}'")
                continue
            is_on = 1 if state_str == '1' else 0
            node.send('PLC', 'COIL', addr, is_on, 1)
            print(f"  [전송] PLC 코일 {addr}(0x{addr:02X}) → {'ON' if is_on else 'OFF'}")
            continue

        if len(parts) < 3:
            print("  [오류] 입력 형식이 너무 짧습니다. (예: 27 1  또는  I RUN 1)")
            continue

        # ── PLC 단독 제어 ─────────────────────────────────────────────────
        if target == 'P':
            m_str, state_str = parts[1], parts[2]
            hex_part = m_str[1:] if m_str.upper().startswith('M') else m_str
            try:
                addr = int(hex_part, 16)
            except ValueError:
                print(f"  [오류] 올바른 16진수 형식이 아닙니다: '{hex_part}'")
                continue

            is_on = 1 if state_str == '1' else 0
            node.send('PLC', 'COIL', addr, is_on, 1)
            print(f"  [전송] PLC M{hex_part.upper()} → {'ON' if is_on else 'OFF'}")

        # ── 인버터 + PLC 연동 제어 ────────────────────────────────────────
        elif target == 'I':
            cmd_type = parts[1].upper()
            val_str  = parts[2]

            if cmd_type == 'F':
                try:
                    freq = int(val_str)
                except ValueError:
                    print("  [오류] 주파수 값은 숫자로 입력해 주세요.")
                    continue
                if not 0 <= freq <= 6000:
                    print("  [오류] 주파수 범위는 0 ~ 6000 사이여야 합니다.")
                    continue
                node.send('INVERTER', 'REGISTER', 4, freq, 2)
                print(f"  [전송] 인버터 목표 주파수 → {freq / 100:.2f} Hz")

            elif cmd_type == 'RUN':
                if val_str == '1':
                    node.send('INVERTER', 'REGISTER', 5, 2, 2)   # 인버터 RUN
                    time.sleep(0.1)
                    node.send('PLC', 'COIL', 0x21, 1, 1)          # M21 ON
                    print("  [전송] 인버터 RUN + PLC M21 ON")
                elif val_str == '0':
                    node.send('INVERTER', 'REGISTER', 5, 1, 2)   # 인버터 STOP
                    time.sleep(0.1)
                    node.send('PLC', 'COIL', 0x21, 0, 1)          # M21 OFF
                    print("  [전송] 인버터 STOP + PLC M21 OFF")
                else:
                    print("  [오류] RUN 값은 1(가동) 또는 0(정지)만 가능합니다.")

            else:
                print("  [오류] 인버터 세부 명령은 'F' 또는 'RUN'만 가능합니다.")

        else:
            print("  [오류] 명령 시작은 'P' 또는 'I'여야 합니다.")


def main(args=None):
    rclpy.init(args=args)
    node = TestPlcNode()

    cli_thread = threading.Thread(target=_cli_loop, args=(node,), daemon=True)
    cli_thread.start()

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
