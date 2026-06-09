"""
DSR E0509 Safety Recovery — ROS2 node + FastAPI web server (port 8001)

동작 흐름:
  1. /arm/status 구독 → SAFE_STOP / SAFE_OFF 전환 감지
  2. 감지 즉시 SetRobotMode(MANUAL=0) 자동 호출 → LED 황색, 직접 교시 가능
  3. 웹 UI: 현재 복구 상태 표시 + "복구 완료" 버튼 (MANUAL 모드 진입 후 활성화)
  4. "복구 완료" 클릭 → /api/recovery/complete → SetRobotMode(AUTONOMOUS=1) 복귀
  5. /safety_recovery/complete ROS2 서비스도 동일 동작 (다른 노드에서 호출 가능)

Note:
  /dsr01/state/robot_state 토픽은 dsr_controller2에서 발행하지 않음.
  arm_controller 가 /dsr01/system/get_robot_state 서비스를 폴링하여
  /arm/status (RobotStatus)로 재발행하므로 해당 토픽을 구독함.
  dsr_controller2가 직접 발행하는 /dsr01/error (RobotError) 는 에러 메시지 수신용.
"""

import asyncio
import threading
import time
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles

from robot_arm_interfaces.msg import RobotStatus
from dsr_msgs2.msg import RobotError
from dsr_msgs2.srv import SetRobotMode

# ── 로봇 상태 상수 ───────────────────────────────────────────────────────────
STATE_INITIALIZING = 0
STATE_STANDBY      = 1
STATE_MOVING       = 2
STATE_SAFE_OFF     = 3
STATE_TEACHING     = 4
STATE_SAFE_STOP    = 5
STATE_NOT_READY    = 15

STATE_STR = {
    STATE_INITIALIZING: 'INITIALIZING',
    STATE_STANDBY:      'STANDBY',
    STATE_MOVING:       'MOVING',
    STATE_SAFE_OFF:     'SAFE_OFF',
    STATE_TEACHING:     'TEACHING',
    STATE_SAFE_STOP:    'SAFE_STOP',
    STATE_NOT_READY:    'NOT_READY',
}

ROBOT_MODE_MANUAL     = 0
ROBOT_MODE_AUTONOMOUS = 1

# ── 복구 상태 ────────────────────────────────────────────────────────────────
RS_IDLE      = 'IDLE'           # 정상 동작
RS_DETECTED  = 'ERROR_DETECTED' # 에러 감지, MANUAL 전환 중
RS_MANUAL    = 'MANUAL_MODE'    # MANUAL 전환 완료, 교시 가능
RS_RESTORING = 'RESTORING'      # AUTONOMOUS 복귀 중
RS_COMPLETE  = 'COMPLETE'       # 복구 완료

# ── 에러 트리거 상태 ─────────────────────────────────────────────────────────
_ERROR_STATES = (STATE_SAFE_STOP, STATE_SAFE_OFF)
_ACTIVE_STATES = (STATE_STANDBY, STATE_MOVING)

STATIC_DIR = Path(__file__).parent / 'static'


# ════════════════════════════════════════════════════════════════════════════
# ROS2 Node
# ════════════════════════════════════════════════════════════════════════════

class SafetyRecoveryNode(Node):

    def __init__(self):
        super().__init__('safety_recovery_node')

        self.declare_parameter('robot_ns', 'dsr01')
        self._ns = self.get_parameter('robot_ns').value

        self._lock = threading.Lock()
        self._state: dict = {
            'robot_state':     -1,
            'robot_state_str': 'UNKNOWN',
            'recovery_state':  RS_IDLE,
            'error_message':   '',
            'servo_on':        False,
            'mode_manual':     False,
        }
        # 활성 상태(STANDBY/MOVING)에서 에러 상태로 전환했는지 추적
        self._prev_active = False

        self._sub_cbg = MutuallyExclusiveCallbackGroup()
        self._srv_cbg = MutuallyExclusiveCallbackGroup()
        self._cli_cbg = MutuallyExclusiveCallbackGroup()

        # arm_controller 가 발행하는 /arm/status 구독
        self.create_subscription(
            RobotStatus, '/arm/status',
            self._status_cb, 10,
            callback_group=self._sub_cbg,
        )
        # DSR 컨트롤러가 직접 발행하는 에러 메시지 구독
        self.create_subscription(
            RobotError, f'/{self._ns}/error',
            self._error_cb, 10,
            callback_group=self._sub_cbg,
        )

        self._cli_set_mode = self.create_client(
            SetRobotMode,
            f'/{self._ns}/system/set_robot_mode',
            callback_group=self._cli_cbg,
        )

        # 다른 ROS2 노드에서 직접 호출 가능한 복구 완료 서비스
        self.create_service(
            Trigger,
            '/safety_recovery/complete',
            self._complete_srv_cb,
            callback_group=self._srv_cbg,
        )

        self.get_logger().info(
            f'SafetyRecoveryNode 시작  ns={self._ns}  web=http://0.0.0.0:8001'
        )

    # ── 상태 콜백 ────────────────────────────────────────────────────────────

    def _status_cb(self, msg: RobotStatus) -> None:
        with self._lock:
            robot_state = msg.robot_state
            self._state['robot_state']     = robot_state
            self._state['robot_state_str'] = msg.robot_state_str
            self._state['servo_on']        = msg.servo_on
            prev_active    = self._prev_active
            recovery_state = self._state['recovery_state']

            is_active = robot_state in _ACTIVE_STATES
            is_error  = robot_state in _ERROR_STATES

            self._prev_active = is_active or (prev_active and not is_error)

        # 활성 상태에서 에러 상태로 새로 전환된 경우에만 트리거
        if is_error and prev_active and recovery_state == RS_IDLE:
            self.get_logger().warn(
                f'[SafetyRecovery] 에러 상태 감지: {msg.robot_state_str} '
                f'→ MANUAL 모드 자동 전환 시작'
            )
            threading.Thread(target=self._enter_manual, daemon=True).start()

    def _error_cb(self, msg: RobotError) -> None:
        with self._lock:
            self._state['error_message'] = (
                f'[level={msg.level} code={msg.code}] '
                f'{msg.msg1} {msg.msg2}'.strip()
            )

    # ── 모드 전환 로직 ────────────────────────────────────────────────────────

    def _call_set_mode(self, mode: int, timeout: float = 5.0) -> bool:
        """SetRobotMode 서비스 동기 호출."""
        if not self._cli_set_mode.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(
                f'/{self._ns}/system/set_robot_mode 서비스 없음'
            )
            return False
        req = SetRobotMode.Request()
        req.robot_mode = mode
        fut = self._cli_set_mode.call_async(req)
        deadline = time.time() + timeout
        while not fut.done() and time.time() < deadline:
            time.sleep(0.05)
        if not fut.done():
            self.get_logger().error('SetRobotMode 타임아웃')
            return False
        resp = fut.result()
        if resp is None or not resp.success:
            self.get_logger().error(f'SetRobotMode({mode}) 거부')
            return False
        return True

    def _enter_manual(self) -> None:
        """에러 감지 시 백그라운드에서 MANUAL 모드 전환."""
        with self._lock:
            self._state['recovery_state'] = RS_DETECTED

        success = self._call_set_mode(ROBOT_MODE_MANUAL)

        with self._lock:
            if success:
                self._state['recovery_state'] = RS_MANUAL
                self._state['mode_manual']    = True
                self.get_logger().info(
                    '[SafetyRecovery] MANUAL 모드 진입 완료 — LED 황색, 직접 교시 가능'
                )
            else:
                # 실패 시 IDLE로 복귀 (다음 감지 가능하도록)
                self._state['recovery_state'] = RS_IDLE
                self.get_logger().error(
                    '[SafetyRecovery] MANUAL 전환 실패 — 로봇 상태를 확인하세요'
                )

    def enter_autonomous(self) -> tuple[bool, str]:
        """복구 완료: AUTONOMOUS 모드 복귀."""
        with self._lock:
            if self._state['recovery_state'] != RS_MANUAL:
                return False, f'복구 완료 불가 — 현재 상태: {self._state["recovery_state"]}'
            self._state['recovery_state'] = RS_RESTORING

        success = self._call_set_mode(ROBOT_MODE_AUTONOMOUS)

        with self._lock:
            if success:
                self._state['recovery_state'] = RS_COMPLETE
                self._state['mode_manual']    = False
                msg = 'AUTONOMOUS 모드 복귀 완료'
                self.get_logger().info(f'[SafetyRecovery] {msg}')
            else:
                self._state['recovery_state'] = RS_MANUAL  # 실패 시 MANUAL 유지
                msg = 'SetRobotMode(AUTONOMOUS) 실패'
                self.get_logger().error(f'[SafetyRecovery] {msg}')
        return success, msg

    def reset_idle(self) -> None:
        """COMPLETE 상태 이후 다음 에러 감지를 위해 IDLE 복귀."""
        with self._lock:
            self._state['recovery_state'] = RS_IDLE
            self._state['error_message']  = ''
            self._state['mode_manual']    = False
            self._prev_active             = False

    # ── ROS2 서비스 서버 콜백 ─────────────────────────────────────────────────

    def _complete_srv_cb(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        success, msg = self.enter_autonomous()
        response.success = success
        response.message = msg
        return response

    # ── 상태 조회 ─────────────────────────────────────────────────────────────

    def get_state(self) -> dict:
        with self._lock:
            return dict(self._state)


# ════════════════════════════════════════════════════════════════════════════
# 전역 노드 홀더
# ════════════════════════════════════════════════════════════════════════════

_node: Optional[SafetyRecoveryNode] = None


def _spin_ros() -> None:
    global _node
    rclpy.init()
    _node = SafetyRecoveryNode()
    executor = MultiThreadedExecutor()
    executor.add_node(_node)
    try:
        executor.spin()
    except Exception:
        pass
    finally:
        _node.destroy_node()
        rclpy.shutdown()


def get_node() -> Optional[SafetyRecoveryNode]:
    return _node


# ════════════════════════════════════════════════════════════════════════════
# FastAPI 앱
# ════════════════════════════════════════════════════════════════════════════

app = FastAPI(title='DSR Safety Recovery', version='0.0.1')

if STATIC_DIR.exists():
    app.mount('/static', StaticFiles(directory=str(STATIC_DIR)), name='static')


class _WsManager:
    def __init__(self):
        self._clients: set[WebSocket] = set()

    async def connect(self, ws: WebSocket) -> None:
        await ws.accept()
        self._clients.add(ws)

    def disconnect(self, ws: WebSocket) -> None:
        self._clients.discard(ws)

    async def broadcast(self, data: dict) -> None:
        dead: set[WebSocket] = set()
        for c in self._clients:
            try:
                await c.send_json(data)
            except Exception:
                dead.add(c)
        self._clients -= dead

    @property
    def count(self) -> int:
        return len(self._clients)


_wm = _WsManager()


@app.get('/', response_class=HTMLResponse)
async def index() -> HTMLResponse:
    return HTMLResponse((STATIC_DIR / 'index.html').read_text())


@app.websocket('/ws')
async def ws_endpoint(websocket: WebSocket) -> None:
    await _wm.connect(websocket)
    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        _wm.disconnect(websocket)


@app.get('/api/state')
async def get_state() -> dict:
    node = get_node()
    return node.get_state() if node else {'recovery_state': 'ROS_NOT_READY'}


@app.post('/api/recovery/complete')
async def recovery_complete() -> dict:
    """'복구 완료' 버튼 → AUTONOMOUS 모드 복귀."""
    node = get_node()
    if node is None:
        return {'success': False, 'message': 'ROS2 노드 초기화 중'}

    loop = asyncio.get_running_loop()
    success, msg = await loop.run_in_executor(None, node.enter_autonomous)
    return {'success': success, 'message': msg}


@app.post('/api/recovery/reset')
async def recovery_reset() -> dict:
    """복구 완료 후 다음 감지를 위해 IDLE 리셋."""
    node = get_node()
    if node:
        node.reset_idle()
    return {'success': True}


async def _broadcast_loop() -> None:
    while True:
        if _wm.count > 0:
            node = get_node()
            if node:
                await _wm.broadcast(node.get_state())
        await asyncio.sleep(0.1)  # 10 Hz


@app.on_event('startup')
async def _on_startup() -> None:
    ros_thread = threading.Thread(target=_spin_ros, daemon=True)
    ros_thread.start()
    asyncio.create_task(_broadcast_loop())


# ════════════════════════════════════════════════════════════════════════════
# Entry point
# ════════════════════════════════════════════════════════════════════════════

def main() -> None:
    uvicorn.run(
        'dsr_safety_recovery.node:app',
        host='0.0.0.0',
        port=8001,
        reload=False,
    )


if __name__ == '__main__':
    main()
