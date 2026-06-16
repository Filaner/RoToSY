"""
LS M100 인버터(컨베이어) 제어 엔드포인트.
plc_controller_node가 실행 중이어야 Modbus RTU로 전달됩니다.
"""

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from .. import ros_node as ros

router = APIRouter(prefix='/api/inverter', tags=['inverter'])


class FreqRequest(BaseModel):
    freq: int  # 0.01 Hz 단위 (3000 = 30.00 Hz)


class RunRequest(BaseModel):
    run: bool


@router.post('/freq')
def set_freq(body: FreqRequest) -> dict:
    node = ros.get_node()
    if node is None:
        raise HTTPException(status_code=503, detail='ROS2 노드 미연결')
    if not 0 <= body.freq <= 6000:
        raise HTTPException(status_code=422, detail='주파수 범위: 0~6000 (0.01 Hz 단위, 최대 60.00 Hz)')
    node.publish_inverter_freq(body.freq)
    return {'success': True, 'freq': body.freq, 'hz': round(body.freq / 100, 2)}


@router.post('/run')
def set_run(body: RunRequest) -> dict:
    node = ros.get_node()
    if node is None:
        raise HTTPException(status_code=503, detail='ROS2 노드 미연결')
    node.publish_inverter_run(body.run)
    return {'success': True, 'run': body.run}


@router.get('/status')
def get_status() -> dict:
    node = ros.get_node()
    if node is None:
        return {'inverter_running': False, 'inverter_freq': 0}
    state = node.get_state()
    return {
        'inverter_running': state.get('inverter_running', False),
        'inverter_freq':    state.get('inverter_freq', 0),
    }
