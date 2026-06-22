"""데모 파이프라인 — 실 하드웨어 없이 전체 흐름을 단계별로 시뮬레이션."""
import math
import random
from datetime import datetime

from fastapi import APIRouter, HTTPException

from .. import mission_state as ms
from .. import prescription_state as ps
from .. import ros_bridge as ros
from .. import sensor_db as sdb
from .. import pallet_pack as pp
from .. import demo_create

router = APIRouter(prefix='/api/demo', tags=['demo'])


@router.post('/full_reset')
async def demo_full_reset():
    """DB 전체 초기화 + 데모 시드 재생성 (테스트 페이지 '전체 초기화' 버튼용)."""
    demo_create.reset()
    ms.add_audit('demo', 'DEMO_FULL_RESET', 'DB 전체 초기화 + 재시딩')
    return {'step': 'full_reset', 'ok': True}


@router.post('/quick_start')
async def demo_quick_start():
    """원클릭 테스트 — DB 전체 초기화·재시딩 → 처방 승인 → 로봇 집기 시작(미션+적재계획 생성)까지 한 번에.

    적재(palletizing) 단독 테스트용 단축 경로. 이후 단계(적재확인/AMR출발/도착/완료)는
    필요할 때 /api/demo/step/* 를 개별 호출한다.
    """
    await demo_full_reset()
    approve_result = await demo_approve()
    robot_result   = await demo_robot_start()

    return {
        'step':     'quick_start',
        'approve':  approve_result,
        'robot':    robot_result,
    }


@router.post('/step/approve')
async def demo_approve():
    """① 대기 중인 처방전 자동 승인 + 배송 요청."""
    all_p = ps.list_all()
    pending = [p for p in all_p if p['status'] == 'pending']
    if not pending:
        raise HTTPException(400, '대기 중인 처방전 없음 — 간호사 페이지에서 처방전을 먼저 발행하세요.')
    p = pending[0]
    ps.approve(p['id'], note='[DEMO] 자동 승인')
    approved = ps.request_delivery(p['id'])
    ms.add_audit('demo', 'DEMO_APPROVE', f"{p['id']} 자동 승인 — {p['patient_name']} ({p['ward']})")
    return {'step': 'approve', 'prescription': approved}


@router.post('/step/robot_start')
async def demo_robot_start():
    """② 로봇 암 집기 시작 — 처방 약품 데이터를 로봇 암에 전달하고 동작 개시."""
    all_p = ps.list_all()
    target = next(
        (p for p in all_p if p['status'] == 'approved' and p.get('delivery_requested')),
        None,
    ) or next((p for p in all_p if p['status'] == 'approved'), None)
    if not target:
        raise HTTPException(400, '승인된 처방전 없음 — 먼저 처방 승인 단계를 실행하세요.')

    # 처방 데이터를 로봇 암에 전달 (실제 환경: RoToSY /api/mission/start 호출)
    drug_payload = [
        {'name': d['name'], 'quantity': d['quantity'], 'drawer': f'D{i+1:02d}'}
        for i, d in enumerate(target['drugs'])
    ]

    # 미션 생성 (상태: AWAITING_CONFIRM)
    mission = ms.new_mission(target['ward'], target['id'])
    # 처방전 상태 → 적재 확인 대기
    ps.set_status(target['id'], 'awaiting_load_confirm')

    ms.add_audit(
        'demo', 'DEMO_ROBOT_START',
        f"로봇 암 집기 시작 — {target['patient_name']} | "
        + ', '.join(f"{d['name'].split()[0]}×{d['quantity']}" for d in target['drugs'])
    )

    # 적재 레이아웃 계산 + DB 저장 (실패해도 미션 자체는 막지 않음) — start_picking과 동일.
    pallet_plan = None
    try:
        pallet_plan = pp.plan_for_mission(mission['mission_id'])
        if pallet_plan.get('ok'):
            ms.add_audit('system', 'PALLET_PLAN',
                         f"{mission['mission_id']} → {pallet_plan['box_code']} "
                         f"{pallet_plan['placed_count']}슬롯 계획 완료")
        else:
            ms.add_audit('system', 'PALLET_PLAN_WARN',
                         f"{mission['mission_id']} plan 실패: {pallet_plan.get('error')}")
    except Exception as exc:
        ms.add_audit('system', 'PALLET_PLAN_ERROR', f'plan_for_mission 예외: {exc}')

    return {
        'step':            'robot_start',
        'mission':         mission,
        'prescription_id': target['id'],
        'drug_payload':    drug_payload,
        'pallet_plan':      pallet_plan,
        'message':         f"로봇 암이 {len(drug_payload)}종 약품 집기를 시작했습니다.",
    }


@router.post('/step/confirm_all')
async def demo_confirm_all():
    """③ 관리자 + 약사 적재 확인 동시 완료."""
    ms.confirm_loading('pharmacist')
    result = ms.confirm_loading('admin')
    ms.add_audit('demo', 'DEMO_CONFIRM_ALL', '관리자·약사 적재 확인 완료 → AMR 출발 가능')
    return {'step': 'confirm_all', 'mission': result}


@router.post('/step/dispatch')
async def demo_dispatch():
    """④ AMR 출발."""
    mission = ms.get_mission()
    if not mission.get('can_dispatch'):
        ms.confirm_loading('pharmacist')
        ms.confirm_loading('admin')
    result = ms.update_status('DISPATCHED', actor='demo', detail='AMR 출발 (데모)')
    dest = mission.get('destination', '3병동')
    with ros._lock:
        ros._state['amr']['status']      = 'DELIVERING'
        ros._state['amr']['destination'] = dest
        ros._state['amr']['battery']     = round(random.uniform(72.0, 95.0), 1)
    ms.add_audit('demo', 'DEMO_DISPATCH', f'AMR 출발 → {dest}')
    return {'step': 'dispatch', 'mission': result}


@router.post('/step/amr_arrive')
async def demo_amr_arrive():
    """⑤ AMR 목적지 도착."""
    result = ms.update_status('ARRIVED', actor='demo', detail='AMR 도착 (데모)')
    with ros._lock:
        ros._state['amr']['status'] = 'ARRIVED'
    ms.add_audit('demo', 'DEMO_ARRIVE', 'AMR 도착 — 간호사 수령 대기')
    return {'step': 'amr_arrive', 'mission': result}


@router.post('/step/complete')
async def demo_complete():
    """⑥ 배송 완료."""
    mission = ms.get_mission()
    pid = mission.get('prescription_id')
    if pid:
        ps.set_status(pid, 'completed')
    result = ms.update_status('COMPLETED', actor='demo', detail='배송 완료 (데모)')
    with ros._lock:
        ros._state['amr']['status']      = 'RETURNING'
        ros._state['amr']['destination'] = ''
    ms.add_audit('demo', 'DEMO_COMPLETE', '배송 완료 — AMR 복귀 중')
    return {'step': 'complete', 'mission': result}


@router.post('/step/sensor')
async def demo_sensor():
    """온습도 센서 값 즉시 갱신."""
    now   = datetime.now()
    h_fac = math.sin(now.hour * math.pi / 12) * 1.5
    temp  = round(20.5 + h_fac + random.gauss(0, 0.4), 1)
    humi  = round(55.0 - h_fac * 1.2 + random.gauss(0, 1.2), 1)
    temp  = max(14.5, min(26.5, temp))
    humi  = max(36.0, min(74.0, humi))
    result = sdb.insert_reading(temp, humi)
    ros.update_arduino_reading(temp, humi)
    ms.add_audit('demo', 'DEMO_SENSOR', f'센서 갱신 — {temp}°C / {humi}%')
    return {'step': 'sensor', **result}


@router.post('/reset')
async def demo_reset():
    """데모 전체 초기화."""
    ms.update_status('IDLE', actor='demo', detail='데모 초기화')
    with ros._lock:
        ros._state['amr']['status']      = 'IDLE'
        ros._state['amr']['destination'] = ''
    ms.add_audit('demo', 'DEMO_RESET', '데모 초기화')
    return {'step': 'reset', 'ok': True}
