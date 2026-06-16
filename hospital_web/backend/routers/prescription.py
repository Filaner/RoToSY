"""Prescription management endpoints (pharmacist-facing)."""

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import Optional

from .. import prescription_state as ps
from .. import mission_state as ms
from .. import robot_proxy
from ..db_schema import get_conn

import httpx

router = APIRouter(prefix='/api/prescription', tags=['prescription'])


def _trigger_arm_pick(drawer_index: int) -> dict:
    """로봇팔 피킹 시퀀스를 web_interface(:8000) 게이트웨이를 통해 시작시킨다.

    로봇팔/카메라/모션 시퀀스는 web_interface가 단독 소유하므로, 모든 팔 제어는
    8000 의 /api/motion/start 로 위임한다(drawer_index 0..5). 8000 미기동/에러여도
    미션 생성 자체는 막지 않도록 best-effort.
    """
    try:
        with httpx.Client(base_url=robot_proxy.ROTOSY_BASE,
                          timeout=httpx.Timeout(5.0)) as cli:
            r = cli.post('/api/motion/start', json={'marker_id': int(drawer_index)})
            return {'success': r.status_code < 400,
                    'status': r.status_code, 'detail': r.text[:200]}
    except Exception as exc:
        return {'success': False, 'detail': f'web_interface(:8000) 연결 불가: {exc}'}


def _build_marker_queue(pid: str) -> tuple[list, list, list]:
    """처방의 각 약품을 cabinet_slot.medicine_id로 조인해서 마커 큐 생성.

    반환: (markers, labels, missing)
      markers — ArUco ID 정수 리스트
      labels  — 표시용 약품명 리스트
      missing — 슬롯 미매핑된 약품명 (빠짐 경고용)
    """
    markers, labels, missing = [], [], []
    with get_conn() as c:
        pres = c.execute('SELECT id FROM prescription WHERE code=?', (pid,)).fetchone()
        if not pres:
            return [], [], []
        items = c.execute(
            '''SELECT pi.medicine_name, pi.medicine_id
               FROM prescription_item pi
               WHERE pi.prescription_id = ?
               ORDER BY pi.sort_order, pi.id''',
            (pres['id'],)
        ).fetchall()
        for it in items:
            mid = it['medicine_id']
            name = it['medicine_name']
            if mid is None:
                missing.append(name); continue
            slot = c.execute(
                'SELECT aruco_marker_id FROM cabinet_slot WHERE medicine_id=?',
                (mid,)
            ).fetchone()
            if slot and slot['aruco_marker_id'] is not None:
                markers.append(int(slot['aruco_marker_id']))
                labels.append(name)
            else:
                missing.append(name)
    return markers, labels, missing


class ApproveReq(BaseModel):
    note: str = ''

class RejectReq(BaseModel):
    reason: str

class CreateReq(BaseModel):
    patient_name: str
    patient_id:   str
    ward:         str
    doctor:       str
    priority:     str = 'general'
    drugs:        list = []


@router.get('')
async def list_prescriptions():
    return ps.list_all()


@router.get('/{pid}')
async def get_prescription(pid: str):
    p = ps.get(pid)
    if not p:
        raise HTTPException(status_code=404, detail='처방전을 찾을 수 없습니다.')
    return p


@router.post('')
async def create_prescription(req: CreateReq):
    return ps.create(req.model_dump())


@router.post('/{pid}/approve')
async def approve(pid: str, req: ApproveReq):
    p = ps.approve(pid, req.note)
    if not p:
        raise HTTPException(status_code=404, detail='처방전을 찾을 수 없습니다.')
    ms.add_audit('pharmacist', 'PRESCRIPTION_APPROVED', f'{pid} 승인 — {p["patient_name"]}')
    return p


@router.post('/{pid}/reject')
async def reject(pid: str, req: RejectReq):
    if not req.reason.strip():
        raise HTTPException(status_code=400, detail='반려 사유를 입력하세요.')
    p = ps.reject(pid, req.reason)
    if not p:
        raise HTTPException(status_code=404, detail='처방전을 찾을 수 없습니다.')
    ms.add_audit('pharmacist', 'PRESCRIPTION_REJECTED', f'{pid} 반려 — {req.reason}')
    return p


@router.post('/{pid}/request_delivery')
async def request_delivery(pid: str):
    """간호사의 배송 요청."""
    p = ps.request_delivery(pid)
    if not p:
        raise HTTPException(status_code=400, detail='배송 요청 불가 상태입니다.')
    ms.add_audit('nurse', 'REQUEST_DELIVERY', f'{pid} — {p["patient_name"]} 배송 요청')
    return p


@router.post('/{pid}/confirm_loading')
async def confirm_loading(pid: str):
    """약사의 AMR 적재 확인 — mission_state의 pharmacist 확인과 연동."""
    p = ps.get(pid)
    if not p:
        raise HTTPException(status_code=404, detail='처방전을 찾을 수 없습니다.')
    mission = ms.confirm_loading('pharmacist')
    ms.add_audit('pharmacist', 'CONFIRM_LOADING', f'{pid} — {p["patient_name"]} 적재 확인')
    return {'prescription': p, 'mission': mission}


@router.post('/{pid}/start_picking')
async def start_picking(pid: str):
    """관리자의 '조제 시작' — 배송 요청된 처방에 대해 미션 생성 + 로봇 호출.

    조건:
      - prescription.status == 'approved'
      - prescription.delivery_requested == 1
    """
    p = ps.get(pid)
    if not p:
        raise HTTPException(status_code=404, detail='처방전을 찾을 수 없습니다.')
    if p['status'] != 'approved':
        raise HTTPException(status_code=400, detail='승인된 처방만 조제 시작 가능합니다.')
    if not p['delivery_requested']:
        raise HTTPException(status_code=400, detail='간호사 배송 요청이 먼저 필요합니다.')

    # 1) 처방 약품 → 서랍(drawer index) 큐 빌드. 매핑된 슬롯이 없으면 진행 불가(블록).
    markers, labels, missing = _build_marker_queue(pid)
    if not markers:
        raise HTTPException(
            status_code=400,
            detail=f'조제할 약품의 서랍 매핑이 없습니다 (미매핑: {", ".join(missing) or "전체"}). '
                   '약품-서랍(cabinet_slot) 매핑을 확인하세요.')

    # 2) 로봇팔 피킹 트리거(web_interface :8000). 실패하면 미션을 만들지 않고 즉시 에러(블록).
    arm = _trigger_arm_pick(markers[0])
    if not arm.get('success'):
        raise HTTPException(
            status_code=502,
            detail=f'로봇팔 피킹 시작 실패 — web_interface(:8000) 확인 필요. ({arm.get("detail")})')

    # 3) 트리거 성공 → 미션 생성 + 상태/큐/감사 기록
    mission = ms.new_mission(p['ward'], pid)
    ps.set_status(pid, 'awaiting_load_confirm')
    queue = ms.set_marker_queue(markers, labels)

    detail = f'{pid} 조제 시작 — {p["patient_name"]} ({p["ward"]}) · 마커큐 {markers}'
    if missing:
        detail += f' · 슬롯 미매핑: {", ".join(missing)}'
    ms.add_audit('admin', 'START_PICKING', detail)
    ms.add_audit('robot', 'ARM_PICK_TRIGGER',
                 f'{pid} — drawer index {markers[0]} 피킹 시작 ({arm})')

    return {
        'prescription': ps.get(pid),
        'mission':      mission,
        'marker_queue': queue,
        'missing':      missing,
        'arm_trigger':  arm,
    }
