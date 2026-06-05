"""Prescription management endpoints (pharmacist-facing)."""

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import Optional

from .. import prescription_state as ps
from .. import mission_state as ms

router = APIRouter(prefix='/api/prescription', tags=['prescription'])


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
