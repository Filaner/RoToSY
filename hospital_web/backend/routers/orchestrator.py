"""Orchestrator control endpoints."""

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

from .. import orchestrator

router = APIRouter(prefix='/api/orchestrator', tags=['orchestrator'])


class StartReq(BaseModel):
    prescription_id: str
    actor: str = 'admin'


class CancelReq(BaseModel):
    actor: str = 'admin'
    detail: str = '오케스트레이터 취소'


@router.get('/status')
async def status():
    return orchestrator.get_state()


@router.post('/start')
async def start(req: StartReq):
    try:
        return await orchestrator.start(req.prescription_id, actor=req.actor)
    except Exception as exc:
        raise HTTPException(status_code=409, detail=str(exc))


@router.post('/cancel')
async def cancel(req: CancelReq | None = None):
    req = req or CancelReq()
    return await orchestrator.cancel(actor=req.actor, detail=req.detail)


@router.post('/retry')
async def retry(actor: str = 'admin'):
    try:
        return await orchestrator.retry(actor=actor)
    except Exception as exc:
        raise HTTPException(status_code=409, detail=str(exc))
