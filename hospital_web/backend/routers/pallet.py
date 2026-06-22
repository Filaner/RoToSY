"""팔레타이징 적재 API — 처방 품목을 병동 박스에 배치하는 레이아웃 계획/조회/진행."""

from fastapi import APIRouter
from pydantic import BaseModel
from typing import Optional

from .. import pallet_pack as pp

router = APIRouter(prefix='/api/pallet', tags=['pallet'])


class PlanReq(BaseModel):
    mission_code: Optional[str] = None   # 미지정 시 현재 미션
    box_code:     Optional[str] = None   # 미지정 시 병동으로 자동 해결


class PlacedReq(BaseModel):
    mission_code: Optional[str] = None
    slot_idx:     int


class PreviewItem(BaseModel):
    name:  str
    qty:   int   = 1
    w_cm:  Optional[float] = None
    d_cm:  Optional[float] = None
    h_cm:  Optional[float] = None


class PreviewReq(BaseModel):
    box_code: str = 'BOX-B'
    items:    list[PreviewItem]


@router.post('/plan')
async def plan(req: PlanReq):
    """처방 품목 + 박스 치수로 적재 레이아웃 계산 후 저장.

    반환: { ok, box_code, placed_count, unplaced, skipped, layout }
    """
    return pp.plan_for_mission(req.mission_code, req.box_code)


@router.get('/plan')
async def get_plan(mission_code: Optional[str] = None):
    """저장된 적재 레이아웃 + 박스 메타 조회 (로봇 노드용)."""
    return pp.get_plan(mission_code)


@router.post('/placed')
async def placed(req: PlacedReq):
    """슬롯 배치 완료 표시."""
    return pp.mark_placed(req.mission_code, req.slot_idx)


@router.get('/catalog')
async def catalog():
    """치수가 등록된 약품 목록 반환 (테스트 UI 드롭다운용)."""
    return {'items': pp.get_catalog()}


@router.post('/preview')
async def preview(req: PreviewReq):
    """DB 미션 없이 임의 품목으로 배치 레이아웃 계산 (단독 테스트용).

    반환: { ok, box_code, box, placed_count, unplaced, skipped, layout }
    """
    items_raw = [{'name': it.name, 'qty': it.qty,
                  'w_cm': it.w_cm, 'd_cm': it.d_cm, 'h_cm': it.h_cm}
                 for it in req.items]
    return pp.preview_plan(items_raw, req.box_code)
