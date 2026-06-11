"""OCR 인증 API — 로봇 픽업 후 약품 라벨 검증."""

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

from .. import ocr_verify as ov

router = APIRouter(prefix='/api/ocr', tags=['ocr'])


class VerifyReq(BaseModel):
    """Groq llama-4-scout가 반환한 OCR 파싱 결과."""
    medicine_name:     str | None = ''
    dosage:            str | None = ''
    instructions:      str | None = ''
    patient_name:      str | None = None
    prescription_date: str | None = None
    ward:              str | None = None
    raw_text:          str | None = ''


@router.post('/verify')
async def verify(req: VerifyReq):
    """
    로봇이 OCR 결과를 전송 → 현재 미션 처방 목록과 매칭 후 저장.

    반환:
      { scan_id, match_status, matched_item, mismatch_reason,
        pending_count, all_matched, pending_items }
    """
    result = ov.verify_and_save(req.model_dump())
    return result


@router.get('/scans/{mission_code}')
async def get_scans(mission_code: str):
    """미션의 전체 OCR 스캔 이력."""
    return {'scans': ov.get_scans(mission_code)}


@router.get('/pending/{mission_code}')
async def get_pending(mission_code: str):
    """아직 인증되지 않은 처방 품목 목록."""
    items = ov.get_pending_items(mission_code)
    return {
        'mission_code':  mission_code,
        'pending_count': len(items),
        'items':         items,
    }
