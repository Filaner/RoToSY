"""Medicine catalog endpoint — nurse 약품 선택지 / OCR 스캔 시뮬 용."""

from fastapi import APIRouter

from ..db_schema import get_conn

router = APIRouter(prefix='/api/medicine', tags=['medicine'])


@router.get('')
async def list_medicines():
    with get_conn() as c:
        rows = c.execute(
            'SELECT id, name, display_name FROM medicine ORDER BY id'
        ).fetchall()
    return [
        {'id':           r['id'],
         'name':         r['name'] or '',
         'display_name': r['display_name'] or r['name'] or ''}
        for r in rows
    ]
