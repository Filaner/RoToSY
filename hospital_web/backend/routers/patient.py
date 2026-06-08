"""Patient lookup endpoints — nurse 발행 폼 / patient 페이지 동적화 용."""

from fastapi import APIRouter, HTTPException

from ..db_schema import get_conn
from .. import prescription_state as ps

router = APIRouter(prefix='/api/patient', tags=['patient'])


@router.get('')
async def list_patients():
    with get_conn() as c:
        rows = c.execute(
            '''SELECT pa.chart_no, pa.name, pa.bed_no, w.name AS ward
               FROM patient pa
               LEFT JOIN ward w ON w.id = pa.ward_id
               ORDER BY pa.id'''
        ).fetchall()
    return [
        {'chart_no': r['chart_no'] or '',
         'name':     r['name'] or '',
         'bed_no':   r['bed_no'] or '',
         'ward':     r['ward'] or ''}
        for r in rows
    ]


@router.get('/{chart_no}')
async def get_patient(chart_no: str):
    with get_conn() as c:
        row = c.execute(
            '''SELECT pa.chart_no, pa.name, pa.bed_no, w.name AS ward
               FROM patient pa
               LEFT JOIN ward w ON w.id = pa.ward_id
               WHERE pa.chart_no = ?''',
            (chart_no,)
        ).fetchone()
    if not row:
        raise HTTPException(status_code=404, detail='환자를 찾을 수 없습니다.')
    return {
        'chart_no': row['chart_no'] or '',
        'name':     row['name'] or '',
        'bed_no':   row['bed_no'] or '',
        'ward':     row['ward'] or '',
    }


@router.get('/{chart_no}/prescriptions')
async def list_patient_prescriptions(chart_no: str):
    return ps.list_by_patient(chart_no)
