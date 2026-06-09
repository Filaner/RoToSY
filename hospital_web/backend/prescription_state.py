"""
Prescription store — DB-backed (was in-memory).

Public 시그니처는 기존 in-memory 모듈과 동일:
  list_all(), get(pid), approve(pid, note), reject(pid, reason),
  set_status(pid, status), create(data), request_delivery(pid)

반환 dict 모양도 기존과 동일 (라우터/UI 변경 없음):
  {
    'id', 'patient_name', 'patient_id', 'ward', 'doctor', 'priority',
    'drugs': [{'name','quantity','frequency'}],
    'ocr':   {'raw','parsed','confidence'},
    'vision': [...],
    'status', 'reject_reason', 'pharmacist_note',
    'delivery_requested', 'delivery_requested_at',
    'created_at', 'updated_at'
  }

상태 enum (기존 그대로):
  pending → approved → awaiting_load_confirm → completed
                     → rejected
"""

import json
import threading
import uuid
from datetime import datetime
from typing import Optional

from .db_schema import get_conn

_lock = threading.Lock()
_PRIORITY_ORDER = {'emergency': 0, 'general': 1, 'scheduled': 2}


# ── helpers ───────────────────────────────────────────────────────────────────

def _now() -> str:
    return datetime.now().isoformat(timespec='seconds')


def _gen_code() -> str:
    return f'P-{uuid.uuid4().hex[:6].upper()}'


def _row_to_dict(c, row) -> dict:
    """prescription row + 관련 join → 기존 dict 포맷으로 재구성."""
    pid_int = row['id']

    items = c.execute(
        '''SELECT medicine_name, quantity, frequency, dosage
           FROM prescription_item
           WHERE prescription_id = ?
           ORDER BY sort_order, id''',
        (pid_int,)
    ).fetchall()
    drugs = [
        {'name': i['medicine_name'],
         'quantity': i['quantity'],
         'frequency': i['frequency']}
        for i in items
    ]

    patient_name = ''
    patient_chart = ''
    patient_bed = ''
    ward_name = ''
    if row['patient_id']:
        p = c.execute(
            '''SELECT pa.name, pa.chart_no, pa.bed_no, w.name AS ward_name
               FROM patient pa
               LEFT JOIN ward w ON w.id = pa.ward_id
               WHERE pa.id = ?''',
            (row['patient_id'],)
        ).fetchone()
        if p:
            patient_name = p['name'] or ''
            patient_chart = p['chart_no'] or ''
            patient_bed = p['bed_no'] or ''
            ward_name = p['ward_name'] or ''

    doctor_name = ''
    if row['doctor_id']:
        d = c.execute('SELECT name FROM staff WHERE id = ?', (row['doctor_id'],)).fetchone()
        if d:
            doctor_name = d['name'] or ''

    try:
        ocr_parsed = json.loads(row['ocr_parsed']) if row['ocr_parsed'] else []
    except (TypeError, json.JSONDecodeError):
        ocr_parsed = []
    try:
        vision = json.loads(row['vision_data']) if row['vision_data'] else []
    except (TypeError, json.JSONDecodeError):
        vision = []

    return {
        'id':              row['code'],
        'patient_name':    patient_name,
        'patient_id':      patient_chart,
        'bed_no':          patient_bed,
        'ward':            ward_name,
        'doctor':          doctor_name,
        'priority':        row['priority'],
        'drugs':           drugs,
        'ocr': {
            'raw':        row['ocr_raw'] or '',
            'parsed':     ocr_parsed,
            'confidence': row['ocr_confidence'] or 0,
        },
        'vision':                vision,
        'status':                row['status'],
        'reject_reason':         row['reject_reason'] or '',
        'pharmacist_note':       row['pharmacist_note'] or '',
        'delivery_requested':    bool(row['delivery_requested']),
        'delivery_requested_at': row['delivery_requested_at'],
        'created_at':            row['created_at'],
        'updated_at':            row['updated_at'],
    }


# ── Read ──────────────────────────────────────────────────────────────────────

def list_all() -> list:
    with _lock, get_conn() as c:
        rows = c.execute('SELECT * FROM prescription').fetchall()
        items = [_row_to_dict(c, r) for r in rows]
    items.sort(key=lambda x: (_PRIORITY_ORDER.get(x['priority'], 9), x['created_at']))
    return items


def get(pid: str) -> Optional[dict]:
    with _lock, get_conn() as c:
        row = c.execute('SELECT * FROM prescription WHERE code = ?', (pid,)).fetchone()
        if not row:
            return None
        return _row_to_dict(c, row)


def list_by_patient(chart_no: str) -> list:
    """차트번호로 한 환자의 처방 전체 (최신순)."""
    with _lock, get_conn() as c:
        rows = c.execute(
            '''SELECT p.* FROM prescription p
               JOIN patient pa ON pa.id = p.patient_id
               WHERE pa.chart_no = ?
               ORDER BY p.created_at DESC''',
            (chart_no,)
        ).fetchall()
        return [_row_to_dict(c, r) for r in rows]


# ── Write ─────────────────────────────────────────────────────────────────────

def approve(pid: str, note: str = '') -> Optional[dict]:
    with _lock, get_conn() as c:
        cur = c.execute(
            '''UPDATE prescription
               SET status='approved', pharmacist_note=?, updated_at=?
               WHERE code=?''',
            (note, _now(), pid)
        )
        if cur.rowcount == 0:
            return None
        row = c.execute('SELECT * FROM prescription WHERE code=?', (pid,)).fetchone()
        return _row_to_dict(c, row)


def reject(pid: str, reason: str) -> Optional[dict]:
    with _lock, get_conn() as c:
        cur = c.execute(
            '''UPDATE prescription
               SET status='rejected', reject_reason=?, updated_at=?
               WHERE code=?''',
            (reason, _now(), pid)
        )
        if cur.rowcount == 0:
            return None
        row = c.execute('SELECT * FROM prescription WHERE code=?', (pid,)).fetchone()
        return _row_to_dict(c, row)


def set_status(pid: str, status: str) -> Optional[dict]:
    with _lock, get_conn() as c:
        cur = c.execute(
            'UPDATE prescription SET status=?, updated_at=? WHERE code=?',
            (status, _now(), pid)
        )
        if cur.rowcount == 0:
            return None
        row = c.execute('SELECT * FROM prescription WHERE code=?', (pid,)).fetchone()
        return _row_to_dict(c, row)


def create(data: dict) -> dict:
    """
    data: {
        'patient_name', 'patient_id'(=chart_no), 'ward', 'doctor', 'priority',
        'drugs': [{'name','quantity','frequency'}]
    }
    누락 필드는 기본값으로 처리. patient/staff/ward는 이름 기준으로 자동 upsert.
    """
    code = _gen_code()
    now = _now()

    with _lock, get_conn() as c:
        ward_id    = _ensure_ward(c,    data.get('ward', ''))
        patient_id = _ensure_patient(c, data.get('patient_name', ''),
                                        data.get('patient_id', ''), ward_id)
        doctor_id  = _ensure_staff(c,   data.get('doctor', ''), 'DOCTOR')

        c.execute(
            '''INSERT INTO prescription
               (code, patient_id, doctor_id, priority,
                ocr_raw, ocr_confidence, ocr_parsed, vision_data,
                created_at, updated_at)
               VALUES (?, ?, ?, ?, '', 0, '[]', '[]', ?, ?)''',
            (code, patient_id, doctor_id, data.get('priority', 'general'),
             now, now)
        )
        prescription_id = c.execute(
            'SELECT id FROM prescription WHERE code=?', (code,)
        ).fetchone()['id']

        for i, drug in enumerate(data.get('drugs', [])):
            med_name = drug.get('name', '')
            medicine_id = _ensure_medicine(c, med_name) if med_name else None
            c.execute(
                '''INSERT INTO prescription_item
                   (prescription_id, medicine_id, medicine_name, quantity,
                    frequency, dosage, sort_order)
                   VALUES (?, ?, ?, ?, ?, ?, ?)''',
                (prescription_id, medicine_id, med_name,
                 drug.get('quantity', 1),
                 drug.get('frequency', ''),
                 drug.get('dosage', ''),
                 i)
            )

        row = c.execute('SELECT * FROM prescription WHERE id=?',
                        (prescription_id,)).fetchone()
        return _row_to_dict(c, row)


def request_delivery(pid: str) -> Optional[dict]:
    """approved / awaiting_load_confirm 상태에서만 배송 요청 가능."""
    now = _now()
    with _lock, get_conn() as c:
        row = c.execute('SELECT * FROM prescription WHERE code=?', (pid,)).fetchone()
        if not row:
            return None
        if row['status'] not in ('approved', 'awaiting_load_confirm'):
            return None
        c.execute(
            '''UPDATE prescription
               SET delivery_requested=1, delivery_requested_at=?, updated_at=?
               WHERE code=?''',
            (now, now, pid)
        )
        row = c.execute('SELECT * FROM prescription WHERE code=?', (pid,)).fetchone()
        return _row_to_dict(c, row)


# ── lookup helpers (auto-upsert for create()) ────────────────────────────────

def _ensure_ward(c, name: str) -> Optional[int]:
    if not name:
        return None
    row = c.execute('SELECT id FROM ward WHERE name=?', (name,)).fetchone()
    if row:
        return row['id']
    cur = c.execute('INSERT INTO ward (name) VALUES (?)', (name,))
    return cur.lastrowid


def _ensure_patient(c, name: str, chart_no: str, ward_id: Optional[int]) -> Optional[int]:
    if not name and not chart_no:
        return None
    if chart_no:
        row = c.execute('SELECT id FROM patient WHERE chart_no=?', (chart_no,)).fetchone()
        if row:
            return row['id']
    cur = c.execute(
        'INSERT INTO patient (ward_id, name, chart_no) VALUES (?, ?, ?)',
        (ward_id, name, chart_no or None)
    )
    return cur.lastrowid


def _ensure_staff(c, name: str, role: str) -> Optional[int]:
    if not name:
        return None
    row = c.execute('SELECT id FROM staff WHERE name=? AND role=?',
                    (name, role)).fetchone()
    if row:
        return row['id']
    cur = c.execute('INSERT INTO staff (name, role) VALUES (?, ?)', (name, role))
    return cur.lastrowid


def _ensure_medicine(c, name: str) -> Optional[int]:
    if not name:
        return None
    row = c.execute('SELECT id FROM medicine WHERE name=?', (name,)).fetchone()
    if row:
        return row['id']
    cur = c.execute('INSERT INTO medicine (name) VALUES (?)', (name,))
    return cur.lastrowid
