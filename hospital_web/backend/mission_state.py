"""
Mission + Audit log — DB-backed (was in-memory).

Public 시그니처는 기존 in-memory 모듈과 동일:
  get_mission(), get_audit_log(limit),
  new_mission(destination, prescription_id),
  confirm_loading(actor),
  update_status(status, actor='system', detail=''),
  add_audit(actor, action, detail='')

반환 dict는 _snapshot() 그대로:
  {
    'mission_id': 'M-XXXXXX' or None,
    'prescription_id': 'P-XXXXXX' or None,
    'status': 'IDLE'|'AWAITING_CONFIRM'|...,
    'destination': str,
    'pharmacist_confirmed': bool, 'admin_confirmed': bool,
    'can_dispatch': bool,
    'created_at': iso or None, 'dispatched_at': iso or None
  }

"현재 미션" = mission 테이블의 가장 최근 row (created_at DESC).
없으면 IDLE 스냅샷.
"""

import threading
import uuid
from datetime import datetime
from typing import Optional

from .db_schema import get_conn

_lock = threading.Lock()


def _now() -> str:
    return datetime.now().isoformat(timespec='seconds')


def _gen_code() -> str:
    return f'M-{uuid.uuid4().hex[:6].upper()}'


def _idle_snapshot() -> dict:
    return {
        'mission_id':           None,
        'prescription_id':      None,
        'status':               'IDLE',
        'destination':          '',
        'pharmacist_confirmed': False,
        'admin_confirmed':      False,
        'can_dispatch':         False,
        'created_at':           None,
        'dispatched_at':        None,
    }


def _row_to_snapshot(c, row) -> dict:
    pres_code = None
    if row['prescription_id']:
        p = c.execute('SELECT code FROM prescription WHERE id=?',
                      (row['prescription_id'],)).fetchone()
        if p:
            pres_code = p['code']
    pharm = bool(row['pharmacist_confirmed'])
    admin = bool(row['admin_confirmed'])
    status = row['status']
    return {
        'mission_id':           row['code'],
        'prescription_id':      pres_code,
        'status':               status,
        'destination':          row['destination'] or '',
        'pharmacist_confirmed': pharm,
        'admin_confirmed':      admin,
        'can_dispatch':         pharm and admin and status == 'CONFIRMED',
        'created_at':           row['created_at'],
        'dispatched_at':        row['dispatched_at'],
    }


def _latest_row(c):
    return c.execute(
        'SELECT * FROM mission ORDER BY created_at DESC, id DESC LIMIT 1'
    ).fetchone()


# ── Read ──────────────────────────────────────────────────────────────────────

def get_mission() -> dict:
    with _lock, get_conn() as c:
        row = _latest_row(c)
        if not row:
            return _idle_snapshot()
        return _row_to_snapshot(c, row)


def get_audit_log(limit: int = 100) -> list:
    with _lock, get_conn() as c:
        rows = c.execute(
            '''SELECT created_at, actor, action, detail
               FROM audit_log
               ORDER BY id DESC
               LIMIT ?''',
            (limit,)
        ).fetchall()
    return [
        {'timestamp': r['created_at'],
         'actor':     r['actor'],
         'action':    r['action'],
         'detail':    r['detail'] or ''}
        for r in rows
    ]


# ── Write ─────────────────────────────────────────────────────────────────────

def new_mission(destination: str, prescription_id: str = '') -> dict:
    code = _gen_code()
    now = _now()
    with _lock, get_conn() as c:
        pres_int_id: Optional[int] = None
        if prescription_id:
            r = c.execute('SELECT id FROM prescription WHERE code=?',
                          (prescription_id,)).fetchone()
            if r:
                pres_int_id = r['id']

        c.execute(
            '''INSERT INTO mission
               (code, prescription_id, destination, status, created_at)
               VALUES (?, ?, ?, 'AWAITING_CONFIRM', ?)''',
            (code, pres_int_id, destination, now)
        )
        mission_int_id = c.execute('SELECT id FROM mission WHERE code=?',
                                   (code,)).fetchone()['id']
        _insert_audit(c, mission_int_id, 'system', 'MISSION_CREATED',
                      f'{code} → {destination}')
        row = c.execute('SELECT * FROM mission WHERE id=?',
                        (mission_int_id,)).fetchone()
        return _row_to_snapshot(c, row)


def confirm_loading(actor: str) -> dict:
    """actor: 'pharmacist' | 'admin'"""
    now = _now()
    with _lock, get_conn() as c:
        row = _latest_row(c)
        if not row:
            return _idle_snapshot()
        mission_int_id = row['id']

        if actor == 'pharmacist':
            c.execute('UPDATE mission SET pharmacist_confirmed=1 WHERE id=?',
                      (mission_int_id,))
        elif actor == 'admin':
            c.execute('UPDATE mission SET admin_confirmed=1 WHERE id=?',
                      (mission_int_id,))

        _insert_audit(c, mission_int_id, actor, 'CONFIRM_LOADING',
                      f"{row['code']} 적재 확인")

        row = c.execute('SELECT * FROM mission WHERE id=?',
                        (mission_int_id,)).fetchone()
        if (row['pharmacist_confirmed'] and row['admin_confirmed']
                and row['status'] == 'AWAITING_CONFIRM'):
            c.execute(
                'UPDATE mission SET status=?, confirmed_at=? WHERE id=?',
                ('CONFIRMED', now, mission_int_id)
            )
            _insert_audit(c, mission_int_id, 'system', 'BOTH_CONFIRMED',
                          f"{row['code']} 출발 가능")
            row = c.execute('SELECT * FROM mission WHERE id=?',
                            (mission_int_id,)).fetchone()
        return _row_to_snapshot(c, row)


def update_status(status: str, actor: str = 'system', detail: str = '') -> dict:
    now = _now()
    with _lock, get_conn() as c:
        row = _latest_row(c)
        if not row:
            return _idle_snapshot()
        mission_int_id = row['id']
        prev = row['status']

        fields = ['status=?']
        params = [status]
        if status == 'DISPATCHED':
            fields.append('dispatched_at=?'); params.append(now)
        elif status == 'ARRIVED':
            fields.append('arrived_at=?'); params.append(now)
        elif status == 'COMPLETED':
            fields.append('completed_at=?'); params.append(now)
        params.append(mission_int_id)
        c.execute(f'UPDATE mission SET {", ".join(fields)} WHERE id=?', params)

        _insert_audit(c, mission_int_id, actor, f'STATUS_{status}',
                      detail or f'{prev} → {status}')

        row = c.execute('SELECT * FROM mission WHERE id=?',
                        (mission_int_id,)).fetchone()
        return _row_to_snapshot(c, row)


def add_audit(actor: str, action: str, detail: str = '') -> None:
    with _lock, get_conn() as c:
        row = _latest_row(c)
        mid = row['id'] if row else None
        _insert_audit(c, mid, actor, action, detail)


# ── internal ─────────────────────────────────────────────────────────────────

def _insert_audit(c, mission_id: Optional[int], actor: str,
                  action: str, detail: str) -> None:
    c.execute(
        '''INSERT INTO audit_log (mission_id, actor, action, detail, created_at)
           VALUES (?, ?, ?, ?, ?)''',
        (mission_id, actor, action, detail, _now())
    )
