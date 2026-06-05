"""
In-memory shared state: mission lifecycle, dual-confirmation, audit log.

Mission flow:
  IDLE → AWAITING_CONFIRM → CONFIRMED → DISPATCHED → DELIVERING → ARRIVED → COMPLETED
                                ↑
              pharmacist_confirmed AND admin_confirmed
"""

import threading
import uuid
from datetime import datetime
from typing import Optional


class _MissionState:
    def __init__(self):
        self.mission_id:           Optional[str]      = None
        self.prescription_id:      Optional[str]      = None
        self.status:               str                = 'IDLE'
        self.destination:          str                = ''
        self.pharmacist_confirmed: bool               = False
        self.admin_confirmed:      bool               = False
        self.created_at:           Optional[datetime] = None
        self.dispatched_at:        Optional[datetime] = None


_lock   = threading.Lock()
_m      = _MissionState()
_audit: list = []


# ── Read ─────────────────────────────────────────────────────────────────────

def get_mission() -> dict:
    with _lock:
        return _snapshot()


def get_audit_log(limit: int = 100) -> list:
    with _lock:
        return list(reversed(_audit[-limit:]))


# ── Write ─────────────────────────────────────────────────────────────────────

def new_mission(destination: str, prescription_id: str = '') -> dict:
    with _lock:
        _m.mission_id          = f'M-{uuid.uuid4().hex[:6].upper()}'
        _m.prescription_id     = prescription_id or None
        _m.status              = 'AWAITING_CONFIRM'
        _m.destination         = destination
        _m.pharmacist_confirmed = False
        _m.admin_confirmed     = False
        _m.created_at          = datetime.now()
        _m.dispatched_at       = None
        _log('system', 'MISSION_CREATED', f'{_m.mission_id} → {destination}')
        return _snapshot()


def confirm_loading(actor: str) -> dict:
    """actor: 'pharmacist' | 'admin'"""
    with _lock:
        if actor == 'pharmacist':
            _m.pharmacist_confirmed = True
        elif actor == 'admin':
            _m.admin_confirmed = True
        _log(actor, 'CONFIRM_LOADING', f'{_m.mission_id} 적재 확인')
        if _m.pharmacist_confirmed and _m.admin_confirmed and _m.status == 'AWAITING_CONFIRM':
            _m.status = 'CONFIRMED'
            _log('system', 'BOTH_CONFIRMED', f'{_m.mission_id} 출발 가능')
        return _snapshot()


def update_status(status: str, actor: str = 'system', detail: str = '') -> dict:
    with _lock:
        prev = _m.status
        _m.status = status
        if status == 'DISPATCHED':
            _m.dispatched_at = datetime.now()
        if status == 'IDLE':
            _reset()
        _log(actor, f'STATUS_{status}', detail or f'{prev} → {status}')
        return _snapshot()


def add_audit(actor: str, action: str, detail: str = '') -> None:
    with _lock:
        _log(actor, action, detail)


# ── Internal ──────────────────────────────────────────────────────────────────

def _snapshot() -> dict:
    return {
        'mission_id':           _m.mission_id,
        'prescription_id':      _m.prescription_id,
        'status':               _m.status,
        'destination':          _m.destination,
        'pharmacist_confirmed': _m.pharmacist_confirmed,
        'admin_confirmed':      _m.admin_confirmed,
        'can_dispatch': (
            _m.pharmacist_confirmed and _m.admin_confirmed
            and _m.status == 'CONFIRMED'
        ),
        'created_at':    _m.created_at.isoformat()    if _m.created_at    else None,
        'dispatched_at': _m.dispatched_at.isoformat() if _m.dispatched_at else None,
    }


def _log(actor: str, action: str, detail: str) -> None:
    _audit.append({
        'timestamp': datetime.now().isoformat(timespec='seconds'),
        'actor':     actor,
        'action':    action,
        'detail':    detail,
    })


def _reset() -> None:
    _m.mission_id           = None
    _m.prescription_id      = None
    _m.status               = 'IDLE'
    _m.destination          = ''
    _m.pharmacist_confirmed  = False
    _m.admin_confirmed      = False
    _m.created_at           = None
    _m.dispatched_at        = None
