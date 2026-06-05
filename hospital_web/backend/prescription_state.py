"""
In-memory prescription store (demo).
Status flow: pending → approved → awaiting_load_confirm → completed
                     → rejected
"""

import threading
import uuid
from datetime import datetime
from typing import Optional

_lock = threading.Lock()
_prescriptions: dict = {}   # id → prescription dict


# ── Seed demo data ────────────────────────────────────────────────────────────

def _seed():
    samples = [
        {
            'patient_name': '홍길동',
            'patient_id':   'PT-2026-0042',
            'ward':         '3병동',
            'doctor':       '김철수',
            'priority':     'emergency',
            'drugs': [
                {'name': '아세트아미노펜 500mg', 'quantity': 3, 'frequency': '1일 3회 식후'},
                {'name': '이부프로펜 400mg',    'quantity': 2, 'frequency': '1일 2회 식후'},
            ],
            'ocr': {
                'raw':      '아세트아미노펜 500mg 3정 / 이부프로펜 400mg 2정',
                'parsed':   [
                    {'name': '아세트아미노펜 500mg', 'qty': 3, 'match': True},
                    {'name': '이부프로펜 400mg',    'qty': 2, 'match': True},
                ],
                'confidence': 97,
            },
            'vision': [
                {'name': '아세트아미노펜 500mg', 'confidence': 94, 'qty_detected': 3, 'match': True},
                {'name': '이부프로펜 400mg',    'confidence': 91, 'qty_detected': 2, 'match': True},
            ],
        },
        {
            'patient_name': '이영희',
            'patient_id':   'PT-2026-0039',
            'ward':         '5병동',
            'doctor':       '박민준',
            'priority':     'general',
            'drugs': [
                {'name': '오메프라졸 20mg',  'quantity': 1, 'frequency': '1일 1회 아침 식전'},
                {'name': '메트포르민 500mg', 'quantity': 2, 'frequency': '1일 2회 식사 중'},
            ],
            'ocr': {
                'raw':    '오메프라졸 20mg 1정 / 메트포르민 500mg 2정',
                'parsed': [
                    {'name': '오메프라졸 20mg',  'qty': 1, 'match': True},
                    {'name': '메트포르민 500mg', 'qty': 2, 'match': True},
                ],
                'confidence': 99,
            },
            'vision': [
                {'name': '오메프라졸 20mg',  'confidence': 88, 'qty_detected': 1, 'match': True},
                {'name': '메트포르민 500mg', 'confidence': 95, 'qty_detected': 2, 'match': True},
            ],
        },
        {
            'patient_name': '최성호',
            'patient_id':   'PT-2026-0051',
            'ward':         '2병동',
            'doctor':       '정수진',
            'priority':     'general',
            'drugs': [
                {'name': '암로디핀 5mg',    'quantity': 1, 'frequency': '1일 1회 아침'},
                {'name': '로수바스타틴 10mg', 'quantity': 1, 'frequency': '1일 1회 저녁'},
            ],
            'ocr': {
                'raw':    '암로디핀 5mg 1정 / 로수바스타틴 20mg 1정',
                'parsed': [
                    {'name': '암로디핀 5mg',     'qty': 1, 'match': True},
                    {'name': '로수바스타틴 20mg', 'qty': 1, 'match': False},  # ← 불일치
                ],
                'confidence': 72,
            },
            'vision': [
                {'name': '암로디핀 5mg',     'confidence': 89, 'qty_detected': 1, 'match': True},
                {'name': '로수바스타틴 10mg', 'confidence': 61, 'qty_detected': 1, 'match': False},
            ],
        },
        {
            'patient_name': '김민서',
            'patient_id':   'PT-2026-0033',
            'ward':         '1병동',
            'doctor':       '이동훈',
            'priority':     'scheduled',
            'drugs': [
                {'name': '레보티록신 50mcg', 'quantity': 1, 'frequency': '1일 1회 공복'},
            ],
            'ocr': {
                'raw':    '레보티록신 50mcg 1정',
                'parsed': [
                    {'name': '레보티록신 50mcg', 'qty': 1, 'match': True},
                ],
                'confidence': 100,
            },
            'vision': [
                {'name': '레보티록신 50mcg', 'confidence': 97, 'qty_detected': 1, 'match': True},
            ],
        },
    ]

    for s in samples:
        pid = f'P-{uuid.uuid4().hex[:6].upper()}'
        _prescriptions[pid] = {
            'id':           pid,
            'patient_name': s['patient_name'],
            'patient_id':   s['patient_id'],
            'ward':         s['ward'],
            'doctor':       s['doctor'],
            'priority':     s['priority'],
            'drugs':        s['drugs'],
            'ocr':          s['ocr'],
            'vision':       s['vision'],
            'status':       'pending',
            'reject_reason': '',
            'pharmacist_note': '',
            'delivery_requested': False,
            'delivery_requested_at': None,
            'created_at':   datetime.now().isoformat(timespec='seconds'),
            'updated_at':   datetime.now().isoformat(timespec='seconds'),
        }


_seed()


# ── Read ──────────────────────────────────────────────────────────────────────

def list_all() -> list:
    priority_order = {'emergency': 0, 'general': 1, 'scheduled': 2}
    with _lock:
        items = list(_prescriptions.values())
    items.sort(key=lambda x: (priority_order.get(x['priority'], 9), x['created_at']))
    return items


def get(pid: str) -> Optional[dict]:
    with _lock:
        return dict(_prescriptions[pid]) if pid in _prescriptions else None


# ── Write ─────────────────────────────────────────────────────────────────────

def approve(pid: str, note: str = '') -> Optional[dict]:
    with _lock:
        if pid not in _prescriptions:
            return None
        p = _prescriptions[pid]
        p['status']          = 'approved'
        p['pharmacist_note'] = note
        p['updated_at']      = datetime.now().isoformat(timespec='seconds')
        return dict(p)


def reject(pid: str, reason: str) -> Optional[dict]:
    with _lock:
        if pid not in _prescriptions:
            return None
        p = _prescriptions[pid]
        p['status']        = 'rejected'
        p['reject_reason'] = reason
        p['updated_at']    = datetime.now().isoformat(timespec='seconds')
        return dict(p)


def set_status(pid: str, status: str) -> Optional[dict]:
    with _lock:
        if pid not in _prescriptions:
            return None
        p = _prescriptions[pid]
        p['status']     = status
        p['updated_at'] = datetime.now().isoformat(timespec='seconds')
        return dict(p)


def create(data: dict) -> dict:
    pid = f'P-{uuid.uuid4().hex[:6].upper()}'
    entry = {
        'id':            pid,
        'patient_name':  data.get('patient_name', ''),
        'patient_id':    data.get('patient_id', ''),
        'ward':          data.get('ward', ''),
        'doctor':        data.get('doctor', ''),
        'priority':      data.get('priority', 'general'),
        'drugs':         data.get('drugs', []),
        'ocr':           {'raw': '', 'parsed': [], 'confidence': 0},
        'vision':        [],
        'status':        'pending',
        'reject_reason': '',
        'pharmacist_note': '',
        'delivery_requested': False,
        'delivery_requested_at': None,
        'created_at':    datetime.now().isoformat(timespec='seconds'),
        'updated_at':    datetime.now().isoformat(timespec='seconds'),
    }
    with _lock:
        _prescriptions[pid] = entry
    return dict(entry)


def request_delivery(pid: str) -> Optional[dict]:
    with _lock:
        if pid not in _prescriptions:
            return None
        p = _prescriptions[pid]
        if p['status'] not in ('approved', 'awaiting_load_confirm'):
            return None
        p['delivery_requested']    = True
        p['delivery_requested_at'] = datetime.now().isoformat(timespec='seconds')
        p['updated_at']            = datetime.now().isoformat(timespec='seconds')
        return dict(p)
