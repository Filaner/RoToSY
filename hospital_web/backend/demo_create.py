"""
Idempotent demo seed.

기존 prescription_state._seed() / drawer_metadata 모듈 상수의 데모 데이터를
DB에 박는다. 같은 코드(예: 'PT-2026-0042', 'drawer_001')는 두 번 실행해도 중복
생성되지 않음 (INSERT OR IGNORE + 코드/이름 기준 lookup).

CLI:
  python -m backend.demo_create               # 시드 박기 (기본)
  python -m backend.demo_create --reset       # 데모 row 다 지우고 다시 박기
"""

import argparse
import json
import sys
from datetime import datetime
from typing import Optional

from .db_schema import init_schema, get_conn


# ── 데이터 정의 ───────────────────────────────────────────────────────────────

WARDS = [
    {'name': '1병동'}, {'name': '2병동'}, {'name': '3병동'},
    {'name': '5병동'},
]

STAFF = [
    {'name': '김철수', 'role': 'DOCTOR'},
    {'name': '박민준', 'role': 'DOCTOR'},
    {'name': '정수진', 'role': 'DOCTOR'},
    {'name': '이동훈', 'role': 'DOCTOR'},
]

PATIENTS = [
    {'name': '홍길동', 'chart_no': 'PT-2026-0042', 'ward': '3병동'},
    {'name': '이영희', 'chart_no': 'PT-2026-0039', 'ward': '5병동'},
    {'name': '최성호', 'chart_no': 'PT-2026-0051', 'ward': '2병동'},
    {'name': '김민서', 'chart_no': 'PT-2026-0033', 'ward': '1병동'},
]

MEDICINES = [
    '아세트아미노펜 500mg', '이부프로펜 400mg',
    '오메프라졸 20mg', '메트포르민 500mg',
    '암로디핀 5mg', '로수바스타틴 10mg',
    '레보티록신 50mcg',
]

# 처방 시드 (고정 코드 — 데모 재현성 확보)
PRESCRIPTIONS = [
    {
        'code':          'P-DEMO01',
        'patient_chart': 'PT-2026-0042',
        'doctor':        '김철수',
        'priority':      'emergency',
        'drugs': [
            {'name': '아세트아미노펜 500mg', 'quantity': 3, 'frequency': '1일 3회 식후'},
            {'name': '이부프로펜 400mg',    'quantity': 2, 'frequency': '1일 2회 식후'},
        ],
        'ocr': {
            'raw':        '아세트아미노펜 500mg 3정 / 이부프로펜 400mg 2정',
            'parsed': [
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
        'code':          'P-DEMO02',
        'patient_chart': 'PT-2026-0039',
        'doctor':        '박민준',
        'priority':      'general',
        'drugs': [
            {'name': '오메프라졸 20mg',  'quantity': 1, 'frequency': '1일 1회 아침 식전'},
            {'name': '메트포르민 500mg', 'quantity': 2, 'frequency': '1일 2회 식사 중'},
        ],
        'ocr': {
            'raw': '오메프라졸 20mg 1정 / 메트포르민 500mg 2정',
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
        'code':          'P-DEMO03',
        'patient_chart': 'PT-2026-0051',
        'doctor':        '정수진',
        'priority':      'general',
        'drugs': [
            {'name': '암로디핀 5mg',     'quantity': 1, 'frequency': '1일 1회 아침'},
            {'name': '로수바스타틴 10mg', 'quantity': 1, 'frequency': '1일 1회 저녁'},
        ],
        'ocr': {
            'raw': '암로디핀 5mg 1정 / 로수바스타틴 20mg 1정',
            'parsed': [
                {'name': '암로디핀 5mg',     'qty': 1, 'match': True},
                {'name': '로수바스타틴 20mg', 'qty': 1, 'match': False},
            ],
            'confidence': 72,
        },
        'vision': [
            {'name': '암로디핀 5mg',     'confidence': 89, 'qty_detected': 1, 'match': True},
            {'name': '로수바스타틴 10mg', 'confidence': 61, 'qty_detected': 1, 'match': False},
        ],
    },
    {
        'code':          'P-DEMO04',
        'patient_chart': 'PT-2026-0033',
        'doctor':        '이동훈',
        'priority':      'scheduled',
        'drugs': [
            {'name': '레보티록신 50mcg', 'quantity': 1, 'frequency': '1일 1회 공복'},
        ],
        'ocr': {
            'raw': '레보티록신 50mcg 1정',
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

CABINETS = [
    {'code': 'CAB-A', 'location': '약품실 A동',
     'magnet_x': 0, 'magnet_y': 0, 'magnet_z': 0,
     'size_x': 600, 'size_y': 800, 'size_z': 400},
]

# (row, col): 'drawer_xxx', aruco, label, pixel_x, pixel_y
DRAWERS = [
    {'cabinet': 'CAB-A', 'code': 'drawer_001', 'row': 0, 'col': 0, 'aruco': 10, 'label': '감기약', 'px': 100, 'py': 100},
    {'cabinet': 'CAB-A', 'code': 'drawer_002', 'row': 0, 'col': 1, 'aruco': 11, 'label': '소화제', 'px': 300, 'py': 100},
    {'cabinet': 'CAB-A', 'code': 'drawer_003', 'row': 1, 'col': 0, 'aruco': 20, 'label': '항생제', 'px': 100, 'py': 250},
    {'cabinet': 'CAB-A', 'code': 'drawer_004', 'row': 1, 'col': 1, 'aruco': 21, 'label': '항염증', 'px': 300, 'py': 250},
    {'cabinet': 'CAB-A', 'code': 'drawer_005', 'row': 2, 'col': 0, 'aruco': 30, 'label': '혈압약', 'px': 100, 'py': 400},
    {'cabinet': 'CAB-A', 'code': 'drawer_006', 'row': 2, 'col': 1, 'aruco': 31, 'label': '당뇨약', 'px': 300, 'py': 400},
]


# ── 시드 함수 ─────────────────────────────────────────────────────────────────

def _now() -> str:
    return datetime.now().isoformat(timespec='seconds')


def _id_of(c, table: str, where: str, params: tuple) -> Optional[int]:
    row = c.execute(f'SELECT id FROM {table} WHERE {where}', params).fetchone()
    return row['id'] if row else None


def seed() -> None:
    init_schema()
    with get_conn() as c:
        # ward
        for w in WARDS:
            c.execute('INSERT OR IGNORE INTO ward (name) VALUES (?)', (w['name'],))

        # staff
        for s in STAFF:
            c.execute(
                'INSERT OR IGNORE INTO staff (name, role) VALUES (?, ?)',
                (s['name'], s['role'])
            )

        # patient (chart_no 기준 UNIQUE)
        for p in PATIENTS:
            ward_id = _id_of(c, 'ward', 'name=?', (p['ward'],))
            c.execute(
                'INSERT OR IGNORE INTO patient (name, chart_no, ward_id) VALUES (?,?,?)',
                (p['name'], p['chart_no'], ward_id)
            )

        # medicine (name 기준 UNIQUE)
        for m_name in MEDICINES:
            c.execute(
                'INSERT OR IGNORE INTO medicine (name, display_name) VALUES (?, ?)',
                (m_name, m_name)
            )

        # cabinet (code 기준 UNIQUE)
        for cab in CABINETS:
            c.execute(
                '''INSERT OR IGNORE INTO cabinet
                   (code, location, magnet_x, magnet_y, magnet_z, size_x, size_y, size_z)
                   VALUES (?,?,?,?,?,?,?,?)''',
                (cab['code'], cab['location'],
                 cab['magnet_x'], cab['magnet_y'], cab['magnet_z'],
                 cab['size_x'], cab['size_y'], cab['size_z'])
            )

        # cabinet_slot (code 기준 UNIQUE)
        for d in DRAWERS:
            cab_id = _id_of(c, 'cabinet', 'code=?', (d['cabinet'],))
            c.execute(
                '''INSERT OR IGNORE INTO cabinet_slot
                   (cabinet_id, code, row_idx, col_idx, aruco_marker_id,
                    label, pixel_x, pixel_y)
                   VALUES (?,?,?,?,?,?,?,?)''',
                (cab_id, d['code'], d['row'], d['col'], d['aruco'],
                 d['label'], d['px'], d['py'])
            )

        # prescription (code 기준 UNIQUE)
        for pres in PRESCRIPTIONS:
            already = c.execute(
                'SELECT 1 FROM prescription WHERE code=?', (pres['code'],)
            ).fetchone()
            if already:
                continue

            patient_id = _id_of(c, 'patient', 'chart_no=?', (pres['patient_chart'],))
            doctor_id  = _id_of(c, 'staff',   'name=? AND role=?',
                                (pres['doctor'], 'DOCTOR'))
            now = _now()
            c.execute(
                '''INSERT INTO prescription
                   (code, patient_id, doctor_id, priority, status,
                    ocr_raw, ocr_confidence, ocr_parsed, vision_data,
                    created_at, updated_at)
                   VALUES (?, ?, ?, ?, 'pending', ?, ?, ?, ?, ?, ?)''',
                (pres['code'], patient_id, doctor_id, pres['priority'],
                 pres['ocr']['raw'], pres['ocr']['confidence'],
                 json.dumps(pres['ocr']['parsed'], ensure_ascii=False),
                 json.dumps(pres['vision'],         ensure_ascii=False),
                 now, now)
            )
            pres_id = c.execute('SELECT id FROM prescription WHERE code=?',
                                (pres['code'],)).fetchone()['id']
            for i, drug in enumerate(pres['drugs']):
                med_id = _id_of(c, 'medicine', 'name=?', (drug['name'],))
                c.execute(
                    '''INSERT INTO prescription_item
                       (prescription_id, medicine_id, medicine_name, quantity,
                        frequency, sort_order)
                       VALUES (?,?,?,?,?,?)''',
                    (pres_id, med_id, drug['name'], drug['quantity'],
                     drug['frequency'], i)
                )
    print('[demo_create] seed 완료')


def reset() -> None:
    """모든 도메인 row 삭제 후 재시딩. sensor_db 테이블은 건드리지 않음."""
    init_schema()
    with get_conn() as c:
        for t in ('audit_log', 'mission',
                  'prescription_item', 'prescription',
                  'cabinet_slot', 'cabinet',
                  'patient', 'staff', 'ward', 'medicine'):
            c.execute(f'DELETE FROM {t}')
    print('[demo_create] 기존 row 전부 삭제')
    seed()


def main() -> int:
    ap = argparse.ArgumentParser(description='hospital_web 데모 시드')
    ap.add_argument('--reset', action='store_true',
                    help='기존 row 전부 지우고 다시 박기')
    args = ap.parse_args()
    if args.reset:
        reset()
    else:
        seed()
    return 0


if __name__ == '__main__':
    sys.exit(main())
