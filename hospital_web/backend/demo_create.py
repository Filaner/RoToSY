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
    # 좌표: mobile_simulation 맵 기준 (단위 m, theta는 rad)
    #   Wing A: y > 0,  Wing B: y < 0
    {'name': '1병동', 'goal_x': 1.7, 'goal_y':  3.2, 'goal_theta': 0},  # Wing A 좌측
    {'name': '2병동', 'goal_x': 4.4, 'goal_y':  3.2, 'goal_theta': 0},  # Wing A 우측
    {'name': '3병동', 'goal_x': 1.7, 'goal_y': -3.2, 'goal_theta': 0},  # Wing B 좌측
    {'name': '5병동', 'goal_x': 4.4, 'goal_y': -3.2, 'goal_theta': 0},  # Wing B 우측
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
    # 실제 보유 약품 (단위 cm). barcode_plane: 박스면 번호 (없으면 None)
    {'name': '벤포벨S',       'width':  5.7, 'depth': 5.6, 'height': 11.0, 'barcode_plane': '4'},
    {'name': '심미안정',      'width':  7.8, 'depth': 4.6, 'height': 10.7, 'barcode_plane': '2'},
    {'name': '유한 비타민C',  'width': 10.7, 'depth': 6.4, 'height':  7.7, 'barcode_plane': '2'},
    {'name': '니뽄 유산균',   'width':  5.7, 'depth': 5.5, 'height': 10.2, 'barcode_plane': '6'},
    {'name': '메디폼 H뷰티',  'width': 10.1, 'depth': 1.6, 'height': 16.5, 'barcode_plane': '3'},
    {'name': '애크논',        'width': 12.5, 'depth': 2.1, 'height':  3.3, 'barcode_plane': None},
    {'name': '임팩타민',      'width':  5.0, 'depth': 5.0, 'height': 10.1, 'barcode_plane': None},
]

# 처방 시드 (고정 코드 — 데모 재현성 확보). 실제 보유 약품으로 임의 매칭.
PRESCRIPTIONS = [
    {
        'code':          'P-DEMO01',
        'patient_chart': 'PT-2026-0042',
        'doctor':        '김철수',
        'priority':      'emergency',
        'drugs': [
            {'name': '벤포벨S',  'quantity': 2, 'frequency': '1일 2회 식후'},
            {'name': '심미안정', 'quantity': 1, 'frequency': '1일 1회 취침 전'},
        ],
        'ocr': {
            'raw':        '벤포벨S 2정 / 심미안정 1정',
            'parsed': [
                {'name': '벤포벨S',  'qty': 2, 'match': True},
                {'name': '심미안정', 'qty': 1, 'match': True},
            ],
            'confidence': 97,
        },
        'vision': [
            {'name': '벤포벨S',  'confidence': 94, 'qty_detected': 2, 'match': True},
            {'name': '심미안정', 'confidence': 91, 'qty_detected': 1, 'match': True},
        ],
    },
    {
        'code':          'P-DEMO02',
        'patient_chart': 'PT-2026-0039',
        'doctor':        '박민준',
        'priority':      'general',
        'drugs': [
            {'name': '유한 비타민C', 'quantity': 1, 'frequency': '1일 1회 아침 식후'},
            {'name': '니뽄 유산균',  'quantity': 2, 'frequency': '1일 2회 식사 중'},
        ],
        'ocr': {
            'raw': '유한 비타민C 1정 / 니뽄 유산균 2정',
            'parsed': [
                {'name': '유한 비타민C', 'qty': 1, 'match': True},
                {'name': '니뽄 유산균',  'qty': 2, 'match': True},
            ],
            'confidence': 99,
        },
        'vision': [
            {'name': '유한 비타민C', 'confidence': 88, 'qty_detected': 1, 'match': True},
            {'name': '니뽄 유산균',  'confidence': 95, 'qty_detected': 2, 'match': True},
        ],
    },
    {
        'code':          'P-DEMO03',
        'patient_chart': 'PT-2026-0051',
        'doctor':        '정수진',
        'priority':      'general',
        'drugs': [
            {'name': '메디폼 H뷰티', 'quantity': 1, 'frequency': '1일 1회 세안 후'},
            {'name': '애크논',       'quantity': 1, 'frequency': '1일 1회 취침 전'},
        ],
        'ocr': {
            'raw': '메디폼 H뷰티 1개 / 애크논 1개',
            'parsed': [
                {'name': '메디폼 H뷰티', 'qty': 1, 'match': True},
                {'name': '애크논',       'qty': 1, 'match': False},
            ],
            'confidence': 72,
        },
        'vision': [
            {'name': '메디폼 H뷰티', 'confidence': 89, 'qty_detected': 1, 'match': True},
            {'name': '애크논',       'confidence': 61, 'qty_detected': 1, 'match': False},
        ],
    },
    {
        'code':          'P-DEMO04',
        'patient_chart': 'PT-2026-0033',
        'doctor':        '이동훈',
        'priority':      'scheduled',
        'drugs': [
            {'name': '임팩타민', 'quantity': 1, 'frequency': '1일 1회 아침 식후'},
        ],
        'ocr': {
            'raw': '임팩타민 1정',
            'parsed': [
                {'name': '임팩타민', 'qty': 1, 'match': True},
            ],
            'confidence': 100,
        },
        'vision': [
            {'name': '임팩타민', 'confidence': 97, 'qty_detected': 1, 'match': True},
        ],
    },
]

CABINETS = [
    {'code': 'CAB-A', 'location': '약품실 A동',
     'magnet_x': 0, 'magnet_y': 0, 'magnet_z': 0,
     'size_x': 600, 'size_y': 800, 'size_z': 400},
]

# (row, col): 'drawer_xxx', aruco, label, medicine, pixel_x, pixel_y
# medicine=None 이면 매핑 없음 (현재는 임팩타민이 슬롯 미배치)
DRAWERS = [
    {'cabinet': 'CAB-A', 'code': 'drawer_001', 'row': 0, 'col': 0, 'aruco': 0, 'label': '비타민B',     'medicine': '벤포벨S',     'px': 100, 'py': 100},
    {'cabinet': 'CAB-A', 'code': 'drawer_002', 'row': 0, 'col': 1, 'aruco': 1, 'label': '신경안정제',  'medicine': '심미안정',     'px': 300, 'py': 100},
    {'cabinet': 'CAB-A', 'code': 'drawer_003', 'row': 1, 'col': 0, 'aruco': 2, 'label': '비타민C',     'medicine': '유한 비타민C', 'px': 100, 'py': 250},
    {'cabinet': 'CAB-A', 'code': 'drawer_004', 'row': 1, 'col': 1, 'aruco': 3, 'label': '유산균',      'medicine': '니뽄 유산균',  'px': 300, 'py': 250},
    {'cabinet': 'CAB-A', 'code': 'drawer_005', 'row': 2, 'col': 0, 'aruco': 4, 'label': '피부미용',    'medicine': '메디폼 H뷰티', 'px': 100, 'py': 400},
    {'cabinet': 'CAB-A', 'code': 'drawer_006', 'row': 2, 'col': 1, 'aruco': 5, 'label': '여드름크림',  'medicine': '애크논',       'px': 300, 'py': 400},
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
            c.execute(
                '''INSERT INTO ward (name, goal_x, goal_y, goal_theta)
                   VALUES (?, ?, ?, ?)
                   ON CONFLICT(name) DO UPDATE SET
                       goal_x     = excluded.goal_x,
                       goal_y     = excluded.goal_y,
                       goal_theta = excluded.goal_theta''',
                (w['name'], w.get('goal_x'), w.get('goal_y'), w.get('goal_theta', 0))
            )

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

        # medicine (name 기준 UNIQUE). 재시딩 시 사이즈/바코드도 갱신.
        for m in MEDICINES:
            c.execute(
                '''INSERT INTO medicine
                   (name, display_name, width, depth, height, barcode_plane)
                   VALUES (?, ?, ?, ?, ?, ?)
                   ON CONFLICT(name) DO UPDATE SET
                       display_name  = excluded.display_name,
                       width         = excluded.width,
                       depth         = excluded.depth,
                       height        = excluded.height,
                       barcode_plane = excluded.barcode_plane''',
                (m['name'], m['name'],
                 m['width'], m['depth'], m['height'], m['barcode_plane'])
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

        # cabinet_slot (code 기준 UNIQUE). 재시딩 시 라벨/약품 매핑 갱신.
        for d in DRAWERS:
            cab_id = _id_of(c, 'cabinet', 'code=?', (d['cabinet'],))
            med_id = (_id_of(c, 'medicine', 'name=?', (d['medicine'],))
                      if d.get('medicine') else None)
            c.execute(
                '''INSERT INTO cabinet_slot
                   (cabinet_id, medicine_id, code, row_idx, col_idx,
                    aruco_marker_id, label, pixel_x, pixel_y)
                   VALUES (?,?,?,?,?,?,?,?,?)
                   ON CONFLICT(code) DO UPDATE SET
                       medicine_id     = excluded.medicine_id,
                       label           = excluded.label,
                       aruco_marker_id = excluded.aruco_marker_id''',
                (cab_id, med_id, d['code'], d['row'], d['col'], d['aruco'],
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
