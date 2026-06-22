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
    #   admin/test 프리셋과 일치하는 검증된 좌표. 시뮬에선 병동 A/B 2개만 사용.
    {'name': '병동A',  'goal_x':  4.20, 'goal_y':  2.72, 'goal_theta':  1.5708},  # Wing A (y>0)
    {'name': '병동B',  'goal_x':  3.60, 'goal_y': -2.50, 'goal_theta': -1.5708},  # Wing B (corrected to -2.5)
    {'name': '약재실', 'goal_x': -4.30, 'goal_y':  2.50, 'goal_theta': -1.5708},  # 홈/복귀 지점
    {'name': '간호스테이션', 'goal_x': -2.50, 'goal_y': -3.20, 'goal_theta': 0.0},     # 추가: 간호사 수령 지점
]

STAFF = [
    {'name': '김철수', 'role': 'DOCTOR'},
    {'name': '박민준', 'role': 'DOCTOR'},
    {'name': '정수진', 'role': 'DOCTOR'},
    {'name': '이동훈', 'role': 'DOCTOR'},
]

PATIENTS = [
    {'name': '홍길동', 'chart_no': 'PT-2026-0042', 'ward': '병동B'},
    {'name': '이영희', 'chart_no': 'PT-2026-0039', 'ward': '병동B'},
    {'name': '최성호', 'chart_no': 'PT-2026-0051', 'ward': '병동A'},
    {'name': '김민서', 'chart_no': 'PT-2026-0033', 'ward': '병동A'},
]

MEDICINES = [
    # 실제 보유 약품 (단위 cm). drawer_num: ArUco 마커 번호 = 서랍장 번호 (1~6)
    {'name': '벤포벨S',           'width':  5.7, 'depth': 5.6, 'height': 11.0, 'drawer_num': '5'},
    {'name': '심미안정',          'width':  7.8, 'depth': 4.6, 'height': 10.7, 'drawer_num': '3'},
    {'name': '유한 비타민C',      'width': 10.7, 'depth': 6.4, 'height':  7.7, 'drawer_num': '6'},
    {'name': '新ビオフェルミンS錠', 'width':  5.7, 'depth': 5.5, 'height': 10.2, 'drawer_num': '4'},
    {'name': '메디폼 H뷰티',      'width': 10.1, 'depth': 1.6, 'height': 16.5, 'drawer_num': '1'},
    {'name': '애크논',            'width': 12.5, 'depth': 2.1, 'height':  3.3, 'drawer_num': None},
    {'name': '임팩타민',          'width':  5.0, 'depth': 5.0, 'height': 10.1, 'drawer_num': None},
    {'name': 'MEDIPHARMAPLAN',    'width':  9.8, 'depth': 2.0, 'height': 18.2, 'drawer_num': '2'},
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
            {'name': '新ビオフェルミンS錠', 'quantity': 1, 'frequency': '1일 1회 취침 전'},
        ],
        'ocr': {
            'raw':        '벤포벨S 2정 / 新ビオフェルミンS錠 1정',
            'parsed': [
                {'name': '벤포벨S',  'qty': 2, 'match': True},
                {'name': '新ビオフェルミンS錠', 'qty': 1, 'match': True},
            ],
            'confidence': 97,
        },
        'vision': [
            {'name': '벤포벨S',  'confidence': 94, 'qty_detected': 2, 'match': True},
            {'name': '新ビオフェルミンS錠', 'confidence': 91, 'qty_detected': 1, 'match': True},
        ],
    },
    {
        'code':          'P-DEMO02',
        'patient_chart': 'PT-2026-0039',
        'doctor':        '박민준',
        'priority':      'general',
        'drugs': [
            {'name': '유한 비타민C', 'quantity': 1, 'frequency': '1일 1회 아침 식후'},
            {'name': '新ビオフェルミンS錠',  'quantity': 2, 'frequency': '1일 2회 식사 중'},
        ],
        'ocr': {
            'raw': '유한 비타민C 1정 / 니뽄 유산균 2정',
            'parsed': [
                {'name': '유한 비타민C', 'qty': 1, 'match': True},
                {'name': '新ビオフェルミンS錠',  'qty': 2, 'match': True},
            ],
            'confidence': 99,
        },
        'vision': [
            {'name': '유한 비타민C', 'confidence': 88, 'qty_detected': 1, 'match': True},
            {'name': '新ビオフェルミンS錠',  'confidence': 95, 'qty_detected': 2, 'match': True},
        ],
    },
    {
        'code':          'P-DEMO03',
        'patient_chart': 'PT-2026-0051',
        'doctor':        '정수진',
        'priority':      'general',
        'drugs': [
            {'name': '메디폼 H뷰티', 'quantity': 1, 'frequency': '1일 1회 세안 후'},
            {'name': 'MEDIPHARMAPLAN',       'quantity': 1, 'frequency': '1일 1회 취침 전'},
        ],
        'ocr': {
            'raw': '메디폼 H뷰티 1개 / 애크논 1개',
            'parsed': [
                {'name': '메디폼 H뷰티', 'qty': 1, 'match': True},
                {'name': 'MEDIPHARMAPLAN',       'qty': 1, 'match': False},
            ],
            'confidence': 72,
        },
        'vision': [
            {'name': '메디폼 H뷰티', 'confidence': 89, 'qty_detected': 1, 'match': True},
            {'name': 'MEDIPHARMAPLAN',       'confidence': 61, 'qty_detected': 1, 'match': False},
        ],
    },
    {
        'code':          'P-DEMO04',
        'patient_chart': 'PT-2026-0033',
        'doctor':        '이동훈',
        'priority':      'scheduled',
        'drugs': [
            {'name': '벤포벨S', 'quantity': 1, 'frequency': '1일 1회 아침 식후'},
        ],
        'ocr': {
            'raw': '벤포벨S 1정',
            'parsed': [
                {'name': '벤포벨S', 'qty': 1, 'match': True},
            ],
            'confidence': 100,
        },
        'vision': [
            {'name': '벤포벨S', 'confidence': 97, 'qty_detected': 1, 'match': True},
        ],
    },
]

CABINETS = [
    {'code': 'CAB-A', 'location': '약품실 A동',
     'magnet_x': 0, 'magnet_y': 0, 'magnet_z': 0,
     'size_x': 600, 'size_y': 800, 'size_z': 400},
]

# 병동별 배송 박스 (팔레타이징 목적지). 치수 단위 mm.
#   물리 배치: 250×190 흰색 박스 2개가 '긴 면(250)'을 맞대고 붙어 있음. 각 박스
#             정중앙에 ArUco 마커 — 카메라 이미지 상단=마커 3, 하단=마커 4.
#   inner_w = 박스 로컬 X(긴 변 250, 마커 X축 방향), inner_d = 박스 로컬 Y(짧은 변 190).
#   aruco: 박스 정중앙 마커. 마커4=1병동(병동A/BOX-A), 마커3=2병동(병동B/BOX-B).
#          (서랍 마커 0~5 는 제거됨 → 서랍은 고정 좌표 접근, 0·1 은 캘리브레이션용.)
#   origin_*: 마커 미검출 시 fallback 으로 쓸 박스 '정중앙'의 base 좌표 (mm).
#            RealSense 실측값(2026-06-19 캡처) 기준. 마커 검출 시엔 실측 자동 사용.
DELIVERY_BOXES = [
    {'code': 'BOX-A', 'ward': '병동A', 'inner_w': 250, 'inner_d': 190, 'inner_h': 120,
     'wall_margin': 5, 'item_gap': 3, 'aruco': 4, 'origin': (586.0, 328.0, -15.0)},
    {'code': 'BOX-B', 'ward': '병동B', 'inner_w': 250, 'inner_d': 190, 'inner_h': 120,
     'wall_margin': 5, 'item_gap': 3, 'aruco': 3, 'origin': (368.0, 313.0, -23.0)},
]

# (row, col): 'drawer_xxx', aruco(=서랍 번호, 1-based), label, medicine, pixel_x, pixel_y, stock
# medicine=None 이면 매핑 없음
DRAWERS = [
    {'cabinet': 'CAB-A', 'code': 'drawer_001', 'row': 0, 'col': 0, 'aruco': 1, 'label': '피부미용',    'medicine': '메디폼 H뷰티',       'px': 100, 'py': 100, 'stock': 1},
    {'cabinet': 'CAB-A', 'code': 'drawer_002', 'row': 0, 'col': 1, 'aruco': 2, 'label': '수액팩',      'medicine': 'MEDIPHARMAPLAN',     'px': 300, 'py': 100, 'stock': 2},
    {'cabinet': 'CAB-A', 'code': 'drawer_003', 'row': 1, 'col': 0, 'aruco': 3, 'label': '신경안정제',  'medicine': '심미안정',            'px': 100, 'py': 250, 'stock': 1},
    {'cabinet': 'CAB-A', 'code': 'drawer_004', 'row': 1, 'col': 1, 'aruco': 4, 'label': '일본 유산균', 'medicine': '新ビオフェルミンS錠', 'px': 300, 'py': 250, 'stock': 3},
    {'cabinet': 'CAB-A', 'code': 'drawer_005', 'row': 2, 'col': 0, 'aruco': 5, 'label': '비타민B',     'medicine': '벤포벨S',             'px': 100, 'py': 400, 'stock': 3},
    {'cabinet': 'CAB-A', 'code': 'drawer_006', 'row': 2, 'col': 1, 'aruco': 6, 'label': '비타민C',     'medicine': '유한 비타민C',        'px': 300, 'py': 400, 'stock': 1},
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
                   (name, display_name, width, depth, height, drawer_num)
                   VALUES (?, ?, ?, ?, ?, ?)
                   ON CONFLICT(name) DO UPDATE SET
                       display_name = excluded.display_name,
                       width        = excluded.width,
                       depth        = excluded.depth,
                       height       = excluded.height,
                       drawer_num   = excluded.drawer_num''',
                (m['name'], m['name'],
                 m['width'], m['depth'], m['height'], m.get('drawer_num'))
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

        # delivery_box (code 기준 UNIQUE). 재시딩 시 치수/마커/원점 갱신.
        for b in DELIVERY_BOXES:
            ward_id = _id_of(c, 'ward', 'name=?', (b['ward'],))
            ox, oy, oz = b['origin']
            c.execute(
                '''INSERT INTO delivery_box
                   (ward_id, code, inner_w, inner_d, inner_h, wall_margin,
                    item_gap, aruco_marker_id, origin_x, origin_y, origin_z)
                   VALUES (?,?,?,?,?,?,?,?,?,?,?)
                   ON CONFLICT(code) DO UPDATE SET
                       ward_id         = excluded.ward_id,
                       inner_w         = excluded.inner_w,
                       inner_d         = excluded.inner_d,
                       inner_h         = excluded.inner_h,
                       wall_margin     = excluded.wall_margin,
                       item_gap        = excluded.item_gap,
                       aruco_marker_id = excluded.aruco_marker_id,
                       origin_x        = excluded.origin_x,
                       origin_y        = excluded.origin_y,
                       origin_z        = excluded.origin_z''',
                (ward_id, b['code'], b['inner_w'], b['inner_d'], b['inner_h'],
                 b['wall_margin'], b['item_gap'], b['aruco'], ox, oy, oz)
            )

        # cabinet_slot (code 기준 UNIQUE). 재시딩 시 라벨/약품 매핑 갱신.
        for d in DRAWERS:
            cab_id = _id_of(c, 'cabinet', 'code=?', (d['cabinet'],))
            med_id = (_id_of(c, 'medicine', 'name=?', (d['medicine'],))
                      if d.get('medicine') else None)
            c.execute(
                '''INSERT INTO cabinet_slot
                   (cabinet_id, medicine_id, code, row_idx, col_idx,
                    aruco_marker_id, label, pixel_x, pixel_y, current_stock)
                   VALUES (?,?,?,?,?,?,?,?,?,?)
                   ON CONFLICT(code) DO UPDATE SET
                       medicine_id   = excluded.medicine_id,
                       label         = excluded.label,
                       aruco_marker_id = excluded.aruco_marker_id,
                       current_stock = excluded.current_stock''',
                (cab_id, med_id, d['code'], d['row'], d['col'], d['aruco'],
                 d['label'], d['px'], d['py'], d.get('stock', 0))
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
        # FK 순서 주의: ocr_scan/audit_log(자식)을 mission·prescription(부모)보다 먼저 삭제
        for t in ('ocr_scan', 'audit_log', 'pallet_plan', 'mission',
                  'prescription_item', 'prescription',
                  'cabinet_slot', 'cabinet', 'delivery_box',
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
