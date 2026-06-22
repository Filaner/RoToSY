"""
팔레타이징 적재(packing) 로직.

plan_for_mission(mission_code, box_code)
  - 현재(또는 지정) 미션의 처방 품목 + medicine 치수로 2D bin-packing
  - 목적지 박스(병동별 delivery_box)의 내부 가용 영역에 직사각형 배치
  - 결과를 pallet_plan 테이블에 저장하고 슬롯 레이아웃 반환

get_plan(mission_code)        — 저장된 레이아웃 조회
mark_placed(mission_code, slot_idx) — 슬롯 배치 완료 표시

좌표계:
  박스 로컬 프레임. 원점 = 박스 내부 가용 영역(벽 여유 포함)의 좌하단 모서리.
  local_x → inner_w 방향(폭), local_y → inner_d 방향(깊이). 단위 mm.
  슬롯의 local_x/local_y 는 품목 footprint 의 '중심' 좌표.
  rot_deg: 0 = 원래 방향(w=width,h=depth), 90 = 90도 회전 배치.

치수 단위 주의:
  medicine.width/depth/height 는 cm 로 시드됨 → packing 시 mm 로 환산(×10).
  delivery_box.inner_* / wall_margin / item_gap 는 mm.
"""

import json
import threading
from datetime import datetime
from typing import Optional

from rectpack import newPacker

from .db_schema import get_conn
from . import mission_state as ms

_lock = threading.Lock()

CM_TO_MM = 10.0


def _now() -> str:
    return datetime.now().isoformat(timespec='seconds')


# ── 내부 조회 헬퍼 ────────────────────────────────────────────────────────────

def _resolve_box(c, mission_row, box_code: Optional[str]):
    """미션의 환자 병동 → delivery_box, 또는 box_code 명시 조회."""
    if box_code:
        return c.execute('SELECT * FROM delivery_box WHERE code=?',
                         (box_code,)).fetchone()

    if mission_row and mission_row['prescription_id']:
        row = c.execute(
            '''SELECT b.* FROM delivery_box b
               JOIN patient p   ON p.ward_id = b.ward_id
               JOIN prescription pr ON pr.patient_id = p.id
               WHERE pr.id = ?
               LIMIT 1''',
            (mission_row['prescription_id'],)
        ).fetchone()
        if row:
            return row

    # destination(병동명) fallback
    if mission_row and mission_row['destination']:
        return c.execute(
            '''SELECT b.* FROM delivery_box b
               JOIN ward w ON w.id = b.ward_id
               WHERE w.name = ? LIMIT 1''',
            (mission_row['destination'],)
        ).fetchone()
    return None


def _item_rects(c, prescription_id: int) -> list[dict]:
    """처방 품목을 (수량만큼 펼친) footprint 직사각형 리스트로 변환.

    그리퍼(전자석)는 **가장 넓은 면을 무조건 집어** 그 면으로 눕힌다. 따라서 박스 바닥에
    닿는 footprint = (width, depth, height) 중 **가장 큰 두 치수**(=가장 넓은 면), 남는
    가장 작은 치수가 적재 높이가 된다. (이전엔 width×depth 로 고정해 height 가 더 큰
    품목에서 footprint 를 과소평가 → 박스 밖으로 넘쳤다.)

    medicine 치수가 2개 미만이면 건너뜀.
    반환 각 원소: {rid, item_id, medicine_name, w_mm, h_mm, stack_h_mm}
    """
    items = c.execute(
        '''SELECT pi.id AS item_id, pi.medicine_name, pi.quantity,
                  m.width, m.depth, m.height
           FROM prescription_item pi
           LEFT JOIN medicine m ON m.id = pi.medicine_id
           WHERE pi.prescription_id = ?
           ORDER BY pi.sort_order, pi.id''',
        (prescription_id,)
    ).fetchall()

    rects: list[dict] = []
    skipped: list[str] = []
    rid = 0
    for it in items:
        # 가장 넓은 면을 집어 눕힘 → 가장 큰 두 치수가 바닥 footprint, 가장 작은 치수가 높이.
        dims = sorted(
            (float(it[k]) for k in ('width', 'depth', 'height')
             if it[k] is not None and float(it[k]) > 0),
            reverse=True,
        )
        if len(dims) < 2:
            skipped.append(it['medicine_name'])
            continue
        w_mm = dims[0] * CM_TO_MM          # 가장 넓은 면의 긴 변
        h_mm = dims[1] * CM_TO_MM          # 가장 넓은 면의 짧은 변
        stack_h_mm = dims[2] * CM_TO_MM if len(dims) >= 3 else None  # 적재 높이(최소 치수)
        for _ in range(max(1, int(it['quantity']))):
            rects.append({
                'rid': rid,
                'item_id': it['item_id'],
                'medicine_name': it['medicine_name'],
                'w_mm': w_mm,
                'h_mm': h_mm,
                'stack_h_mm': stack_h_mm,
            })
            rid += 1
    return rects, skipped


# ── 적층 헬퍼 ────────────────────────────────────────────────────────────────

def _stack_unplaced(unplaced_rects: list, layout: list, inner_h: float):
    """2D 패킹에서 못 넣은 품목을 기존 슬롯 위에 적층.

    각 (local_x, local_y) 위치의 현재 최상단 높이를 추적하며,
    높이 여유가 가장 많은 위치부터 채운다.

    반환: (새 적층 슬롯 목록, 높이 초과로 여전히 못 넣은 약품명 목록)
    슬롯 필드: slot_idx, medicine_name, local_x, local_y, w, h,
              rot_deg, z_offset_mm, stack_h_mm, placed
    """
    if not layout or not unplaced_rects:
        return [], [r['medicine_name'] for r in unplaced_rects]

    # (local_x, local_y) → 현재 적층 최상단 mm
    pos_top: dict = {}
    for s in layout:
        key = (s['local_x'], s['local_y'])
        pos_top[key] = s.get('stack_h_mm') or 0.0

    new_slots: list[dict] = []
    still_unplaced: list[str] = []
    next_idx = max(s['slot_idx'] for s in layout) + 1

    for r in unplaced_rects:
        best = min(pos_top, key=pos_top.get)
        z_off = pos_top[best]
        item_h = r.get('stack_h_mm') or 50.0

        if z_off + item_h > inner_h:
            still_unplaced.append(r['medicine_name'])
            continue

        new_slots.append({
            'slot_idx':      next_idx,
            'item_id':       r.get('item_id'),
            'medicine_name': r['medicine_name'],
            'local_x':       best[0],
            'local_y':       best[1],
            'w':             round(r['w_mm'], 2),
            'h':             round(r['h_mm'], 2),
            'rot_deg':       0.0,
            'z_offset_mm':   round(z_off, 2),
            'stack_h_mm':    round(item_h, 2),
            'placed':        False,
        })
        pos_top[best] += item_h
        next_idx += 1

    return new_slots, still_unplaced


# ── 핵심 ──────────────────────────────────────────────────────────────────────

def plan_for_mission(mission_code: Optional[str] = None,
                     box_code: Optional[str] = None) -> dict:
    """처방 품목 + 박스 치수로 적재 레이아웃 계산 후 pallet_plan 저장.

    반환:
      { ok, box_code, mission_code, placed_count, unplaced, skipped, layout }
    """
    mission = ms.get_mission()
    mc = mission_code or mission.get('mission_id')

    with _lock, get_conn() as c:
        mission_row = None
        mission_int_id = None
        if mc:
            mission_row = c.execute('SELECT * FROM mission WHERE code=?',
                                    (mc,)).fetchone()
            if mission_row:
                mission_int_id = mission_row['id']

        if not mission_row:
            return {'ok': False, 'error': f'미션 없음: {mc}', 'layout': []}

        box = _resolve_box(c, mission_row, box_code)
        if not box:
            return {'ok': False,
                    'error': '배송 박스 미해결 (병동/box_code 확인)',
                    'layout': []}

        if not mission_row['prescription_id']:
            return {'ok': False, 'error': '미션에 처방 미연결', 'layout': []}

        rects, skipped = _item_rects(c, mission_row['prescription_id'])
        if not rects:
            return {'ok': False, 'error': '배치할 품목 없음 (치수 미등록 가능)',
                    'skipped': skipped, 'layout': []}

        # 가용 영역 = 내부치수 - 양쪽 벽 여유. item_gap 은 각 rect 에 절반씩 패딩.
        margin = float(box['wall_margin'])
        gap = float(box['item_gap'])
        bin_w = float(box['inner_w']) - 2 * margin
        bin_d = float(box['inner_d']) - 2 * margin

        packer = newPacker(rotation=True)
        for r in rects:
            packer.add_rect(r['w_mm'] + gap, r['h_mm'] + gap, rid=r['rid'])
        packer.add_bin(bin_w, bin_d)
        packer.pack()

        rect_by_rid = {r['rid']: r for r in rects}
        layout: list[dict] = []
        placed_rids: set[int] = set()
        for slot_idx, (b_idx, x, y, w, h, rid) in enumerate(packer.rect_list()):
            r = rect_by_rid[rid]
            placed_rids.add(rid)
            # rectpack 이 90도 회전했는지 판별 (gap 보정 후 원치수와 비교)
            packed_w = w - gap
            packed_h = h - gap
            rotated = abs(packed_w - r['w_mm']) > 1e-3  # 폭이 바뀌면 회전된 것
            # footprint 중심 = 셀 좌하단 + (실치수/2), 벽 여유 오프셋 포함
            local_x = margin + x + packed_w / 2.0
            local_y = margin + y + packed_h / 2.0
            layout.append({
                'slot_idx':      slot_idx,
                'item_id':       r['item_id'],
                'medicine_name': r['medicine_name'],
                'local_x':       round(local_x, 2),
                'local_y':       round(local_y, 2),
                'w':             round(packed_w, 2),
                'h':             round(packed_h, 2),
                'rot_deg':       90.0 if rotated else 0.0,
                'z_offset_mm':   0.0,
                'stack_h_mm':    round(r.get('stack_h_mm') or 0.0, 2),
                'placed':        False,
            })

        unplaced_rects = [r for r in rects if r['rid'] not in placed_rids]
        stacked, truly_unplaced = _stack_unplaced(
            unplaced_rects, layout, float(box['inner_h']))
        layout.extend(stacked)
        unplaced = [{'item_id': r['item_id'], 'medicine_name': r['medicine_name']}
                    for r in unplaced_rects if r['medicine_name'] in truly_unplaced]

        # pallet_plan 갱신: 기존 미션 plan 교체
        c.execute('DELETE FROM pallet_plan WHERE mission_id=?', (mission_int_id,))
        c.execute(
            '''INSERT INTO pallet_plan
               (mission_id, box_id, layout_json, status, created_at, updated_at)
               VALUES (?, ?, ?, ?, ?, ?)''',
            (mission_int_id, box['id'],
             json.dumps(layout, ensure_ascii=False),
             'PLANNED', _now(), _now())
        )

    return {
        'ok': True,
        'mission_code': mc,
        'box_code': box['code'],
        'box': {
            'inner_w': box['inner_w'], 'inner_d': box['inner_d'],
            'inner_h': box['inner_h'], 'wall_margin': margin,
            'aruco_marker_id': box['aruco_marker_id'],
            'origin': [box['origin_x'], box['origin_y'], box['origin_z']],
        },
        'placed_count': len(layout),
        'unplaced': unplaced,
        'skipped': skipped,
        'layout': layout,
    }


def get_plan(mission_code: Optional[str] = None) -> dict:
    """저장된 적재 레이아웃 + 박스 메타 조회 (로봇 노드가 호출)."""
    mission = ms.get_mission()
    mc = mission_code or mission.get('mission_id')
    with _lock, get_conn() as c:
        row = c.execute(
            '''SELECT pp.*, b.code AS box_code, b.inner_w, b.inner_d, b.inner_h,
                      b.wall_margin, b.aruco_marker_id,
                      b.origin_x, b.origin_y, b.origin_z
               FROM pallet_plan pp
               JOIN mission m ON m.id = pp.mission_id
               LEFT JOIN delivery_box b ON b.id = pp.box_id
               WHERE m.code = ?
               ORDER BY pp.id DESC LIMIT 1''',
            (mc,)
        ).fetchone()
        if not row:
            return {'ok': False, 'error': f'plan 없음: {mc}', 'layout': []}
        return {
            'ok': True,
            'mission_code': mc,
            'box_code': row['box_code'],
            'status': row['status'],
            'box': {
                'inner_w': row['inner_w'], 'inner_d': row['inner_d'],
                'inner_h': row['inner_h'], 'wall_margin': row['wall_margin'],
                'aruco_marker_id': row['aruco_marker_id'],
                'origin': [row['origin_x'], row['origin_y'], row['origin_z']],
            },
            'layout': json.loads(row['layout_json'] or '[]'),
        }


def get_catalog() -> list:
    """medicine 테이블에서 치수가 등록된 약품 목록 반환 (단독 테스트용)."""
    with _lock, get_conn() as c:
        rows = c.execute(
            '''SELECT name, display_name, width, depth, height
               FROM medicine
               WHERE width IS NOT NULL AND depth IS NOT NULL AND height IS NOT NULL
               ORDER BY name'''
        ).fetchall()
    return [
        {'name': r['name'],
         'display_name': r['display_name'] or r['name'],
         'w_cm': r['width'], 'd_cm': r['depth'], 'h_cm': r['height']}
        for r in rows
    ]


def preview_plan(items_raw: list, box_code: str) -> dict:
    """DB 미션 없이 임의 품목 리스트로 배치 레이아웃 계산 (단독 테스트용).

    items_raw 원소: { name, qty, w_cm, d_cm, h_cm }
    """
    with _lock, get_conn() as c:
        box = c.execute('SELECT * FROM delivery_box WHERE code=?', (box_code,)).fetchone()
    if not box:
        return {'ok': False, 'error': f'알 수 없는 박스: {box_code}', 'layout': []}

    rects: list[dict] = []
    skipped: list[str] = []
    rid = 0
    for it in items_raw:
        raw_dims = [it.get('w_cm'), it.get('d_cm'), it.get('h_cm')]
        dims = sorted(
            (float(d) for d in raw_dims if d is not None and float(d) > 0),
            reverse=True,
        )
        if len(dims) < 2:
            skipped.append(it['name'])
            continue
        w_mm = dims[0] * CM_TO_MM
        h_mm = dims[1] * CM_TO_MM
        stack_h_mm = dims[2] * CM_TO_MM if len(dims) >= 3 else None
        for _ in range(max(1, int(it.get('qty', 1)))):
            rects.append({'rid': rid, 'medicine_name': it['name'],
                          'w_mm': w_mm, 'h_mm': h_mm, 'stack_h_mm': stack_h_mm})
            rid += 1

    if not rects:
        return {'ok': False, 'error': '배치할 품목 없음',
                'skipped': skipped, 'layout': []}

    margin = float(box['wall_margin'])
    gap = float(box['item_gap'])
    bin_w = float(box['inner_w']) - 2 * margin
    bin_d = float(box['inner_d']) - 2 * margin

    packer = newPacker(rotation=True)
    for r in rects:
        packer.add_rect(r['w_mm'] + gap, r['h_mm'] + gap, rid=r['rid'])
    packer.add_bin(bin_w, bin_d)
    packer.pack()

    by_rid = {r['rid']: r for r in rects}
    layout: list[dict] = []
    placed_rids: set[int] = set()
    for slot_idx, (_b, x, y, w, h, rid) in enumerate(packer.rect_list()):
        r = by_rid[rid]
        placed_rids.add(rid)
        packed_w = w - gap
        packed_h = h - gap
        rotated = abs(packed_w - r['w_mm']) > 1e-3
        layout.append({
            'slot_idx':      slot_idx,
            'medicine_name': r['medicine_name'],
            'local_x':       round(margin + x + packed_w / 2.0, 2),
            'local_y':       round(margin + y + packed_h / 2.0, 2),
            'w':             round(packed_w, 2),
            'h':             round(packed_h, 2),
            'rot_deg':       90.0 if rotated else 0.0,
            'z_offset_mm':   0.0,
            'stack_h_mm':    round(r.get('stack_h_mm') or 0.0, 2),
            'placed':        False,
        })

    unplaced_rects = [by_rid[r] for r in (set(by_rid) - placed_rids)]
    stacked, truly_unplaced = _stack_unplaced(
        unplaced_rects, layout, float(box['inner_h']))
    layout.extend(stacked)

    return {
        'ok': True,
        'box_code': box_code,
        'box': {
            'inner_w': box['inner_w'], 'inner_d': box['inner_d'],
            'inner_h': box['inner_h'], 'wall_margin': margin,
            'aruco_marker_id': box['aruco_marker_id'],
            'origin': [box['origin_x'], box['origin_y'], box['origin_z']],
        },
        'placed_count': len(layout),
        'unplaced': truly_unplaced,
        'skipped': skipped,
        'layout': layout,
    }


def mark_placed(mission_code: Optional[str], slot_idx: int) -> dict:
    """슬롯 배치 완료 표시. 모든 슬롯 완료 시 status='DONE'."""
    mission = ms.get_mission()
    mc = mission_code or mission.get('mission_id')
    with _lock, get_conn() as c:
        row = c.execute(
            '''SELECT pp.id, pp.layout_json FROM pallet_plan pp
               JOIN mission m ON m.id = pp.mission_id
               WHERE m.code=? ORDER BY pp.id DESC LIMIT 1''',
            (mc,)
        ).fetchone()
        if not row:
            return {'ok': False, 'error': f'plan 없음: {mc}'}
        layout = json.loads(row['layout_json'] or '[]')
        found = False
        for s in layout:
            if s['slot_idx'] == slot_idx:
                s['placed'] = True
                found = True
                break
        if not found:
            return {'ok': False, 'error': f'슬롯 없음: {slot_idx}'}
        all_done = all(s['placed'] for s in layout)
        c.execute(
            'UPDATE pallet_plan SET layout_json=?, status=?, updated_at=? WHERE id=?',
            (json.dumps(layout, ensure_ascii=False),
             'DONE' if all_done else 'IN_PROGRESS', _now(), row['id'])
        )
    return {'ok': True, 'slot_idx': slot_idx, 'all_done': all_done}
