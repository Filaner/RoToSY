"""
팔레타이징 좌표 알고리즘 (self-contained, ROS 비의존).

처방 품목을 병동 배송 박스에 어떻게 깔지 계산하는 **순수 알고리즘**만 담는다.
로봇 제어(모션 실행)·카메라(마커/yaw 측정)는 호출 측(motion_sequence / palletizing_sequence)
이 담당하고, 여기서는 그쪽에 넘길 **적재 좌표만** 만든다.

분담:
  - 이 모듈(알고리즘)            : 품목 → 슬롯 레이아웃(패킹), 슬롯 → base 배치좌표 변환
  - motion_sequence(로봇 제어)  : 마커/yaw 측정값을 받아 여기 함수로 좌표를 구한 뒤 실제 이동

핵심 규칙:
  - 그리퍼는 **항상 가장 넓은 면을 파지** → 박스 바닥 footprint = (w,d,h) 중 가장 큰 두 치수,
    남는 가장 작은 치수가 적재 높이. (height 가 더 큰 품목에서 footprint 과소평가 → 박스밖 방지)
  - 마커는 박스 **정중앙**. 슬롯 로컬좌표(좌하단 원점)를 −inner/2 로 중앙 보정 후 회전·평행이동.

좌표계:
  박스 로컬 프레임. 원점 = 내부 가용영역 좌하단. local_x→inner_w(긴변), local_y→inner_d(짧은변). mm.
  slot.local_x/local_y = footprint '중심'. rot_deg = 0(원방향) / 90(90도 회전 배치).

단독 실행(알고리즘 점검):
  python3 -m doosan_controller.palletizing_planner          # 기본 시나리오 레이아웃 출력
"""

import math

CM_TO_MM = 10.0
DEFAULT_PLACE_DROP_MM = 20.0   # 마커(박스 윗면) 평면 기준 배치 하강 깊이

# ── 박스 / 약품 설정 (RealSense 실측 기준, demo 시드와 동일) ───────────────────
# inner_w = 긴 변(250, 마커 X축), inner_d = 짧은 변(190). origin = 마커 미검출 시 박스
# 정중앙 fallback base 좌표(mm). aruco = 박스 정중앙 마커 id.
BOXES: dict[str, dict] = {
    'BOX-A': {'ward': '병동A', 'inner_w': 250.0, 'inner_d': 190.0, 'inner_h': 120.0,
              'wall_margin': 10.0, 'item_gap': 8.0, 'aruco_marker_id': 4,
              'origin': (586.0, 328.0, -15.0)},
    'BOX-B': {'ward': '병동B', 'inner_w': 250.0, 'inner_d': 190.0, 'inner_h': 120.0,
              'wall_margin': 10.0, 'item_gap': 8.0, 'aruco_marker_id': 3,
              'origin': (368.0, 313.0, -23.0)},
}

# 약품 치수(cm): name -> (width, depth, height). footprint 계산용.
CATALOG: dict[str, tuple] = {
    '벤포벨S':      (5.7, 5.6, 11.0),
    '심미안정':     (7.8, 4.6, 10.7),
    '유한 비타민C': (10.7, 6.4, 7.7),
    '니뽄 유산균':  (5.7, 5.5, 10.2),
    '메디폼 H뷰티': (10.1, 1.6, 16.5),
    '애크논':       (12.5, 2.1, 3.3),
    '임팩타민':     (5.0, 5.0, 10.1),
}

# 알고리즘 단독 점검/‌web_interface 단독 테스트용 기본 시나리오.
DEFAULT_TEST_BOX = 'BOX-B'
DEFAULT_TEST_ITEMS = [('유한 비타민C', 1), ('니뽄 유산균', 2)]


# ── 순수 기하 헬퍼 ─────────────────────────────────────────────────────────────

def wrap_deg(angle: float) -> float:
    """각도를 (-180, 180] 으로 래핑."""
    a = (angle + 180.0) % 360.0 - 180.0
    return 180.0 if a == -180.0 else a


def box_local_to_base(center, theta_deg: float, lx: float, ly: float):
    """박스 중앙(마커) 상대좌표(lx, ly) → base XY.  base = center + Rz(theta)·[lx, ly]."""
    t = math.radians(theta_deg)
    c, s = math.cos(t), math.sin(t)
    return center[0] + lx * c - ly * s, center[1] + lx * s + ly * c


def widest_face_footprint(dims_cm) -> tuple | None:
    """(w,d,h) cm → (긴변_mm, 짧은변_mm, 적재높이_mm).  가장 넓은 면 = 가장 큰 두 치수."""
    d = sorted((float(x) for x in dims_cm if x is not None and float(x) > 0), reverse=True)
    if len(d) < 2:
        return None
    return d[0] * CM_TO_MM, d[1] * CM_TO_MM, (d[2] * CM_TO_MM if len(d) >= 3 else None)


# ── 패킹: 품목 → 슬롯 레이아웃 ─────────────────────────────────────────────────

def _resolve_items(items) -> list[dict]:
    """입력 품목을 footprint rect 로 펼친다.

    items 원소 형식(둘 다 허용):
      ('이름', 수량)                      → CATALOG 에서 치수 조회
      ('이름', 수량, (w_cm, d_cm, h_cm)) → 치수 직접 지정
    """
    rects: list[dict] = []
    skipped: list[str] = []
    rid = 0
    for it in items:
        name = it[0]
        qty = int(it[1]) if len(it) > 1 else 1
        dims = it[2] if len(it) > 2 else CATALOG.get(name)
        fp = widest_face_footprint(dims) if dims else None
        if fp is None:
            skipped.append(name)
            continue
        w_mm, h_mm, stack_h_mm = fp
        for _ in range(max(1, qty)):
            rects.append({'rid': rid, 'medicine_name': name,
                          'w_mm': w_mm, 'h_mm': h_mm, 'stack_h_mm': stack_h_mm})
            rid += 1
    return rects, skipped


def plan_layout(items, box: dict) -> dict:
    """품목 + 박스 → 적재 레이아웃(2D 빈패킹, 회전 허용).

    반환: { 'layout': [slot...], 'unplaced': [name...], 'skipped': [name...] }
      slot = {slot_idx, medicine_name, local_x, local_y, w, h, rot_deg, placed}
    """
    from rectpack import newPacker   # lazy: 미설치 환경서도 좌표변환 함수는 import 가능

    rects, skipped = _resolve_items(items)
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
        rotated = abs(packed_w - r['w_mm']) > 1e-3   # 폭이 바뀌면 90도 회전된 것
        layout.append({
            'slot_idx': slot_idx,
            'medicine_name': r['medicine_name'],
            'local_x': round(margin + x + packed_w / 2.0, 2),
            'local_y': round(margin + y + packed_h / 2.0, 2),
            'w': round(packed_w, 2),
            'h': round(packed_h, 2),
            'rot_deg': 90.0 if rotated else 0.0,
            'placed': False,
        })
    unplaced = [by_rid[r]['medicine_name'] for r in (set(by_rid) - placed_rids)]
    return {'layout': layout, 'unplaced': unplaced, 'skipped': skipped}


# ── 슬롯 → base 배치좌표 ───────────────────────────────────────────────────────

def compute_placement(slot: dict, box: dict, theta_box: float, center,
                      theta_item: float = 0.0,
                      place_drop_mm: float = DEFAULT_PLACE_DROP_MM,
                      enable_orientation: bool = True) -> tuple:
    """슬롯 → (base_x, base_y, place_z, rz_place) [mm, deg].

    XY = center + Rz(θ_box)·[local_x − inner_w/2, local_y − inner_d/2]
    rz = wrap(θ_box − θ_item)   ← rot_deg 는 XY 위치 계산에만 반영; 그리퍼는 회전하지 않는다.
    z  = center_z + place_drop_mm
    center = 박스 정중앙 마커 base 좌표(없으면 box['origin']).

    슬롯의 rot_deg(90° 회전 필요 여부)는 slot['needs_rotation'] 또는 slot['rot_deg'] != 0 으로
    호출 측에서 참조할 수 있다. 그리퍼 물리 회전은 호출 측 판단에 맡긴다.
    """
    tb = theta_box if enable_orientation else 0.0
    ti = theta_item if enable_orientation else 0.0
    lx = float(slot['local_x']) - float(box['inner_w']) / 2.0
    ly = float(slot['local_y']) - float(box['inner_d']) / 2.0
    bx, by = box_local_to_base(center, tb, lx, ly)
    rz = wrap_deg(tb - ti)   # rot_deg 제외: 그리퍼를 슬롯 방향으로 돌리지 않음
    return bx, by, float(center[2]) + place_drop_mm, rz


def next_slot(layout: list, last_match: str | None = None) -> dict | None:
    """다음 배치할 슬롯. last_match(약품명) 우선 매칭, 없으면 첫 미배치."""
    unplaced = [s for s in layout if not s.get('placed')]
    if not unplaced:
        return None
    if last_match:
        for s in unplaced:
            if (s.get('medicine_name') or '').strip() == last_match.strip():
                return s
    return unplaced[0]


# ── 상태 보유 래퍼 (노드에서 미션 단위로 사용) ─────────────────────────────────

class PalletPlanner:
    """한 박스에 대한 적재 레이아웃을 만들고, 슬롯을 하나씩 내어 주는 상태 보유 헬퍼.

    노드(motion_sequence/palletizing_sequence)는 약품 1개를 들 때마다:
      slot = planner.take_next(ocr_name)
      x,y,z,rz = planner.placement(slot, theta_box, center, theta_item)
      → (로봇 제어부가 x,y,z,rz 로 이동·배치)
      planner.mark_placed(slot)
    """

    def __init__(self, box_code: str = DEFAULT_TEST_BOX, items=None):
        if box_code not in BOXES:
            raise KeyError(f'알 수 없는 박스: {box_code} (가능: {list(BOXES)})')
        self.box_code = box_code
        self.box = BOXES[box_code]
        result = plan_layout(items if items is not None else DEFAULT_TEST_ITEMS, self.box)
        self.layout = result['layout']
        self.unplaced = result['unplaced']
        self.skipped = result['skipped']

    @classmethod
    def for_ward(cls, ward: str, items=None) -> 'PalletPlanner':
        code = next((c for c, b in BOXES.items() if b['ward'] == ward), DEFAULT_TEST_BOX)
        return cls(code, items)

    def take_next(self, medicine_name: str | None = None) -> dict | None:
        return next_slot(self.layout, medicine_name)

    def placement(self, slot: dict, theta_box: float, center,
                  theta_item: float = 0.0,
                  place_drop_mm: float = DEFAULT_PLACE_DROP_MM,
                  enable_orientation: bool = True) -> tuple:
        return compute_placement(slot, self.box, theta_box, center,
                                 theta_item, place_drop_mm, enable_orientation)

    def mark_placed(self, slot) -> bool:
        idx = slot['slot_idx'] if isinstance(slot, dict) else int(slot)
        for s in self.layout:
            if s['slot_idx'] == idx:
                s['placed'] = True
                return True
        return False

    @property
    def all_done(self) -> bool:
        return bool(self.layout) and all(s['placed'] for s in self.layout)

    @property
    def fallback_center(self) -> tuple:
        """마커 미검출 시 쓸 박스 정중앙 fallback 좌표."""
        return tuple(self.box['origin'])


# ── 단독 점검 ──────────────────────────────────────────────────────────────────

def _selftest():
    box = BOXES[DEFAULT_TEST_BOX]
    p = PalletPlanner(DEFAULT_TEST_BOX, DEFAULT_TEST_ITEMS)
    iw, id_ = box['inner_w'], box['inner_d']
    print(f'[palletizing_planner] {DEFAULT_TEST_BOX} inner {iw:.0f}×{id_:.0f}  '
          f'items={DEFAULT_TEST_ITEMS}')
    ok_all = True
    for s in p.layout:
        x0, x1 = s['local_x'] - s['w'] / 2, s['local_x'] + s['w'] / 2
        y0, y1 = s['local_y'] - s['h'] / 2, s['local_y'] + s['h'] / 2
        inside = x0 >= 0 and x1 <= iw and y0 >= 0 and y1 <= id_
        ok_all &= inside
        # 마커 미검출 fallback(θ_box=0, center=origin) 기준 예시 좌표
        bx, by, bz, rz = p.placement(s, 0.0, p.fallback_center)
        print(f"  #{s['slot_idx']} {s['medicine_name']:11} foot {s['w']:.0f}×{s['h']:.0f} "
              f"local=({s['local_x']:.0f},{s['local_y']:.0f}) rot{s['rot_deg']:.0f} "
              f"base=({bx:.0f},{by:.0f},{bz:.0f}) rz={rz:.0f} {'OK' if inside else 'OUT!'}")
    print(f"  unplaced={p.unplaced} skipped={p.skipped}  전슬롯 박스내부={'YES' if ok_all else 'NO'}")
    return ok_all


if __name__ == '__main__':
    _selftest()
