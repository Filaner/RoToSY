"""
Drawer / cabinet slot 메타데이터 — DB-backed (was hardcoded module constants).

Public 시그니처는 기존 모듈과 동일:
  get_all_drawers()           — 모든 서랍 (행 우선)
  get_drawer_by_aruco(int)    — ArUco 마커 번호로 조회
  get_drawer_by_id(str)       — 'drawer_001' 형식 ID로 조회

반환 dict 모양도 기존과 동일:
  {'id': 'drawer_001', 'aruco': 10, 'label': '감기약',
   'pos': (px, py), 'grid_pos': (row, col)}
"""

from typing import Optional

from .db_schema import get_conn


def _row_to_dict(row) -> dict:
    return {
        'id':        row['code'],
        'aruco':     row['aruco_marker_id'],
        'label':     row['label'] or '',
        'pos':       (row['pixel_x'] or 0, row['pixel_y'] or 0),
        'grid_pos':  (row['row_idx'], row['col_idx']),
    }


def get_all_drawers() -> list:
    with get_conn() as c:
        rows = c.execute(
            'SELECT * FROM cabinet_slot ORDER BY row_idx, col_idx'
        ).fetchall()
    return [_row_to_dict(r) for r in rows]


def get_drawer_by_aruco(aruco_id: int) -> Optional[dict]:
    with get_conn() as c:
        row = c.execute(
            'SELECT * FROM cabinet_slot WHERE aruco_marker_id=?',
            (aruco_id,)
        ).fetchone()
    return _row_to_dict(row) if row else None


def get_drawer_by_id(drawer_id: str) -> Optional[dict]:
    with get_conn() as c:
        row = c.execute(
            'SELECT * FROM cabinet_slot WHERE code=?',
            (drawer_id,)
        ).fetchone()
    return _row_to_dict(row) if row else None
