"""
약품 서랍(카비닛) 위치 & ArUco 마커 정의.

2열 × 3행 = 6개 서랍
좌표: grid[row][col] → (픽셀X, 픽셀Y)
"""

DRAWER_GRID = {
    # 행(위아래), 열(좌우) → 서랍 ID, ArUco 마커 번호, 설명
    (0, 0): {'id': 'drawer_001', 'aruco': 10, 'label': '감기약', 'pos': (100, 100)},
    (0, 1): {'id': 'drawer_002', 'aruco': 11, 'label': '소화제', 'pos': (300, 100)},
    (1, 0): {'id': 'drawer_003', 'aruco': 20, 'label': '항생제', 'pos': (100, 250)},
    (1, 1): {'id': 'drawer_004', 'aruco': 21, 'label': '항염증', 'pos': (300, 250)},
    (2, 0): {'id': 'drawer_005', 'aruco': 30, 'label': '혈압약', 'pos': (100, 400)},
    (2, 1): {'id': 'drawer_006', 'aruco': 31, 'label': '당뇨약', 'pos': (300, 400)},
}

# 서랍 ID로 빠르게 조회
DRAWER_BY_ID = {v['id']: {**v, 'grid': k} for k, v in DRAWER_GRID.items()}
DRAWER_BY_ARUCO = {v['aruco']: {**v, 'grid': k, 'id': k[0] * 2 + k[1]} for k, v in DRAWER_GRID.items()}


def get_all_drawers() -> list[dict]:
    """모든 서랍 정보 반환 (행 우선 순서)."""
    return [
        {**info, 'grid_pos': grid_pos}
        for grid_pos, info in sorted(DRAWER_GRID.items())
    ]


def get_drawer_by_aruco(aruco_id: int) -> dict | None:
    """ArUco 마커 번호로 서랍 조회."""
    return DRAWER_BY_ARUCO.get(aruco_id)


def get_drawer_by_id(drawer_id: str) -> dict | None:
    """서랍 ID로 조회."""
    return DRAWER_BY_ID.get(drawer_id)
