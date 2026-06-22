# RoToSY hospital_web DB 인수인계서

작성 기준일: 2026-06-22  
작성자: 선임 인수인계 (Claude)

---

## 1. 전체 구조 한눈에 보기

```
hospital_web/
├── backend/
│   ├── hospital.db        ← 유일한 DB 파일 (SQLite)
│   ├── db_schema.py       ← 테이블 정의 + 마이그레이션
│   ├── demo_create.py     ← 데모 시드 데이터 (여기서 초기값 세팅)
│   ├── sensor_db.py       ← 온습도 센서 전용 테이블 (별도 소유)
│   ├── main.py            ← FastAPI 앱, 포트 8080
│   └── routers/           ← API 라우터들
```

DB는 **`hospital.db` 하나**에 모든 게 들어 있습니다. SQLite라 별도 DB 서버 없이 파일만 있으면 됩니다.

---

## 2. DB 연결하는 법

```python
from backend.db_schema import get_conn

with get_conn() as c:
    rows = c.execute('SELECT * FROM medicine').fetchall()
    print(dict(rows[0]))
```

`get_conn()`은 항상 `PRAGMA foreign_keys = ON`과 `row_factory = sqlite3.Row`를 세팅해 줍니다.  
`sqlite3.Row`는 딕셔너리처럼 `row['column_name']`으로 접근할 수 있습니다.

터미널에서 바로 확인하고 싶을 때 (sqlite3 CLI 없는 환경에서):

```bash
python3 -c "
import sys; sys.path.insert(0, '.')
from backend.db_schema import get_conn
c = get_conn()
for r in c.execute('SELECT * FROM medicine').fetchall():
    print(dict(r))
"
```

---

## 3. 테이블 목록과 역할

### 핵심 테이블

| 테이블 | 한 줄 요약 |
|---|---|
| `ward` | 병동 정보 + AMR 이동 목표 좌표 |
| `staff` | 의사/간호사/약사/관리자 계정 |
| `patient` | 환자 정보, 차트번호로 식별 |
| `medicine` | 약품 카탈로그 (크기, 서랍 번호 포함) |
| `cabinet` | 약품 캐비닛(서랍장 전체 유닛) |
| `cabinet_slot` | 서랍 한 칸 = ArUco 마커 1개에 대응 |
| `prescription` | 처방전 (상태 기계의 핵심) |
| `prescription_item` | 처방전에 포함된 약품 목록 |
| `mission` | AMR 배송 미션 |
| `audit_log` | 누가 언제 무슨 행동을 했는지 기록 |
| `delivery_box` | 병동별 배송 박스 (팔레타이징 대상) |
| `pallet_plan` | 박스 안 약품 배치 레이아웃 |
| `ocr_scan` | 로봇 픽업 후 라벨 OCR 검증 기록 |

---

## 4. 테이블 상세

### ward (병동)

```sql
id, name, location, goal_x, goal_y, goal_theta
```

- `goal_x/y/theta`는 AMR이 Nav2로 이동할 때 쓰는 좌표 (단위: m, rad)
- 현재 시드값: 병동A, 병동B, 약재실, 간호스테이션

### medicine (약품)

```sql
id, name, display_name, width, height, depth, img_path, drawer_num
```

- `width/depth/height`: 단위 **cm**, 팔레타이징 레이아웃 계산에 사용
- `drawer_num`: ArUco 마커 번호 = 이 약품이 들어있는 **서랍 번호** (1~6)
  - 기존 코드에서는 `barcode_plane`이라고 잘못 명명되어 있었음. 2026-06-22에 `drawer_num`으로 rename.

현재 시드 약품 목록:

| drawer_num | 약품명 | 재고 |
|---|---|---|
| 1 | 메디폼 H뷰티 | 1 |
| 2 | MEDIPHARMAPLAN (수액팩) | 2 |
| 3 | 심미안정 | 1 |
| 4 | 新ビオフェルミンS錠 (일본 유산균) | 3 |
| 5 | 벤포벨S | 3 |
| 6 | 유한 비타민C | 1 |
| - | 애크논 | 0 |
| - | 임팩타민 | 0 |

### cabinet / cabinet_slot (서랍장)

```sql
-- cabinet
id, code, location, magnet_x/y/z, size_x/y/z

-- cabinet_slot
id, cabinet_id, medicine_id, code, row_idx, col_idx,
aruco_marker_id, label, pixel_x, pixel_y, max_capacity, current_stock
```

- `cabinet_slot` 한 행 = 물리적 서랍 한 칸
- `aruco_marker_id`: 서랍에 붙은 ArUco 마커 번호. **1번부터 시작** (0번 없음)
- `medicine_id`: 이 서랍에 어떤 약품이 들어있는지
- `current_stock`: 현재 재고 수량
- `pixel_x/y`: 카메라 이미지 상의 서랍 위치 (픽셀 좌표)

현재 서랍 배정:

| aruco | code | 약품 | 재고 |
|---|---|---|---|
| 1 | drawer_001 | 메디폼 H뷰티 | 1 |
| 2 | drawer_002 | MEDIPHARMAPLAN | 2 |
| 3 | drawer_003 | 심미안정 | 1 |
| 4 | drawer_004 | 新ビオフェルミンS錠 | 3 |
| 5 | drawer_005 | 벤포벨S | 3 |
| 6 | drawer_006 | 유한 비타민C | 1 |

**중요:** 로봇팔이 서랍을 열 때 `cabinet_slot.aruco_marker_id`로 서랍을 식별합니다.  
처방에서 약품 → `medicine_id` → `cabinet_slot.aruco_marker_id` 순서로 마커 번호를 찾습니다.  
(코드: `routers/prescription.py` `_build_marker_queue()`)

### prescription (처방전)

```sql
id, code, patient_id, doctor_id, priority, status,
pharmacist_note, reject_reason,
ocr_raw, ocr_confidence, ocr_parsed, vision_data,
delivery_requested, delivery_requested_at,
prescribed_at, created_at, updated_at
```

처방전 상태 기계:

```
pending
  → (약사 승인) → approved
  → (약사 반려) → rejected
  
approved
  → (간호사 배송 요청) → delivery_requested=1 유지, status=approved
  → (관리자 조제 시작) → awaiting_load_confirm
  
awaiting_load_confirm
  → (약사+관리자 적재 확인 완료) → [mission CONFIRMED]
  → (AMR 출발) → [mission DISPATCHED]
  → (배송 완료) → completed
```

- `priority`: `'emergency'` / `'general'` / `'scheduled'`
- `delivery_requested`: 간호사가 배송을 요청했는지 (0/1)

### prescription_item (처방 약품 목록)

```sql
id, prescription_id, medicine_id, medicine_name,
quantity, frequency, dosage, sort_order
```

- 처방전 1개 = prescription_item N개
- `medicine_name`은 텍스트로도 저장 (medicine 테이블이 없어도 표시 가능)
- `medicine_id`가 NULL이면 cabinet_slot 매핑 불가 → 로봇 피킹 불가

### mission (배송 미션)

```sql
id, code, prescription_id, destination, status,
pharmacist_confirmed, admin_confirmed,
created_at, confirmed_at, dispatched_at, arrived_at, completed_at
```

미션 상태:
```
IDLE → AWAITING_CONFIRM → CONFIRMED → DISPATCHED → ARRIVED → COMPLETED
```

- `pharmacist_confirmed` + `admin_confirmed` 둘 다 1이어야 AMR 출발 가능
- `destination`: 병동 이름 (ward.name)

### delivery_box (배송 박스)

```sql
id, ward_id, code, inner_w, inner_d, inner_h,
wall_margin, item_gap, aruco_marker_id,
origin_x, origin_y, origin_z
```

- 물리 배치: 250×190mm 흰색 박스 2개가 붙어 있음
- `aruco_marker_id`: 박스 정중앙 마커 (BOX-A=4, BOX-B=3). **서랍 마커(1~6)와 별개.**
- `origin_*`: 마커 미검출 시 fallback 좌표 (RealSense 실측값, 단위 mm)

### pallet_plan (팔레타이징 레이아웃)

```sql
id, mission_id, box_id, layout_json, status, created_at, updated_at
```

- `layout_json`: 각 약품의 박스 내 배치 좌표 JSON 배열
- `status`: `PLANNED` / `IN_PROGRESS` / `DONE` / `FAILED`

### audit_log (감사 로그)

```sql
id, mission_id, actor, action, detail, created_at
```

- 모든 주요 동작이 여기 기록됨
- `actor`: `'pharmacist'` / `'admin'` / `'nurse'` / `'demo'` / `'robot'`

### sensor_db 테이블 (별도 소유)

`hospital.db` 안에 있지만 `sensor_db.py`가 단독으로 관리합니다. `db_schema.py`는 건드리지 않습니다.

- `sensor_readings`: 온도/습도 시계열
- `drawer_sensors`: 각 서랍의 최신 센서값 (upsert 방식)

---

## 5. 데모 데이터 초기화

### 처음 시딩 (DB 없을 때)

```bash
cd /home/user/RoToSY/src/hospital_web
python3 -m backend.demo_create
```

### 완전 초기화 후 재시딩

```bash
python3 -m backend.demo_create --reset
```

`--reset`은 모든 도메인 row를 지우고 다시 박습니다. `sensor_readings`, `drawer_sensors`는 건드리지 않습니다.

### 시드 데이터를 바꾸고 싶을 때

`backend/demo_create.py` 상단의 상수들을 수정하면 됩니다:

- `WARDS`: 병동 좌표
- `MEDICINES`: 약품 목록, `drawer_num`으로 서랍 번호 지정
- `DRAWERS`: 서랍-약품 매핑, `stock`으로 초기 재고 지정
- `DELIVERY_BOXES`: 배송 박스 치수/마커
- `PRESCRIPTIONS`: 데모 처방전

---

## 6. 스키마 변경 방법

### 새 컬럼 추가

`db_schema.py`의 `CREATE TABLE IF NOT EXISTS` 안에 컬럼을 추가해도 **기존 DB에는 반영 안 됩니다**.  
기존 DB에 적용하려면 `init_schema()` 끝에 마이그레이션을 추가해야 합니다:

```python
# 예시 (db_schema.py init_schema() 마지막에 추가)
cols = {row[1] for row in c.execute("PRAGMA table_info(medicine)")}
if 'new_column' not in cols:
    c.execute('ALTER TABLE medicine ADD COLUMN new_column TEXT')
```

### 컬럼 이름 변경

SQLite 3.25+ 이상은 `RENAME COLUMN`을 지원합니다 (현재 환경: 3.37.2):

```python
if 'old_name' in cols and 'new_name' not in cols:
    c.execute('ALTER TABLE medicine RENAME COLUMN old_name TO new_name')
```

> **참고:** 이미 적용된 마이그레이션: `barcode_plane` → `drawer_num` (2026-06-22)

---

## 7. 서버 실행

```bash
cd /home/user/RoToSY/src/hospital_web
python3 -m uvicorn backend.main:app --host 0.0.0.0 --port 8080
```

실행 시 자동으로 `db_schema.init_schema()` 호출 → 테이블 없으면 생성, 마이그레이션 적용.

화면:
- `/` : 관리자 대시보드
- `/pharmacist` : 약사 화면
- `/nurse` : 간호사 화면
- `/patient` : 환자 화면

WebSocket `/ws` : 10Hz로 전체 시스템 상태 브로드캐스트.

---

## 8. 테이블 간 관계 (핵심 흐름)

```
patient ──┐
           ├──► prescription ──► prescription_item ──► medicine
staff ─────┘                                               │
                                                    cabinet_slot
                                                    (aruco_marker_id)
                                                           │
prescription ──► mission ──► audit_log              로봇팔 피킹
                    │
                    ▼
              delivery_box ──► pallet_plan
              (병동별 박스)     (레이아웃 JSON)
```

---

## 9. 자주 쓰는 쿼리

```python
# 현재 대기 처방전 목록
c.execute("SELECT * FROM prescription WHERE status='pending' ORDER BY created_at")

# 특정 서랍의 약품과 재고
c.execute("""
    SELECT cs.aruco_marker_id, cs.label, m.name, cs.current_stock
    FROM cabinet_slot cs
    LEFT JOIN medicine m ON cs.medicine_id = m.id
    ORDER BY cs.aruco_marker_id
""")

# 처방전 약품 → 서랍 마커 번호 찾기
c.execute("""
    SELECT pi.medicine_name, cs.aruco_marker_id
    FROM prescription_item pi
    JOIN medicine m ON pi.medicine_id = m.id
    JOIN cabinet_slot cs ON cs.medicine_id = m.id
    WHERE pi.prescription_id = ?
""", (prescription_id,))

# 최근 감사 로그
c.execute("SELECT * FROM audit_log ORDER BY created_at DESC LIMIT 20")
```

---

## 10. 주의사항

1. **DB 직접 수정 후 서버 재시작 불필요** — SQLite WAL 모드라 즉시 반영됩니다.
2. **`--reset` 하면 처방전/미션 다 날아감** — 시연 중엔 절대 실행 금지.
3. **서랍 마커 1~6, 배송 박스 마커 3~4 겹침** — 물리적으로 다른 위치이므로 로봇 비전에서 혼동 없지만, ArUco 마커 ID가 3, 4인 서랍과 박스가 공존한다는 점 인지해야 함. (`cabinet_slot` vs `delivery_box` 테이블이 분리되어 있어 DB 충돌은 없음)
4. **`sensor_db.py`는 건드리지 말 것** — 시리얼 포트 리더가 따로 돌면서 upsert함. `db_schema.py`의 `reset()`에서도 제외되어 있음.
5. **`medicine_id` NULL 처방 약품** — `prescription_item.medicine_id`가 NULL이면 로봇 피킹 큐에 들어가지 않음. 약품 이름을 medicine 테이블에 먼저 추가해야 함.
