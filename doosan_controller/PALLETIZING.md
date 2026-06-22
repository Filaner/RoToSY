# 약품 팔레타이징

약품을 **병동별 배송 박스**에 계획된 배치(packing)대로 적재한다. 픽업·정렬·OCR 검증은
기존 `motion_sequence` 를 그대로 쓰고, **배송 단계(14~15 사이)에 슬롯 좌표 계산·정렬을 삽입**해
"단일 고정 배치점" → "계산된 다중 슬롯 배치"로 만든다. 카메라 앞에서 잡힌 약품의 각도(yaw)와
박스의 각도를 vision 으로 측정해 그리퍼를 회전시키고 배치점을 보정한다.

구성 (좌표 알고리즘 / 로봇 제어 분리):

| 파일 | 역할 |
|------|------|
| `palletizing_planner.py` | **좌표 알고리즘 (self-contained, ROS 비의존)** — 품목→슬롯 패킹, 슬롯→base 좌표 변환. hospital_web 비의존. 단독 실행 가능: `python3 -m doosan_controller.palletizing_planner` |
| `motion_sequence.py` | `_deliver_palletizing()` — 마커/yaw 측정 + planner 로 좌표 산출 + 로봇 모션. `enable_palletizing` 파라미터로 켬 |
| `palletizing_sequence.py` | `enable_palletizing` 를 켠 **전용 진입점 노드**(얇은 래퍼). 독립 실행/관리용 |

켜는 법(둘 다 동일 동작): ① `ros2 run doosan_controller palletizing_sequence`,
② `ros2 run doosan_controller motion_sequence --ros-args -p enable_palletizing:=true`.

## 동작 개요

`motion_sequence` 의 1~13.5 단계(홈 → 서랍 픽업 → MoveC → 약품 픽 → 카메라 앞 정렬 →
OCR 검증)는 그대로다. OCR 통과 후 `_deliver_to_box()` 가 `enable_palletizing` 면
`_deliver_palletizing()` 로 분기해 다음을 수행한다(로봇이 카메라 앞에서 약품 파지 중):

| 단계 | 내용 |
|------|------|
| P1 | 잡힌 약품의 yaw 오차 `θ_item` 을 vision OBB(median) 로 측정 |
| P2 | 박스 상공 staging 관절 자세로 이동 |
| P3 | 적재 plan 의 다음 미배치 슬롯 조회 → 박스 각도 `θ_box`·원점 조회(빈 박스일 때 1회 측정·캐시, 이후 재사용) → 배치 좌표/자세 계산 후 슬롯 상공 XY 정렬(그리퍼 `rz` 회전) |
| P4 | Z 하강 |
| P5 | 전자석 OFF (배치) |
| P6 | Z 상승 → 슬롯 배치 완료 보고(`placed`) |
| P7 | MoveC 전 위치로 복귀 → 공통 서랍 닫기(19~)로 합류 |

OCR 불일치 → 원위치 복구(rollback) 시에는 베이스가 `_deliver_to_box` 를 호출하지 않으므로
슬롯이 소모되지 않는다.

### 물리 배치

- 약품 박스는 **250 × 190 mm** 흰색 박스. **긴 변(250)을 맞대고 2개**가 붙어 있다.
- 각 박스 **정중앙에 ArUco 마커**가 부착되어 박스의 각도(`θ_box`)·중앙 위치(`center`)
  기준이 된다. 매핑: **마커 4 = 1병동(병동A/BOX-A)**, **마커 3 = 2병동(병동B/BOX-B)**
  (카메라 이미지 기준 상단 박스=마커 3, 하단 박스=마커 4).
- 서랍(캐비닛) 마커 0~5 는 제거되어 서랍은 고정 좌표로 접근하며, 마커 0·1 은
  캘리브레이션 전용 — 박스 마커 3·4 와 런타임에서 충돌하지 않는다.

### 보정 수식

packing 슬롯 로컬좌표는 박스 좌하단 원점 기준이지만, **기준점(마커)이 박스 정중앙**이므로
`−inner/2` 로 중앙 보정한 뒤 회전·평행이동한다.

```
XY_place = center + Rz(θ_box) · [local_x − inner_w/2, local_y − inner_d/2]
rz_place = wrap(θ_box + slot_rot − θ_item)
place_z  = center_z + place_drop_mm
```

- `center` : 박스 정중앙(ArUco 마커)의 base 좌표. **빈 박스(plan 0개 배치)일 때 1회 측정해
  캐시**하고 이후 적재는 재사용한다 — 적재로 중앙 마커가 가려져도 재측정하지 않아 무영향.
  측정 성공값만 캐시하며, 마커 미검출 시 DB `origin_*`(=중앙 fallback)
- `θ_item` : 잡힌 약품이 그리퍼 안에서 틀어진 각도 → 역회전으로 슬롯 축에 정렬
- `θ_box`  : 목적지 박스가 base 축에서 회전한 각도 (마커 `rotation_matrix_base` 기준)
- `slot_rot` : packing 이 슬롯을 90° 회전 배치했는지 (0 / 90)

## 아키텍처 / 데이터 흐름

좌표 계산은 **doosan_controller 안에서 self-contained**로 처리한다(hospital_web 비의존).
비전(:8000)만 있으면 web_interface 단독으로 테스트된다.

```
[web_interface 비전 :8000]                 [doosan_controller]
 /camera/detections obb_angle_deg ───────→  motion_sequence._deliver_palletizing
 /camera/markers   rotation_matrix_base ─→    │  · _get_grasped_yaw (θ_item)
                                              │  · _get_box_pose   (θ_box·center, 마커 캐시)
                                              ▼
                                        palletizing_planner (순수 알고리즘)
                                          plan_layout / compute_placement → (x,y,z,rz)
```

### 의존 구성요소

1. **좌표 알고리즘** (`doosan_controller/palletizing_planner.py`) — self-contained, ROS 비의존
   - `BOXES` : 박스 치수/마커/origin 설정(BOX-A=병동A/마커4, BOX-B=병동B/마커3,
     `inner_w`=250 긴 변/마커 X축, `inner_d`=190 짧은 변). `CATALOG` : 약품 치수(cm).
   - `plan_layout(items, box)` : `rectpack` 2D bin-packing(회전 허용). footprint = **가장 넓은 면**
     (=치수 중 가장 큰 두 개; 그리퍼가 가장 넓은 면을 파지하므로).
   - `compute_placement(slot, box, θ_box, center, θ_item)` : 슬롯 → base 좌표/자세.
   - `PalletPlanner` : 박스 1개의 레이아웃 보유 + 슬롯 순차 제공(`take_next`/`mark_placed`).
   - 의존성: `rectpack` (`doosan_controller/package.xml` 의 exec_depend, lazy import).
   - **품목 소스(TODO 팀 연동)** : 현재 `_ensure_planner` 가 planner 기본 시나리오로 패킹.
     실미션 처방 품목 연동은 `motion_sequence._ensure_planner` 가 통합 지점.

2. **Vision OBB** (`web_interface/web_interface/vision_detector.py`)
   - 각 검출에 `obb_angle_deg`(±45° 정규화) 추가, N프레임 **median** 안정화 → `/camera/detections`.
   - `/camera/markers` : 박스 마커(3·4) `rotation_matrix_base`·`x_mm` 제공.

## 실행

선행: `arm_controller`, DSR 드라이버(`dsr01`), `web_interface`(:8000) 기동, 서보 ON.
(좌표는 self-contained 라 hospital_web 없이도 됨.) 풀스택 launch 토글로 켜는 게 가장 쉽다.

```bash
# A) 풀스택 launch — motion_sequence 끄고 palletizing 켜기 (서로 /motion/start 충돌 방지)
ros2 launch doosan_controller robot_controller.launch.py \
     mode:=real enable_motion_sequence:=false enable_palletizing:=true

# B) 노드만 단독 기동 (둘 중 하나, 동일 동작)
ros2 run doosan_controller palletizing_sequence
ros2 run doosan_controller motion_sequence --ros-args -p enable_palletizing:=true

# 드로어별 트리거 (Int32 = drawer_index 0~5)
ros2 topic pub -1 /motion/start std_msgs/msg/Int32 "{data: 0}"
```

드로어마다 약품 1종을 픽업·OCR 후 해당 약품의 슬롯에 배치한다. 여러 약품은 드로어별로
반복 트리거하면 레이아웃대로 박스가 누적 채워진다.

### CLI (단일 실행 / 스텝 모드)

```bash
ros2 run doosan_controller palletizing_sequence <drawer_index> [--step]
# 예: 0번 드로어를 스텝 모드(각 단계 수동 진행)로
ros2 run doosan_controller palletizing_sequence 0 --step

### CLI (단일 실행 / 스텝 모드)

```bash
ros2 run doosan_controller palletizing_sequence <drawer_index> [--step]
# 예: 0번 드로어를 스텝 모드(각 단계 수동 진행)로
ros2 run doosan_controller palletizing_sequence 0 --step
```

스텝 모드에서는 각 단계 전 `/motion/next_step` 으로 진행, `/motion/stop` 으로 중단한다.

## 파라미터

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `box_staging_joints` | `[31.04, 48.6, 38.63, 0.0, 92.77, -58.96]` | 박스 진입 전 안전 staging 관절 자세(deg) |
| `approach_clearance_mm` | `60.0` | 슬롯 상공 접근 높이(배치점 위 mm) |
| `place_drop_mm` | `20.0` | 박스 원점(바닥) 기준 배치 하강 깊이 mm |
| `carry_rx` / `carry_ry` | `90.0` / `-180.0` | 약품 캐리/배치 자세(rx, ry). `rz` 만 슬롯에 맞춰 회전 |
| `enable_orientation_correction` | `True` | `θ_item`/`θ_box` 보정 적용 여부(False 면 origin·rz 고정) |

```bash
ros2 run doosan_controller palletizing_sequence --ros-args \
  -p place_drop_mm:=15.0 -p approach_clearance_mm:=80.0
```

상수 `MAX_ITEM_YAW_CORRECTION_DEG`(기본 45°): 측정 yaw 가 이 값을 넘으면 오측정으로 보고
보정을 생략(0°)한다.

## 캘리브레이션 / 실측 필요 항목

실로봇 적용 전 다음을 실측·조정해야 한다.

- `delivery_box.origin_x/y/z` (박스 **정중앙** base 좌표, 마커 미검출 시 fallback). 정상 운용은
  박스 정중앙 ArUco 마커(BOX-A=4, BOX-B=3)로 자동 측정하므로, 마커가 카메라에 보이는지 우선 확인
- `box_staging_joints` (실제 박스 위치 기준 안전 상공 자세)
- `place_drop_mm` / `approach_clearance_mm` (박스 높이·약품 높이 고려)
- 카메라–TCP 픽셀→mm 환산 캘리브(`rotosy_calibration`) — vision 중심/각도 정확도에 직결

## 제약

- 이 패키지는 ROS 환경에서만 구동된다. 노드 로직(모션 좌표·속도, staging 자세)은
  실로봇 **스텝 모드** 검증을 거쳐야 한다.
- 전자석 평면 그립을 가정한다(가벼운 직사각형 약품/워터팩). 무거운 품목은 회전 중
  미끄러짐 가능성이 있어 별도 재파지 정책이 필요하다.
- packing 은 약품을 **가장 넓은 면**(=`width`/`depth`/`height` 중 가장 큰 두 치수)으로 눕혀
  배치하는 2D 직사각형 기준이다 — 그리퍼가 항상 가장 넓은 면을 집기 때문. 남는 가장 작은
  치수가 적재 높이가 된다. 높이 적층(다단)은 미지원.
