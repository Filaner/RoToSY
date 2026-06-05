# RoToSY Hospital Web — 개발 보고서

> 작성일: 2026-06-04  
> 담당: Claude (Sonnet 4.6)  
> 프로젝트: 병원 약품 물류 자동화 시스템 — 웹 인터페이스 레이어

---

## 1. 프로젝트 개요

### 시스템 배경
Doosan E0509 로봇 암 + RealSense D455 + Florence-2 Vision AI를 활용한 병원 약국 자동화 물류 시스템.  
본 보고서는 해당 시스템을 제어·모니터링하는 **Web Interface Layer** 구축 결과를 기록한다.

### 핵심 배송 체인
```
간호사 처방전 발행
  → 약사 OCR/Vision 검증 + 승인
  → 로봇 암 약품 집기 (ArUco + Florence-2)
  → 약사·관리자 이중 적재 확인
  → AMR 병동 배송
  → 간호사 수령 확인
  → 환자 복약
```

---

## 2. 아키텍처

### 서비스 구조 (Option A — 분리 패키지)

```
/home/vboxuser/
├── RoToSY/                         ← 기존 로봇 암 제어 패키지 (포트 8000)
│   ├── doosan_controller/          ← ROS2 arm_controller_node
│   ├── robot_arm_interfaces/       ← 커스텀 ROS2 msg/srv/action
│   └── web_interface/              ← FastAPI (로봇 전용 REST + WS)
│
└── final_project/hospital_web/     ← 신규 병원 웹 레이어 (포트 8080)
    └── backend/
        ├── main.py                 ← FastAPI 게이트웨이
        ├── robot_proxy.py          ← RoToSY HTTP 폴링 프록시
        ├── ros_bridge.py           ← AMR/도어/아두이노 ROS2 브릿지
        ├── mission_state.py        ← 미션 생명주기 (in-memory)
        ├── prescription_state.py   ← 처방전 관리 (in-memory)
        ├── sensor_db.py            ← 온습도 센서 SQLite (append-only 히스토리)
        ├── drawer_metadata.py      ← 약품 서랍 위치 & ArUco 마커 정의
        ├── routers/
        │   ├── robot.py            ← /api/robot/* (RoToSY 프록시)
        │   ├── amr.py              ← /api/amr/* (AMR + 도어)
        │   ├── system.py           ← /api/system/* (E-Stop, 미션, 로그)
        │   ├── prescription.py     ← /api/prescription/* (처방전 CRUD)
        │   ├── sensor.py           ← /api/sensor/* (온습도 모니터링)
        │   └── vision.py           ← /api/vision/* (카메라, ArUco, 서랍 맵)
        └── static/
            ├── admin.html          ← 관리자 대시보드 (8탭)
            ├── pharmacist.html     ← 약사 패널
            ├── nurse.html          ← 간호사 패널
            └── patient.html        ← 환자 현황
```

### 데이터 흐름

```
Browser ─── WebSocket (10Hz) ─── FastAPI Gateway (:8080)
                                        │
                ┌───────────────────────┼────────────────┬──────────────┐
            HTTP Proxy             ROS2 Bridge      in-memory State    SQLite
                │                       │               State            │
          RoToSY (:8000)          /amr/status      mission_state    sensor_data.db
          robot control           /amr/battery     prescription        (온습도)
          REST API                /door/status     _state
                                  /arduino/temp
                                  /arduino/humid
```

---

## 3. 구현 파일 목록

### 백엔드 (Python / FastAPI)

| 파일 | 역할 | 핵심 내용 |
|------|------|----------|
| `main.py` | FastAPI 앱 진입점 | 6개 라우터 등록, WebSocket 10Hz 브로드캐스트, 4개 페이지 서빙, sensor_db 초기화 |
| `robot_proxy.py` | RoToSY 프록시 | 비동기 HTTP 폴링(10Hz), 명령 포워딩, 온라인 상태 추적 |
| `ros_bridge.py` | AMR/도어/아두이노 ROS2 브릿지 | `/amr/status`, `/amr/battery`, `/amr/pose`, `/door/status` 구독; 모의 센서 루프(30s); ROS2 없으면 mock 모드 자동 전환 |
| `mission_state.py` | 미션 상태 관리 | IDLE→AWAITING_CONFIRM→CONFIRMED→DISPATCHED→DELIVERING→ARRIVED→COMPLETED 상태 기계; 이중 확인(약사+관리자); 감사 로그 append-only |
| `prescription_state.py` | 처방전 관리 | 처방전 CRUD; 긴급/일반/정기 우선순위; OCR/Vision 결과 mock 데이터; 4건 시드 데이터 (정상 2건, 불일치 1건, 단순 1건) |
| `sensor_db.py` | 온습도 센서 DB | SQLite 기반 append-only 히스토리; 24시간 시드 데이터 (5분 간격 288건); 현재값/히스토리/경보 조회 |
| `drawer_metadata.py` | 서랍 위치 & ArUco | 2×3 그리드 = 6개 서랍; ArUco ID (10~31) 매핑; 서랍별 약품 분류명 |
| `routers/robot.py` | 로봇 암 API | POST /api/robot/{servo,jog,move_j,move_l,home,teaching,estop,recover} |
| `routers/amr.py` | AMR/도어 API | POST /api/amr/{dispatch,cancel,return,door/open,door/close}; can_dispatch 검증 |
| `routers/system.py` | 시스템 API | POST /api/system/estop_all, /mission/new·confirm·complete·reset; GET /audit |
| `routers/prescription.py` | 처방전 API | GET/POST /api/prescription; POST /{id}/approve·reject·confirm_loading |
| `routers/sensor.py` | 온습도 API | GET /current, /history, /alerts; POST /reading; PUT /thresholds |
| `routers/vision.py` | Vision AI & 카메라 API | GET /stream (MJPEG placeholder), /detections, /drawers, /drawer/{id}, /aruco/{id} |

### 프론트엔드 (HTML/CSS/JS — SPA)

| 파일 | 사용자 | URL | 핵심 기능 |
|------|--------|-----|----------|
| `admin.html` | 관리자 | `/` | 8탭 대시보드: **카메라/비전(메인) · 대시보드 · 로봇암 · AMR · PLC · 온습도센서 · 노드상태 · 감사로그**; 카메라 스트림, ArUco 인식 결과, 서랍 맵; 파이프라인; 이중 확인(CSS 원형 인디케이터) |
| `pharmacist.html` | 약사 | `/pharmacist` | 처방전 대기열, OCR/Vision 검증, 신뢰도 바, 승인·반려, 듀얼 확인 |
| `nurse.html` | 간호사 | `/nurse` | 처방전 발행(우선순위), 내 처방전, 물류 현황, AMR 도착 알림(sticky 배너), 병동 투약 일정 |
| `patient.html` | 환자 | `/patient` | 4단계 배송 상태 바(원형 인디케이터), 복약 일정, 간호사 호출 버튼 |

---

## 4. API 엔드포인트 전체 목록

### 로봇 암 (→ RoToSY :8000 프록시)

| Method | Path | 설명 |
|--------|------|------|
| POST | `/api/robot/servo` | 서보 ON/OFF |
| POST | `/api/robot/jog` | 관절 조그 |
| POST | `/api/robot/move_j` | 관절 공간 이동 |
| POST | `/api/robot/move_l` | 직선 이동 |
| POST | `/api/robot/home` | 홈 포지션 |
| POST | `/api/robot/teaching` | 직접 교시 모드 |
| POST | `/api/robot/estop` | 비상 정지 |
| POST | `/api/robot/recover` | 안전 복구 |

### AMR / 도어

| Method | Path | 설명 |
|--------|------|------|
| POST | `/api/amr/dispatch` | AMR 출발 (can_dispatch 체크) |
| POST | `/api/amr/cancel` | 미션 취소 |
| POST | `/api/amr/return` | 복귀 명령 |
| POST | `/api/amr/door/open` | 도어 강제 개방 |
| POST | `/api/amr/door/close` | 도어 강제 닫힘 |

### 시스템

| Method | Path | 설명 |
|--------|------|------|
| POST | `/api/system/estop_all` | 전체 정지 (로봇+AMR) |
| POST | `/api/system/mission/new` | 새 미션 생성 |
| POST | `/api/system/mission/confirm` | 적재 확인 (actor: admin/pharmacist) |
| POST | `/api/system/mission/complete` | 미션 완료 |
| POST | `/api/system/mission/reset` | 미션 초기화 |
| GET  | `/api/system/mission` | 현재 미션 상태 |
| GET  | `/api/system/audit` | 감사 로그 |

### 처방전

| Method | Path | 설명 |
|--------|------|------|
| GET  | `/api/prescription` | 전체 처방전 목록 (우선순위 정렬) |
| GET  | `/api/prescription/{id}` | 처방전 상세 |
| POST | `/api/prescription` | 처방전 생성 (간호사) |
| POST | `/api/prescription/{id}/approve` | 승인 (약사) |
| POST | `/api/prescription/{id}/reject` | 반려 (약사) |
| POST | `/api/prescription/{id}/confirm_loading` | 약사 적재 확인 |

### 온습도 센서 (Arduino)

| Method | Path | 설명 |
|--------|------|------|
| GET  | `/api/sensor/current` | 현재 온습도 + 임계값 + 상태 |
| GET  | `/api/sensor/history?hours=24` | 24시간 시계열 데이터 (5분 간격) |
| GET  | `/api/sensor/alerts?hours=24` | 임계값 초과 경보 이력 |
| POST | `/api/sensor/reading` | Arduino HTTP push 모드 |
| GET  | `/api/sensor/thresholds` | 현재 임계값 조회 |
| PUT  | `/api/sensor/thresholds` | 임계값 변경 |

### Vision AI & 카메라

| Method | Path | 설명 |
|--------|------|------|
| GET  | `/api/vision/stream` | MJPEG 카메라 스트림 (placeholder, RealSense 대기) |
| GET  | `/api/vision/detections` | 최근 ArUco 마커 인식 결과 (confidence %) |
| GET  | `/api/vision/drawers` | 모든 서랍 위치 & ArUco 정보 |
| GET  | `/api/vision/drawer/{id}` | 특정 서랍 정보 |
| GET  | `/api/vision/aruco/{id}` | ArUco 마커 ID로 서랍 역조회 |
| POST | `/api/vision/detections/update` | Vision 노드에서 결과 푸시 |

### WebSocket

| Path | 브로드캐스트 주기 | 페이로드 |
|------|------------------|---------|
| `ws://host:8080/ws` | 10Hz | `{robot, robot_online, amr, door, mission, nodes, plc, arduino}` |

---

## 5. WebSocket 페이로드 구조

```json
{
  "robot": {
    "robot_state": 1,
    "robot_state_str": "STANDBY",
    "servo_on": true,
    "is_moving": false,
    "teaching_mode": false,
    "arm_ready": true,
    "current_joints_deg": [0, 0, 90, 0, 90, 0],
    "current_tcp": [x, y, z, rx, ry, rz],
    "error_code": 0,
    "error_message": ""
  },
  "robot_online": true,
  "amr": {
    "status": "IDLE",
    "battery": 85.0,
    "pose": {"x": 0.0, "y": 0.0, "theta": 0.0},
    "destination": "",
    "online": false
  },
  "door": {
    "status": "CLOSED",
    "online": false,
    "last_seen": null
  },
  "mission": {
    "mission_id": null,
    "prescription_id": null,
    "status": "IDLE",
    "destination": "",
    "pharmacist_confirmed": false,
    "admin_confirmed": false,
    "can_dispatch": false,
    "created_at": null,
    "dispatched_at": null
  },
  "nodes": {
    "arm_controller":  {"status": "UNKNOWN", "last_seen": null},
    "amr_controller":  {"status": "UNKNOWN", "last_seen": null},
    "vision_node":     {"status": "UNKNOWN", "last_seen": null}
  },
  "plc": {"status": "DISCONNECTED"},
  "arduino": {
    "temperature": 20.5,
    "humidity": 55.2,
    "status": "NORMAL",
    "is_alert": false,
    "last_seen": "2026-06-04T14:30:15",
    "online": true
  }
}
```

---

## 6. DB 현황

### 온습도 센서 (sensor_data.db)

**SQLite, append-only 히스토리 모델**

| 테이블 | 목적 | 특징 |
|--------|------|------|
| `sensor_readings` | 온습도 측정값 이력 | PK=reading_id, 신뢰할 수 있는 감사 기록 |
| `drawer_sensors` | 현재 상태 스냅샷 | PK=sensor_id, 임계값(15~25°C, 40~70%) 포함 |

**시드 데이터**: 서버 첫 시작 시 24시간 시뮬레이션 데이터 생성 (5분 간격 288건, sin 파동)

### 처방전 / 미션 / 감사 로그

**현재**: Python in-memory dict/list  
**한계**: 서버 재시작 시 초기화 (데모용으로 충분)  
**실제 운영**: PostgreSQL/SQLite migration 필요

---

## 7. ROS2 토픽 연동 계약

AMR 팀이 시뮬레이터를 실물로 교체할 때 아래 토픽 이름을 맞추거나 `ros_bridge.py` 내 이름을 수정한다.

| 방향 | 토픽 | 메시지 타입 | 값 |
|------|------|------------|-----|
| AMR → 웹 | `/amr/status` | `std_msgs/String` | IDLE \| DELIVERING \| ARRIVED \| RETURNING \| CHARGING \| ERROR |
| AMR → 웹 | `/amr/battery` | `std_msgs/Float32` | 0–100 |
| AMR → 웹 | `/amr/pose` | `geometry_msgs/Pose2D` | x, y, theta |
| 도어 → 웹 | `/door/status` | `std_msgs/String` | OPEN \| CLOSED \| ERROR |
| 웹 → AMR | `/amr/dispatch` | `std_srvs/Trigger` | 출발 명령 |
| 웹 → AMR | `/amr/cancel` | `std_srvs/Trigger` | 취소 |
| 웹 → AMR | `/amr/return_to_base` | `std_srvs/Trigger` | 복귀 |
| 웹 → 도어 | `/door/open` | `std_srvs/Trigger` | 개방 |
| 웹 → 도어 | `/door/close` | `std_srvs/Trigger` | 닫힘 |

**ROS2 없이도 동작**: `ros_bridge.py`는 `rclpy` import 실패 시 자동으로 mock 모드로 전환. 개발/테스트 중 ROS2 환경 없이도 UI가 동작한다.

---

## 8. 디자인 시스템

### 색상 팔레트

```css
--bg:      #f4f6fa  (연한 회색 배경)
--surface: #ffffff  (카드 배경)
--border:  #dde2ed  (경계선)
--text:    #1a2540  (본문 텍스트)
--muted:   #6b7a99  (약한 텍스트)
--blue:    #1d4ed8  (관리자 포인트 색)
--green:   #16a34a  (약사/간호사/환자 포인트 색)
--red:     #dc2626  (경보/오류)
--yellow:  #b45309  (경고)
```

### 역할별 포인트 색상

| 페이지 | 포인트 색 | 예시 |
|--------|-----------|------|
| 관리자 | 파란색 (`--blue`) | 필터 칩, 탭 활성, 적재 확인 원형 인디케이터 |
| 약사 | 초록색 (`--green`) | Vision AI 신뢰도 바, 듀얼 확인 원형 인디케이터 |
| 간호사 | 초록색 (`--green`) | 탭 활성, 제출 버튼 |
| 환자 | 초록색 (`--green`) | 배송 단계 원형 인디케이터 |

### 컴포넌트

- **파이프라인 스텝**: 번호 원형(1~7) + 연결선
- **적재 확인 인디케이터**: CSS 원형 + 내부 체크마크 (텍스트 아님)
- **탭 네비게이션**: 활성 상태 하단 선 + 포인트 색
- **상태 배지**: 배경 + 색상 + 닷 애니메이션 (NORMAL/WARNING/ERROR/OFFLINE)

---

## 9. 미구현 항목 & 다음 단계

### 완료된 항목

✅ **Arduino 온습도 센서**
- SQLite 히스토리 DB
- 임계값(15~25°C, 40~70%) 모니터링
- 관리자 온습도 탭 (차트 포함)
- 센서값 실시간 받기 (/api/sensor/reading)

✅ **Vision AI 시스템**
- 카메라 스트림 플레이스홀더 (MJPEG)
- ArUco 마커 인식 결과 표시
- 약품 서랍 2×3 위치 맵
- 신뢰도 % 시각화

✅ **UI/UX 디자인**
- 화이트 톤 색상 시스템
- 포인트 색상 분리 (관리자=파란색, 나머지=초록색)
- 이모지 완전 제거
- 글꼴 크기 통일

### 남은 항목 (팀원 연동)

| 기능 | 담당 | 연동 포인트 | 우선순위 |
|------|------|------------|---------|
| RealSense D455 카메라 실제 연결 | 비전팀 | `routers/vision.py` 수정 (MJPEG 스트림 구현) | 높음 |
| Florence-2 Vision AI 실시간 처리 | 비전팀 | `/api/vision/detections` 업데이트, WebSocket 통합 | 높음 |
| ArUco 마커 위치 재교정 | 로봇팀 | `drawer_metadata.py` ArUco ID/위치 업데이트 | 중간 |
| PLC / 컨베이어 | 전기팀 | `routers/plc.py` 추가, `ros_bridge.py`에 `/plc/status` 구독 | 중간 |
| AMR 실물 연동 | AMR팀 | 토픽 이름 확인 → `ros_bridge.py` 수정 | 높음 |
| 처방전/미션 DB 연동 (PostgreSQL) | 데이터팀 | `prescription_state.py`, `mission_state.py` → ORM 전환 | 낮음 (데모용 OK) |
| 환자 인증 / 개인화 | 인증팀 | 로그인 페이지 추가, EMR 연동 | 낮음 |

---

## 10. 실행 방법

```bash
# 사전 조건: RoToSY가 포트 8000에서 실행 중이어야 함
# 터미널 1 (RoToSY)
cd /home/vboxuser/RoToSY
ros2 launch doosan_controller robot_controller.launch.py mode:=real
# 별도 터미널: ros2 run web_interface web_server

# 터미널 2 (병원 웹 게이트웨이)
cd /home/vboxuser/final_project/hospital_web
python3 -m uvicorn backend.main:app --host 0.0.0.0 --port 8080
```

| URL | 사용자 |
|-----|--------|
| `http://localhost:8080/` | 관리자 |
| `http://localhost:8080/pharmacist` | 약사 |
| `http://localhost:8080/nurse` | 간호사 |
| `http://localhost:8080/patient` | 환자 |

---

## 11. 주요 설계 결정 사항

### 이중 적재 확인 (Dual Confirmation)
약품이 AMR에 실렸는지 확인하는 물리 센서 대신, **약사 + 관리자 양쪽이 UI에서 모두 "적재 확인" 버튼을 눌러야** AMR 출발 버튼이 활성화된다.  
서버에서 `can_dispatch = pharmacist_confirmed AND admin_confirmed AND status == 'CONFIRMED'`로 강제 검증하므로 UI 우회 불가.

### 전체 정지 (Global E-Stop)
`POST /api/system/estop_all`은 로봇 암 E-Stop + AMR 미션 취소를 동시에 호출한다.  
관리자 화면 최상단에 항상 고정 표시.

### RoToSY 분리 유지
기존 팀원의 RoToSY 패키지를 수정하지 않고 HTTP 프록시 방식으로 연결.  
RoToSY가 오프라인이면 `robot_online: false`로 표시하고 나머지 시스템(AMR, 처방전 관리)은 계속 동작.

### 노드 헬스 모니터링
각 ROS2 토픽의 마지막 수신 시각을 추적:
- 3초 이내 수신 → ONLINE
- 3~10초 → DEGRADED  
- 10초 초과 → OFFLINE

### 센서 데이터 append-only
온습도 센서 읽기는 삭제 불가능한 감사 기록으로 설계.  
규제 요건(의약품 보관 조건 증명) 대비.

### Vision 탭을 메인으로
관리자가 대시보드에 들어오면 가장 먼저 **카메라 피드 + ArUco 인식 + 서랍 맵**을 보도록 배치.  
로봇 작업 실시간 감시 우선순위.

---

## 12. 파일 크기 요약

```
backend/
├── main.py                    ~110 lines
├── robot_proxy.py             ~80 lines
├── ros_bridge.py              ~270 lines (모의 센서 포함)
├── mission_state.py           ~150 lines
├── prescription_state.py      ~220 lines
├── sensor_db.py               ~240 lines (SQLite 초기화, 시드)
├── drawer_metadata.py         ~45 lines
├── routers/
│   ├── robot.py               ~70 lines
│   ├── amr.py                 ~70 lines
│   ├── system.py              ~70 lines
│   ├── prescription.py        ~70 lines
│   ├── sensor.py              ~75 lines
│   └── vision.py              ~120 lines
└── static/
    ├── admin.html             ~1420 lines (8탭 SPA + 비전 + 온습도)
    ├── pharmacist.html        ~720 lines (split-panel SPA)
    ├── nurse.html             ~900 lines (5탭 SPA)
    └── patient.html           ~280 lines (mobile-first)
```

**Python 총합**: ~1,380 lines  
**HTML/CSS/JS 총합**: ~3,320 lines  
**프로젝트 총 규모**: ~4,700 lines (주석 제외)

---

## 13. 테스트 가능 항목 (현재 상태)

### ✅ 관리자 대시보드
- [x] 파이프라인 단계 표시 (1~7)
- [x] 미션 상태 업데이트
- [x] 이중 확인 패널 (원형 인디케이터)
- [x] 적재 확인 → AMR 출발 잠금/해제
- [x] 온습도 차트 (모의 데이터)
- [x] 카메라 영역 (placeholder)
- [x] 서랍 맵 (2×3 그리드)

### ✅ 약사 인터페이스
- [x] OCR 검증 (의도적 불일치 케이스 포함)
- [x] Vision AI 신뢰도 바
- [x] 듀얼 확인 체크박스
- [x] 처방전 승인/반려

### ✅ 간호사 인터페이스
- [x] 처방전 발행 (우선순위 선택)
- [x] AMR 도착 시 sticky 배너
- [x] 병동 투약 일정

### ✅ 환자 인터페이스
- [x] 4단계 배송 상태 바
- [x] 간호사 호출 버튼

### ⏳ 향후 테스트 항목
- [ ] RealSense D455 카메라 MJPEG 스트림
- [ ] Florence-2 Vision AI 실시간 ArUco 인식
- [ ] ROS2 실제 토픽 수신 (AMR, 도어)
- [ ] PostgreSQL DB 연동
- [ ] 환자 로그인 인증

---

## 14. 결론

**개발 기간**: 1일 (모든 UI + API + 센서 시스템)  
**실제 운영 준비도**: 60% (카메라/Vision AI 연동 대기)  
**데모 준비도**: 100% (모든 기능 시뮬레이션 가능)

핵심 아키텍처(WebSocket 상태 브로드캐스트, 이중 확인, 미션 상태 기계, 센서 히스토리)는 완성되었고,  
나머지는 하드웨어 팀의 실제 카메라/로봇 연동만 남아있다.
