# RoToSY Hospital Web — 팀원 세팅 가이드

병원 약품 물류 자동화 시스템의 웹 인터페이스입니다.  
Doosan E0509 로봇 암 + RealSense D455 + Florence-2를 모니터링·제어합니다.

---

## 전체 워크플로우

약품 배송이 시작되어 환자에게 도달하기까지의 7단계 흐름입니다.

```
┌─────────────┐     ① 처방전 발행      ┌─────────────┐
│   간호사     │ ──────────────────── ▶ │   서버 DB    │
│ /nurse       │   POST /api/presc...   │ (in-memory) │
└─────────────┘                         └──────┬──────┘
                                               │ ② 대기열 폴링 (5초)
                                               ▼
                                        ┌─────────────┐
                                        │    약사      │
                                        │ /pharmacist  │
                                        │             │
                                        │ OCR 신뢰도  │
                                        │ Vision AI   │
                                        │ 검증 후 승인 │
                                        └──────┬──────┘
                                               │ ③ 승인 + 약사 적재 확인
                                               ▼
┌─────────────┐   ④ 관리자 적재 확인   ┌─────────────┐
│   관리자     │ ─────────────────────▶│  미션 상태   │
│ /admin       │  POST /mission/confirm │  (서버)      │
│             │                         │             │
│  두 확인이   │ ◀─────────────────────│ can_dispatch │
│  모두 완료  │    WebSocket (10Hz)     │  = true      │
│  되면 출발  │                         └─────────────┘
│  버튼 활성  │
└──────┬──────┘
       │ ⑤ AMR 출발
       ▼
┌─────────────┐    ROS2 토픽           ┌─────────────┐
│     AMR      │ ──────────────────── ▶│  ros_bridge  │
│  (실물/시뮬) │  /amr/status          │  (서버 내부) │
│             │  /amr/battery          └──────┬──────┘
│             │  /amr/pose                    │ WebSocket 브로드캐스트
└─────────────┘                               ▼
                                       모든 페이지 동시 업데이트
                                       (관리자·약사·간호사·환자)
                                               │
                                               │ ⑥ AMR 도착 → 배너 표시
                                               ▼
                                        ┌─────────────┐
                                        │   간호사     │
                                        │ /nurse       │
                                        │             │
                                        │ 도착 배너   │
                                        │ 수령 확인   │
                                        └──────┬──────┘
                                               │ ⑦ 미션 완료
                                               ▼
                                        ┌─────────────┐
                                        │   환자       │
                                        │ /patient     │
                                        │             │
                                        │ 4단계 배송  │
                                        │ 상태바 완료 │
                                        └─────────────┘
```

---

## 페이지별 역할 & 데이터 흐름

### 간호사 `/nurse`

```
간호사 화면
├── [처방전 발행] 탭
│     └── 약품·수량·우선순위 입력
│           → POST /api/prescription
│               → 서버 in-memory 저장
│               → 약사 화면 대기열에 즉시 반영 (5초 폴링)
│
├── [내 처방전] 탭
│     └── GET /api/prescription (내 이름 필터)
│           → 상태: 대기 / 승인됨 / 배송 중 / 완료
│
├── [물류 현황] 탭
│     └── WebSocket → AMR 상태·배터리·위치 실시간 표시
│           → AMR status=ARRIVED 이면 상단 도착 배너 자동 표시
│
└── [약품 수령] 탭
      └── 수령 확인 버튼 → POST /api/system/mission/complete
```

### 약사 `/pharmacist`

```
약사 화면
├── [처방전 대기열] (좌측 패널)
│     └── GET /api/prescription 5초마다 자동 갱신
│           → 긴급 / 일반 / 정기 필터 칩
│           → OCR 신뢰도 낮으면 [!] 경고 표시
│
└── [처방전 상세] (우측 패널) — 클릭 시 표시
      ├── OCR 검증
      │     약품명·수량 파싱 결과 vs 처방 내용 일치 여부
      │     신뢰도 % 색상 표시 (≥90% 초록 / ≥75% 노랑 / <75% 빨강)
      │
      ├── Vision AI 신뢰도 바
      │     Florence-2 + ArUco 인식 결과 % 시각화
      │
      ├── [승인] 버튼 → POST /api/prescription/{id}/approve
      │     [반려] 버튼 → POST /api/prescription/{id}/reject
      │
      └── [약사 적재 확인] 버튼
            → POST /api/prescription/{id}/confirm_loading
            → 서버에서 mission.pharmacist_confirmed = true
            → 관리자 화면의 약사 확인 인디케이터가 즉시 초록으로 변경 (WebSocket)
```

### 관리자 `/` (admin)

```
관리자 화면 (8탭)
├── [카메라/비전] — 기본 탭
│     ├── RealSense D455 MJPEG 스트림 (GET /api/vision/stream)
│     ├── ArUco 마커 인식 결과 (GET /api/vision/detections, 2초 갱신)
│     └── 약품 서랍 2×3 위치 맵 (GET /api/vision/drawers)
│
├── [대시보드]
│     ├── 파이프라인 7단계 — WebSocket mission.status 에 따라 자동 이동
│     │     처방수신 → 로봇집기 → OCR검증 → 적재확인 → AMR출발 → 이동중 → 도착/수령
│     │
│     ├── 이중 적재 확인 패널
│     │     약사 확인  ○ ──▶ 약사가 /pharmacist 에서 버튼 클릭 시 초록 체크
│     │     관리자 확인 ○ ──▶ 이 화면에서 직접 클릭
│     │           POST /api/system/mission/confirm {actor: "admin"}
│     │
│     └── AMR 출발 버튼 — 두 확인 모두 완료 시에만 활성화
│           POST /api/amr/dispatch
│
├── [로봇 암]
│     ├── 실시간 TCP Pose / Joint Angles (WebSocket 10Hz)
│     ├── Servo ON/OFF, Home, 직접 교시 모드
│     └── Joint Jog Control (포인터 누르는 동안 계속 이동)
│           POST /api/robot/jog {joint_index, speed}  (RoToSY :8000 프록시)
│
├── [AMR 관제]
│     ├── 상태·배터리·위치 실시간 (WebSocket)
│     ├── 목적지 입력 → AMR 출발
│     └── 도어 강제 개방 / 닫힘
│
├── [온습도 센서]
│     ├── 현재 온도·습도 (WebSocket arduino 필드)
│     ├── 24시간 추이 차트 (GET /api/sensor/history, 1분 갱신)
│     └── 경보 이력 (GET /api/sensor/alerts)
│
└── [감사 로그]
      GET /api/system/audit — 모든 액션 기록 조회
```

### 환자 `/patient`

```
환자 화면
└── WebSocket mission.status 를 4단계로 단순화하여 표시
      IDLE / AWAITING_CONFIRM → [1] 처방 접수
      CONFIRMED / DISPATCHED  → [2] 약품 준비 중
      DELIVERING              → [3] 배송 중
      ARRIVED / COMPLETED     → [4] 도착
```

---

## 실시간 데이터 흐름 (WebSocket)

모든 페이지는 서버의 단일 WebSocket(`ws://{host}/ws`)에 연결합니다.  
서버가 **10Hz(100ms마다)** 로 아래 구조를 브로드캐스트합니다.

```
WebSocket 브로드캐스트 페이로드
│
├── robot          ← RoToSY(:8000) HTTP 폴링 결과
│     state, servo_on, is_moving, current_tcp, current_joints_deg ...
│
├── robot_online   ← RoToSY 연결 상태 (true/false)
│
├── amr            ← ROS2 /amr/status, /amr/battery, /amr/pose
│     status, battery, pose {x,y,theta}, destination, online
│
├── door           ← ROS2 /door/status
│     status (OPEN|CLOSED|ERROR), online
│
├── mission        ← 서버 in-memory 미션 상태
│     status, pharmacist_confirmed, admin_confirmed, can_dispatch ...
│
├── nodes          ← ROS2 노드 헬스 (3초/10초 기준)
│     arm_controller, amr_controller, vision_node → {status, last_seen}
│
├── plc            ← 미구현 (항상 DISCONNECTED)
│
└── arduino        ← ROS2 mock 또는 실제 아두이노 push
      temperature, humidity, status (NORMAL|WARNING), is_alert
```

각 페이지가 구독하는 필드:

| 페이지 | 구독 필드 |
|--------|-----------|
| 관리자 | 전체 (robot, amr, door, mission, nodes, arduino) |
| 약사 | mission (적재 확인 인디케이터 동기화) |
| 간호사 | amr (도착 배너), mission (파이프라인 표시) |
| 환자 | mission (4단계 상태 바) |

---

## 이중 적재 확인 (핵심 안전 장치)

AMR이 출발하려면 **약사 + 관리자 양쪽이 모두** 적재 확인을 해야 합니다.  
서버가 강제 검증하므로 UI를 우회해도 출발 불가입니다.

```
약사 화면                     서버                      관리자 화면
    │                           │                            │
    │ confirm_loading ─────────▶│ pharmacist_confirmed=true  │
    │                           │                            │
    │                           │◀──────────────── confirm ──│
    │                           │  admin_confirmed=true      │
    │                           │                            │
    │                           │ can_dispatch =             │
    │                           │   pharmacist AND admin     │
    │                           │   AND status=CONFIRMED     │
    │                           │                            │
    │◀──── WebSocket ───────────│──────── WebSocket ────────▶│
    │  약사 인디케이터 초록     │      AMR 출발 버튼 활성화   │
```

---

## 접속 URL 요약

| URL | 대상 |
|-----|------|
| `http://[서버IP]:8080/` | 관리자 대시보드 |
| `http://[서버IP]:8080/pharmacist` | 약사 패널 |
| `http://[서버IP]:8080/nurse` | 간호사 패널 |
| `http://[서버IP]:8080/patient` | 환자 현황 |

---

## 사전 요구사항

- Python 3.10 이상
- pip
- (선택) ROS2 Humble — 없으면 자동으로 mock 모드로 동작

```bash
python3 --version   # 3.10 이상인지 확인
```

---

## 빠른 시작 (ROS2 없는 환경 / 개발·데모)

ROS2가 없어도 웹 서버는 단독으로 실행됩니다. AMR·도어·센서는 모의 데이터로 표시됩니다.

```bash
cd /home/vboxuser/final_project/hospital_web

# 의존성 설치 + 서버 실행 (한 번에)
bash run.sh
```

또는 직접 실행:

```bash
cd /home/vboxuser/final_project/hospital_web
pip install -r backend/requirements.txt
python -m uvicorn backend.main:app --host 0.0.0.0 --port 8080 --reload
```

브라우저에서 `http://localhost:8080` 접속하면 관리자 대시보드가 열립니다.

---

## 실제 로봇 연동 시작 순서

시스템 전체를 연동할 때는 아래 순서대로 실행합니다.

### 터미널 1 — RoToSY (로봇 암 API, 포트 8000)

```bash
# ROS2 환경 소싱
source /opt/ros/humble/setup.bash
source /home/vboxuser/final_project/install/setup.bash  # 또는 실제 워크스페이스 경로

# 로봇 암 컨트롤러 실행
ros2 launch doosan_controller robot_controller.launch.py mode:=real

# 별도 터미널에서 웹 API 서버 실행 (RoToSY 측)
ros2 run web_interface web_server
```

### 터미널 2 — Hospital Web Gateway (포트 8080)

```bash
cd /home/vboxuser/final_project/hospital_web

# ROS2 환경 소싱 (AMR·도어 토픽 수신 필요 시)
source /opt/ros/humble/setup.bash
source /home/vboxuser/final_project/install/setup.bash

bash run.sh
```

> RoToSY(:8000)가 오프라인이어도 웹 서버는 실행됩니다. 관리자 화면 상단에 "OFFLINE"으로 표시될 뿐 나머지 기능은 정상 동작합니다.

---

## 팀별 연동 포인트

### 비전팀 (RealSense D455 + Florence-2)

카메라 스트림과 ArUco 인식 결과를 실제 데이터로 교체합니다.

**1. 카메라 MJPEG 스트림**

`backend/routers/vision.py`의 `_generate_placeholder_mjpeg()` 함수를 RealSense 실제 스트림으로 교체합니다.

```python
# 현재 (플레이스홀더)
def _generate_placeholder_mjpeg():
    ...

# 교체 예시 (pyrealsense2 사용)
import pyrealsense2 as rs
import cv2, numpy as np

pipeline = rs.pipeline()
pipeline.start(rs.config())

def _generate_realsense_mjpeg():
    while True:
        frames = pipeline.wait_for_frames()
        color = np.asanyarray(frames.get_color_frame().get_data())
        _, jpeg = cv2.imencode('.jpg', color)
        yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n'
```

**2. ArUco 인식 결과 푸시**

비전 노드에서 인식 결과를 웹 서버로 POST 합니다.

```bash
# 예시: curl로 직접 테스트
curl -X POST http://localhost:8080/api/vision/detections/update \
  -H "Content-Type: application/json" \
  -d '{"detections": {"drawer_001": {"confidence": 0.97, "aruco": 10}}}'
```

Python에서 호출:
```python
import httpx
httpx.post('http://localhost:8080/api/vision/detections/update', json={
    'detections': {
        'drawer_001': {'confidence': 0.97, 'aruco': 10, 'detected_at': '2026-06-04T14:00:00'}
    }
})
```

---

### AMR팀

ROS2 토픽 이름을 아래와 일치시키면 자동으로 연동됩니다. 이름이 다르면 `backend/ros_bridge.py`에서 수정합니다.

| 방향 | 토픽 | 타입 | 값 |
|------|------|------|----|
| AMR → 웹 | `/amr/status` | `std_msgs/String` | `IDLE` \| `DELIVERING` \| `ARRIVED` \| `RETURNING` \| `CHARGING` \| `ERROR` |
| AMR → 웹 | `/amr/battery` | `std_msgs/Float32` | 0~100 |
| AMR → 웹 | `/amr/pose` | `geometry_msgs/Pose2D` | x, y, theta |
| 도어 → 웹 | `/door/status` | `std_msgs/String` | `OPEN` \| `CLOSED` \| `ERROR` |

웹에서 출발 버튼을 누르면 아래 서비스가 호출됩니다.

| 웹 → AMR | 서비스 | 타입 |
|----------|--------|------|
| AMR 출발 | `/amr/dispatch` | `std_srvs/Trigger` |
| 미션 취소 | `/amr/cancel` | `std_srvs/Trigger` |
| 복귀 | `/amr/return_to_base` | `std_srvs/Trigger` |
| 도어 개방 | `/door/open` | `std_srvs/Trigger` |
| 도어 닫힘 | `/door/close` | `std_srvs/Trigger` |

---

### 로봇팀 (ArUco 마커 위치 재교정)

서랍 ID, ArUco 마커 번호, 약품 분류명은 `backend/drawer_metadata.py`에서 수정합니다.

```python
# backend/drawer_metadata.py 예시
DRAWERS = [
    {'id': 'drawer_001', 'label': 'A1', 'aruco': 10, 'drug_class': '진통제'},
    {'id': 'drawer_002', 'label': 'A2', 'aruco': 11, 'drug_class': '항생제'},
    ...
]
```

---

### 전기팀 (PLC / 컨베이어)

현재 PLC 탭은 "연동 대기 중" 플레이스홀더입니다. 연동 시:

1. `backend/routers/` 에 `plc.py` 라우터를 추가합니다.
2. `backend/main.py`에 라우터를 등록합니다.
3. `backend/ros_bridge.py`에 `/plc/status` 토픽 구독을 추가합니다.
4. `backend/static/admin.html`의 PLC 탭(`tab-plc`) UI를 구현합니다.

---

## Arduino 온습도 센서 연동

Arduino가 HTTP로 직접 데이터를 전송하는 방식입니다.

```bash
# Arduino → 웹 서버 POST (Arduino 코드 또는 테스트 curl)
curl -X POST http://[서버IP]:8080/api/sensor/reading \
  -H "Content-Type: application/json" \
  -d '{"temperature": 22.5, "humidity": 58.0, "sensor_id": "arduino_01"}'
```

임계값 기본값: 온도 15~25°C, 습도 40~70%. 변경 시:

```bash
curl -X PUT http://localhost:8080/api/sensor/thresholds \
  -H "Content-Type: application/json" \
  -d '{"temp_min": 15, "temp_max": 25, "humi_min": 40, "humi_max": 70}'
```

---

## API 빠른 테스트

서버가 실행 중인 상태에서 아래 주소로 API 목록 확인:

```
http://localhost:8080/docs
```

FastAPI 자동 문서 (Swagger UI)에서 모든 엔드포인트를 브라우저에서 바로 테스트할 수 있습니다.

---

## 파일 구조

```
hospital_web/
├── run.sh                      ← 서버 실행 스크립트
├── README.md
└── backend/
    ├── main.py                 ← FastAPI 앱 진입점 (포트 8080)
    ├── robot_proxy.py          ← RoToSY(:8000) HTTP 프록시
    ├── ros_bridge.py           ← AMR·도어·아두이노 ROS2 브릿지 (mock 자동 전환)
    ├── mission_state.py        ← 미션 상태 기계 (in-memory)
    ├── prescription_state.py   ← 처방전 CRUD (in-memory)
    ├── sensor_db.py            ← 온습도 SQLite DB
    ├── drawer_metadata.py      ← 서랍 위치 & ArUco 매핑 ← 로봇팀 수정
    ├── requirements.txt
    ├── routers/
    │   ├── robot.py            ← /api/robot/*
    │   ├── amr.py              ← /api/amr/*
    │   ├── system.py           ← /api/system/*
    │   ├── prescription.py     ← /api/prescription/*
    │   ├── sensor.py           ← /api/sensor/*
    │   └── vision.py           ← /api/vision/* ← 비전팀 수정
    └── static/
        ├── admin.html          ← 관리자 (8탭 대시보드)
        ├── pharmacist.html     ← 약사
        ├── nurse.html          ← 간호사
        └── patient.html        ← 환자
```

---

## 자주 묻는 문제

**Q. 서버 실행 시 `ModuleNotFoundError: No module named 'backend'` 오류**

`hospital_web/` 디렉토리에서 실행해야 합니다. `backend/` 안에서 실행하면 안 됩니다.

```bash
cd /home/vboxuser/final_project/hospital_web   # 여기서
python -m uvicorn backend.main:app ...
```

**Q. 카메라 스트림이 회색 화면만 나옴**

정상입니다. RealSense 카메라가 연결되지 않은 경우 플레이스홀더 이미지가 표시됩니다. 비전팀이 `routers/vision.py`를 수정하면 실제 영상이 나옵니다.

**Q. 로봇 암이 OFFLINE으로 표시됨**

RoToSY가 포트 8000에서 실행 중이지 않은 경우입니다. 터미널 1의 RoToSY 서버를 먼저 실행하세요. 웹 서버 자체는 정상 동작합니다.

**Q. ROS2 없이 실행하면 AMR·도어 상태가 움직이지 않음**

ROS2 토픽이 없으면 mock 모드로 동작합니다. 온습도는 30초마다 시뮬레이션 값이 업데이트됩니다. AMR·도어는 버튼을 눌러 수동으로 상태를 바꿀 수 있습니다.

**Q. 포트 8080이 이미 사용 중**

```bash
sudo lsof -i :8080           # 어떤 프로세스인지 확인
sudo kill -9 [PID]           # 종료 후 재시작
```
