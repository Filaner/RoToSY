# RoToSY

처방에 따라 약품을 집고, OCR로 확인해 배송 박스에 적재하는 ROS2 병원 물류 프로젝트입니다.
로봇팔·PLC는 실장치 연결을 전제로 하며, AMR 배송은 Gazebo·Nav2 시뮬레이션입니다.

**[사용 가이드](https://filaner.github.io/RoToSY/)** · **[데모 영상](https://youtu.be/MdKExcUtkrw)**

## 주요 구성

- **인식·집기:** RealSense, ArUco, YOLO로 좌표를 구하고 Doosan E0509·전자석 그리퍼로 약품을 집습니다.
- **검수·적재:** Google Cloud Vision OCR로 약품명을 대조하고, PLC·컨베이어와 박스 적재를 제어합니다.
- **관제·배송:** FastAPI 웹에서 처방과 작업 상태를 관리하며, AMR 이동은 시뮬레이션으로 연결합니다.

## 문서

시스템 구조, 작업 순서, 설치·설정, package별 역할은 사용 가이드에 모았습니다.

- [시스템 연결](https://filaner.github.io/RoToSY/#architecture)
- [설치·실행](https://filaner.github.io/RoToSY/#installation) · [모델·장치 설정](https://filaner.github.io/RoToSY/#configuration)
- [화면 안내](https://filaner.github.io/RoToSY/#roles) · [저장소 구조](https://filaner.github.io/RoToSY/#repository)

## 실행 진입점

[로봇 통합 launch](doosan_controller/launch/robot_controller.launch.py) · [AMR simulation launch](mobile_simulation/launch/mobile_simulation.launch.py) · [웹 앱](hospital_web/backend/main.py)

실행에는 외부 Doosan driver, 모델 weights, 보정값, Cloud 인증이 필요합니다.
통합 launch는 실제 장치에 연결합니다. `mode:=virtual`도 PLC·카메라를 가상화하지 않습니다.

## License

루트 `LICENSE`는 없습니다. package manifest의 `Apache-2.0` 표기를 저장소 전체의 라이선스로 단정하지 않습니다.
