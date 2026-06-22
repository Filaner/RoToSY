#!/bin/bash
# RoToSY (로봇 암 API) 가 localhost:8000 에서 먼저 실행 중이어야 합니다.
# 이 스크립트는 hospital_web/backend 를 포트 8080 으로 실행합니다.

cd "$(dirname "$0")"

# ROS2 환경 소싱 (필요 시)
# source /opt/ros/humble/setup.bash
# source /home/vboxuser/final_project/install/setup.bash

pip install -q -r backend/requirements.txt

python3 -m uvicorn backend.main:app --host 0.0.0.0 --port 8081 --reload
