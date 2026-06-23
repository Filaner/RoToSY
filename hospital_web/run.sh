#!/bin/bash
# hospital_web/backend 를 포트 8080 으로 실행합니다.
# 로봇 팔 제어는 web_interface(:8000) 없이 ROS2 bridge로 직접 연결됩니다.

cd "$(dirname "$0")"

# ROS2 환경 소싱 (필요 시)
# source /opt/ros/humble/setup.bash
# source /home/vboxuser/final_project/install/setup.bash

pip install -q -r backend/requirements.txt

python3 -m uvicorn backend.main:app --host 0.0.0.0 --port 8080 --reload
