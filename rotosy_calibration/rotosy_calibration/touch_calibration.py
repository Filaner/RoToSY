#!/usr/bin/env python3
"""
ros2 run rotosy_calibration touch_calibration [marker_id...]

테이블 위의 ArUco 마커를 TCP로 직접 터치하여 카메라-로봇 좌표계 캘리브레이션.

원리 (포인트 대응 SVD):
  - 마커 i의 카메라 좌표 p_cam[i]  (카메라가 감지, camera frame)
  - 마커 i의 로봇 좌표  p_base[i] (TCP가 터치한 위치, robot base frame)
  - SVD로 T_base_camera 해를 구함:  p_base ≈ R @ p_cam + t

준비사항:
  1. DICT_4X4_50 ArUco 마커를 테이블에 4개 이상 배치
     (서로 비 동일선상, 가능하면 작업영역 전체에 분산)
  2. 카메라가 모든 마커를 동시에 볼 수 있는지 웹에서 확인
  3. TCP를 날카로운 포인터(침)로 설정하면 터치 정밀도 향상
  4. 마커에 터치할 때 카메라 시야를 가리지 않도록 옆에서 접근

사용 예:
  ros2 run rotosy_calibration touch_calibration 0 1 2 3
  ros2 run rotosy_calibration touch_calibration 0 1 2 3 4 5
"""

import json
import sys
import time
import urllib.request
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from robot_arm_interfaces.msg import RobotStatus
from robot_arm_interfaces.srv import Teaching

# ── 설정 ─────────────────────────────────────────────────────────────────────
CAMERA_API    = 'http://localhost:8000/camera/markers'
RESULT_PATHS  = [
    Path('/home/cheol/RoToSY_ws/src/rotosy_calibration/config/camera_extrinsic.yaml'),
    Path('/home/cheol/RoToSY_ws/install/rotosy_calibration/share/rotosy_calibration/config/camera_extrinsic.yaml'),
]
N_CAM_AVG = 20   # 카메라 위치 평균 샘플 수 (노이즈 감소)
N_TCP_AVG = 10   # TCP 위치 평균 샘플 수


def _line(char='─', n=62):
    print(char * n)


class TouchCalibrationNode(Node):

    def __init__(self):
        super().__init__('touch_calibration')
        self._tcp      = None   # [x, y, z, rx, ry, rz]  mm
        self._servo_on = False

        self._status_sub   = self.create_subscription(
            RobotStatus, '/arm/status', self._status_cb, 10
        )
        self._teaching_cli = self.create_client(Teaching, '/arm/teaching')

    # ── ROS callbacks ────────────────────────────────────────────────────────

    def _status_cb(self, msg: RobotStatus):
        self._tcp      = list(msg.current_tcp)
        self._servo_on = msg.servo_on

    def _spin_for(self, sec: float):
        deadline = time.time() + sec
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    # ── 데이터 취득 ──────────────────────────────────────────────────────────

    def get_tcp_m(self) -> np.ndarray | None:
        """현재 TCP 위치 N회 평균, 미터 단위."""
        samples = []
        for _ in range(N_TCP_AVG):
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._tcp and any(v != 0.0 for v in self._tcp[:3]):
                samples.append(self._tcp[:3])
            time.sleep(0.05)
        if not samples:
            return None
        return np.mean(samples, axis=0) / 1000.0   # mm → m

    def get_camera_marker(self, marker_id: int) -> np.ndarray | None:
        """카메라에서 특정 마커의 3D 위치 평균 (camera frame, 미터)."""
        readings = []
        for _ in range(N_CAM_AVG):
            try:
                with urllib.request.urlopen(CAMERA_API, timeout=2) as resp:
                    data = json.loads(resp.read())
                for m in data.get('markers', []):
                    if m['id'] == marker_id and m.get('x_cam_m') is not None:
                        readings.append([m['x_cam_m'], m['y_cam_m'], m['z_cam_m']])
                        break
            except Exception:
                pass
            time.sleep(0.07)

        if len(readings) < N_CAM_AVG // 2:
            return None
        arr = np.array(readings)
        # 이상치 제거: 중앙값 ± 2σ 밖 제거
        med = np.median(arr, axis=0)
        std = np.std(arr, axis=0) + 1e-9
        mask = np.all(np.abs(arr - med) < 2 * std, axis=1)
        return np.mean(arr[mask], axis=0) if mask.sum() > 0 else np.mean(arr, axis=0)

    # ── 교시 모드 ────────────────────────────────────────────────────────────

    def enable_teaching(self, enable: bool) -> bool:
        if not self._teaching_cli.wait_for_service(timeout_sec=3.0):
            print('[!] /arm/teaching 서비스 없음')
            return False
        req = Teaching.Request()
        req.enable = enable
        fut = self._teaching_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        resp = fut.result()
        return resp is not None and resp.success

    # ── 수학 ─────────────────────────────────────────────────────────────────

    @staticmethod
    def solve_svd(p_cam: np.ndarray, p_base: np.ndarray):
        """
        SVD로 T_base_camera 계산.
          p_cam  : (N,3)  camera frame  (m)
          p_base : (N,3)  robot base frame (m)
          반환   : R(3x3), t(3,)  →  p_base ≈ R @ p_cam + t
        """
        c_cam  = np.mean(p_cam,  axis=0)
        c_base = np.mean(p_base, axis=0)

        A = p_cam  - c_cam
        B = p_base - c_base

        H          = A.T @ B
        U, _, Vt   = np.linalg.svd(H)
        R          = Vt.T @ U.T

        # 반사(reflection) 보정
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T

        t = c_base - R @ c_cam
        return R, t

    @staticmethod
    def compute_residuals(p_cam, p_base, R, t) -> np.ndarray:
        """각 포인트의 예측-실측 거리 오차 (m)."""
        predicted = (R @ p_cam.T).T + t
        return np.linalg.norm(predicted - p_base, axis=1)

    @staticmethod
    def rotation_to_quaternion(R: np.ndarray) -> np.ndarray:
        """3x3 회전 행렬 → [qx, qy, qz, qw]."""
        trace = np.trace(R)
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            return np.array([
                (R[2, 1] - R[1, 2]) * s,
                (R[0, 2] - R[2, 0]) * s,
                (R[1, 0] - R[0, 1]) * s,
                0.25 / s,
            ])
        if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            return np.array([0.25 * s,
                             (R[0, 1] + R[1, 0]) / s,
                             (R[0, 2] + R[2, 0]) / s,
                             (R[2, 1] - R[1, 2]) / s])
        if R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            return np.array([(R[0, 1] + R[1, 0]) / s,
                             0.25 * s,
                             (R[1, 2] + R[2, 1]) / s,
                             (R[0, 2] - R[2, 0]) / s])
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        return np.array([(R[0, 2] + R[2, 0]) / s,
                         (R[1, 2] + R[2, 1]) / s,
                         0.25 * s,
                         (R[1, 0] - R[0, 1]) / s])

    # ── 저장 ─────────────────────────────────────────────────────────────────

    def save_yaml(self, R, t, res_m, n_points):
        q   = self.rotation_to_quaternion(R)
        rms = float(np.sqrt(np.mean(res_m ** 2)) * 1000)
        mx  = float(np.max(res_m) * 1000)
        content = (
            'camera_extrinsic:\n'
            '  ros__parameters:\n'
            '    base_frame: "base_link"\n'
            '    camera_frame: "camera_color_optical_frame"\n'
            f'    translation_xyz: [{t[0]:.8f}, {t[1]:.8f}, {t[2]:.8f}]\n'
            f'    rotation_quat_xyzw: [{q[0]:.8f}, {q[1]:.8f}, {q[2]:.8f}, {q[3]:.8f}]\n'
            '\n'
            'touch_calibration_result:\n'
            '  ros__parameters:\n'
            '    method: "point_correspondence_SVD"\n'
            f'    n_points: {n_points}\n'
            f'    rms_residual_mm: {rms:.3f}\n'
            f'    max_residual_mm: {mx:.3f}\n'
        )
        for path in RESULT_PATHS:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(content, encoding='utf-8')
            print(f'  저장: {path}')

    # ── 메인 플로우 ──────────────────────────────────────────────────────────

    def run(self, marker_ids: list):
        _line('═')
        print('  TCP 터치 카메라-로봇 캘리브레이션')
        _line('═')
        print(f'  대상 마커 ID : {marker_ids}  ({len(marker_ids)}개)')
        print(f'  권장 포인트  : 4개 이상, 작업영역 전체에 분산')
        print()

        # 로봇 상태 확인
        print('로봇 상태 수신 대기...')
        self._spin_for(2.0)
        if not self._servo_on:
            print('[!] 서보 OFF. 서보 ON 후 다시 실행하세요.')
            return

        # 직접 교시 모드
        print('직접 교시 모드 진입 중...')
        if not self.enable_teaching(True):
            print('[!] 교시 모드 진입 실패.')
            return
        print('✓ 교시 모드 활성화 — 로봇 암을 자유롭게 움직일 수 있습니다.\n')

        p_cam_list  = []
        p_base_list = []
        done_ids    = []

        try:
            for idx, mid in enumerate(marker_ids):
                _line()
                print(f'[{idx+1}/{len(marker_ids)}]  마커 ID {mid}')
                _line()

                # 카메라 감지 확인
                print('  카메라 감지 확인 중...')
                cam = self.get_camera_marker(mid)
                while cam is None:
                    print(f'  [!] 마커 ID {mid} 미감지.')
                    ans = input('      재시도=Enter  건너뜀=s : ').strip().lower()
                    if ans == 's':
                        break
                    cam = self.get_camera_marker(mid)
                if cam is None:
                    print('  건너뜁니다.\n')
                    continue

                print(f'  카메라 감지 위치 (camera frame):')
                print(f'    X={cam[0]*1000:7.1f}  Y={cam[1]*1000:7.1f}  Z={cam[2]*1000:7.1f}  mm')
                print()
                print(f'  ▶  TCP를 마커 ID {mid} 중심에 정확히 위치시키세요.')
                print(f'     (카메라 시야를 가리지 않도록 옆에서 접근)')
                input('     이동 완료 → Enter ')

                # TCP 기록
                print('  TCP 위치 기록 중...')
                tcp = self.get_tcp_m()
                if tcp is None:
                    print('  [!] TCP 읽기 실패. 건너뜁니다.\n')
                    continue

                # 카메라 재측정 (정지 상태에서)
                print('  카메라 위치 재측정 중...')
                cam = self.get_camera_marker(mid)
                if cam is None:
                    print('  [!] 카메라 재측정 실패. 건너뜁니다.\n')
                    continue

                p_cam_list.append(cam)
                p_base_list.append(tcp)
                done_ids.append(mid)

                print(f'  ✓ 기록 완료')
                print(f'    TCP   (base)  : X={tcp[0]*1000:7.1f}  Y={tcp[1]*1000:7.1f}  Z={tcp[2]*1000:7.1f}  mm')
                print(f'    카메라(cam)   : X={cam[0]*1000:7.1f}  Y={cam[1]*1000:7.1f}  Z={cam[2]*1000:7.1f}  mm')
                print()

        finally:
            print('직접 교시 모드 해제 중...')
            self.enable_teaching(False)
            print('✓ 교시 모드 해제.\n')

        n = len(p_cam_list)
        if n < 4:
            print(f'[!] 유효 포인트 {n}개 — 최소 4개 필요. 종료.')
            return

        # ── SVD 계산 ─────────────────────────────────────────────────────────
        _line('═')
        print(f'  SVD 캘리브레이션 계산  ({n}개 포인트)')
        _line('═')

        p_cam  = np.array(p_cam_list)
        p_base = np.array(p_base_list)
        R, t   = self.solve_svd(p_cam, p_base)
        res    = self.compute_residuals(p_cam, p_base, R, t)

        print('\n잔차 (각 마커의 예측-실측 오차):')
        for mid, r in zip(done_ids, res):
            flag = '  ← 높음! 재측정 권장' if r * 1000 > 5.0 else ''
            print(f'  ID {mid:2d} : {r*1000:5.2f} mm{flag}')

        rms_mm = np.sqrt(np.mean(res ** 2)) * 1000
        max_mm = np.max(res) * 1000
        print(f'\n  RMS 잔차 : {rms_mm:.2f} mm')
        print(f'  최대 잔차 : {max_mm:.2f} mm')

        if rms_mm <= 2.0:
            print('  ✓ 매우 좋음 (< 2mm)')
        elif rms_mm <= 5.0:
            print('  △ 양호 (< 5mm)')
        else:
            print('  ✗ 불량 (> 5mm) — 잔차가 높은 포인트를 재측정하세요.')

        # ── 이상치 제거 옵션 ─────────────────────────────────────────────────
        if max_mm > 5.0:
            print()
            ans = input('잔차 > 5mm 포인트를 제거하고 재계산할까요? (y/n): ').strip().lower()
            if ans == 'y':
                mask = res * 1000 <= 5.0
                if mask.sum() >= 4:
                    R, t = self.solve_svd(p_cam[mask], p_base[mask])
                    res  = self.compute_residuals(p_cam[mask], p_base[mask], R, t)
                    removed = [done_ids[i] for i, m in enumerate(mask) if not m]
                    print(f'  제거된 마커: {removed}')
                    print(f'  재계산 RMS: {np.sqrt(np.mean(res**2))*1000:.2f} mm')
                    p_cam, p_base = p_cam[mask], p_base[mask]
                    done_ids = [i for i, m in zip(done_ids, mask) if m]
                    n = mask.sum()
                else:
                    print('  [!] 제거 후 포인트 부족. 원본 결과 유지.')

        # ── 저장 ─────────────────────────────────────────────────────────────
        print()
        ans = input('결과를 저장하시겠습니까? (y/n): ').strip().lower()
        if ans == 'y':
            self.save_yaml(R, t, res, n)
            print('\n✓ 캘리브레이션 완료!')
            print('  웹서버가 실행 중이면 재시작해야 새 캘리브레이션이 적용됩니다.')
        else:
            print('저장 취소.')


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    marker_ids = []
    for arg in sys.argv[1:]:
        if arg == '--ros-args':
            break
        try:
            marker_ids.append(int(arg))
        except ValueError:
            continue

    if not marker_ids:
        print(__doc__)
        print('[!] 마커 ID를 지정하세요.  예: touch_calibration 0 1 2 3')
        return

    if len(marker_ids) < 4:
        print(f'[경고] 마커 {len(marker_ids)}개 — 4개 이상 권장합니다.')
        ans = input('계속하시겠습니까? (y/n): ').strip().lower()
        if ans != 'y':
            return

    rclpy.init(args=args)
    node = TouchCalibrationNode()
    try:
        node.run(marker_ids)
    except (KeyboardInterrupt, ExternalShutdownException):
        print('\n중단됨.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
