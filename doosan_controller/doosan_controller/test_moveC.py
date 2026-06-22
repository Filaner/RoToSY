"""
ros2 run doosan_controller test_moveC [radius_mm] [direction]

현재 TCP 위치를 구의 정점으로 하여 구의 겉면을 따라 90° 호 이동.

  구 중심  = 현재 TCP에서 -Z 방향으로 radius_mm
  Via (45°): YZ 평면 상 45° 지점 — TCP가 구 중심을 향하는 방향 유지
  End (90°): YZ 평면 상 90° 지점 — Y축으로 radius_mm 이동, Z는 -radius_mm

방향 공식: Rx=-90, Ry = 180 - direction * angle_deg, Rz=0
  → TCP 공구 축이 항상 구 중심을 향함

파라미터:
  radius_mm  : 반지름 (mm, 기본값 100).
  direction  :  1 → 구 중심 = 현재 TCP 아래(-Z),  +Y 방향 호, Z 감소 (전진)
               -1 → 구 중심 = 현재 TCP의 -Y,       -Y 방향 호, Z 증가 (복귀)
                2 → 원 중심 = 현재 TCP에서 +Y로 r,  진짜 원호, Y·Z 증가 (피킹 후 후퇴)
                3 → 플랜지 기준 Y 증가·Z 감소 비대칭 호 (하강, 실측 예시 일반화)
                4 → MoveL 단일 상대이동 (위치+자세, 원호 없음)
               * +1 끝 위치에서 -1을 실행하면 원래 위치로 정확히 복귀함

  direction=2 (피킹 후 후퇴):
    현재 TCP가 Z축 정렬(Rx=90,Ry≈180,Rz=0, 그리퍼가 아래를 향함 — 실제
    motion_sequence.py 피킹 직후 자세) 상태라고 보고, +Y 방향으로 r만큼 떨어진
    원 중심을 기준으로 진짜 원호(단일 반지름)를 따라 이동하며 자세를 Ry+90°만큼
    바꾼다 (예: 180 → -90, 코드베이스 전체에서 쓰는 "Y축 정렬" 포즈 (90,-90,0)과 일치).
    원 중심 C = (x0, y0+r, z0).  P(θ) = (x0, y0 + r·(1-cosθ), z0 + r·sinθ),
    θ: 0°→90° → Y, Z 모두 단조 증가. θ=90°에서 TCP의 y값이 정확히 원 중심의
    y값(y0+r)과 같아진다 — "TCP에서 +Y로 r 떨어진 곳이 원 중심"이라는 정의 그대로.
    기존 direction=1과 같은 방식(TCP/플랜지 좌표 자체가 원호를 그림, 그리퍼 길이
    보정 없음)이고, 원 중심 위치만 다르다 (-Z 대신 +Y).
    자세: Rx=90, Rz=0 고정 (motion_sequence.py/test_movel.py 등 코드베이스
    전체와 동일한 컨벤션), Ry만 실제 측정된 시작값에서 +90° 보낸다.
    MoveCircle 서비스(2점) 한 번 호출은 실제로 원호를 그리지 않을 수 있어,
    motion_sequence.py의 _move_c와 동일하게 N분할 MoveL로 경로를 직접 따라간다.

    원호가 끝난 뒤에는 추가로 직선 MoveL 한 구간을 이어붙여, 동작 전체의
    최종 결과가 시작점 기준 정확히 (y: RETREAT_FINAL_Y_OFFSET_MM,
    z: RETREAT_FINAL_Z_OFFSET_MM) 상대이동이 되도록 한다 (원호 자체의
    이동량 +r,+r과는 무관). 이 추가 구간도 원호 구간들과 동일한 블렌드 계산
    로직(인접 구간 길이 기준 ARC_BLEND_FRACTION 제한)에 포함시켜, 원호에서
    추가 이동으로 넘어가는 지점이 멈췄다가 다시 출발하지 않고 자연스럽게
    이어지게 한다.

  direction=3 (하강):
    사용자가 실제 로봇을 조그해 찍은 예시를 일반화한 동작.
      시작 (398.11, -182.47, 606.76, 90, 180, 0)
      끝   (398.11,    0.00, 506.76, 90, -90, 0)
      → Δy≈+182.47, Δz≈-100 (radius 기본값 100mm·y_distance 기본값 182.47mm이
        이 예시와 그대로 일치하므로 인자 없이 `test_moveC 100 3`만 호출해도 재현됨)
    플랜지 기준 P(θ) = (x0, y0 + y_distance·(1-cosθ), z0 - radius·sinθ),
    θ: 0°→90° → Y 증가·Z 감소, 초반 Z 위주·후반 Y 위주 비대칭(뚱뚱한) 호.
    자세는 direction=2와 동일한 공식(Rx=90,Rz=0 고정, Ry는 실제 측정된
    시작값에서 +90°). direction=2처럼 그리퍼 끝 보정은 적용하지 않는다 —
    사용자가 준 예시 자체가 플랜지 좌표이고 "이 모양대로 움직여라"는 의도였기
    때문에, 보정 없이 플랜지 좌표를 그대로 일반화한다.

  direction=4 (단일 MoveL 상대이동):
    원호 없이 MoveL 한 번만 호출하되, 위치(x,y,z)뿐 아니라 자세(rx,ry,rz)까지
    전부 현재 TCP 기준 상대값으로 더해서 이동한다 (MoveL.relative=True).
    이동값은 RELATIVE_MOVE = [0, 180, 100, 0, -270, 0] (mm/deg)으로 고정.
    radius_mm 파라미터는 이 모드에서는 사용하지 않는다.

사용 예:
  ros2 run doosan_controller test_moveC              # 100mm, +Y
  ros2 run doosan_controller test_moveC 150          # 150mm, +Y
  ros2 run doosan_controller test_moveC 100 -1       # 100mm, -Y
  ros2 run doosan_controller test_moveC 150 -1       # 150mm, -Y
  ros2 run doosan_controller test_moveC 100 2        # r=100mm 진짜 원호, 피킹 후 후퇴
  ros2 run doosan_controller test_moveC 100 3        # 사용자 예시와 동일한 하강 호 (기본값)
  ros2 run doosan_controller test_moveC 127 3 223    # 이전 예시(Δy=223, Δz=127)를 쓰려면 명시
  ros2 run doosan_controller test_moveC 0 4          # MoveL 상대이동 [0,180,100,0,-270,0]
"""

import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from dsr_msgs2.srv import MoveCircle
from robot_arm_interfaces.action import MoveL
from robot_arm_interfaces.msg import RobotStatus

ROBOT_NS         = 'dsr01'
DEFAULT_RADIUS    = 100.0   # mm
DEFAULT_DIRECTION = 1       # +1: +Y, -1: -Y
VEL               = 30.0    # mm/s, deg/s
ACC               = 60.0    # mm/s², deg/s²
ARC_SEGMENTS      = 6       # direction=2: 90°를 6분할 (15°/segment) MoveL로 구면 추종
ARC_BLEND_MM       = 35.0   # 중간 구간 블렌드 반경 상한 (마지막 구간은 0)
ARC_BLEND_FRACTION = 0.4    # 블렌드 반경은 인접 구간 길이의 이 비율도 넘지 않도록 제한

# direction=2 전용: 원호가 끝난 뒤 이어붙이는 추가 직선 이동의 최종 목표 —
# 시작점(현재 TCP) 기준 상대이동이 정확히 이 값이 되도록 한다 (원호 자체의
# 이동량(+r,+r)과는 무관하게, 동작 전체의 결과를 이 값으로 고정).
RETREAT_FINAL_Y_OFFSET_MM = 150.0   # 시작점 기준 최종 Y 상대이동 목표 (+15cm)
RETREAT_FINAL_Z_OFFSET_MM = -100.0  # 시작점 기준 최종 Z 상대이동 목표 (-10cm)

# direction=3 전용: y_distance_mm 미지정 시 기본값 — 사용자 실측 예시(Δy=182.47mm)와 일치.
DEFAULT_DESCEND_Y_DISTANCE_MM = 182.47

# direction=4 전용: 단일 MoveL 상대이동값 [x, y, z, rx, ry, rz] (mm / deg).
# 현재 TCP 기준으로 위치와 자세를 함께 더해서 이동한다.
RELATIVE_MOVE = [0.0, 180.0, 100.0, 0.0, -270.0, 0.0]


class TestMoveCNode(Node):

    def __init__(self, radius_mm: float, direction: int, y_distance_mm: float | None = None):
        super().__init__('test_movec')
        self._radius      = radius_mm
        self._direction   = direction   # +1: +Y, -1: -Y, 2: 후퇴, 3: 하강
        # direction=3 전용: Y 이동량을 radius와 분리. 미지정 시 radius와 동일.
        self._y_distance  = DEFAULT_DESCEND_Y_DISTANCE_MM if y_distance_mm is None else y_distance_mm
        self._tcp         = None        # [x, y, z, rx, ry, rz]  mm / deg

        self._status_sub = self.create_subscription(
            RobotStatus, '/arm/status', self._status_cb, 10
        )
        self._cli_circle = self.create_client(
            MoveCircle, f'/{ROBOT_NS}/motion/move_circle'
        )
        self._movel = ActionClient(self, MoveL, '/arm/move_l')

    # ── callbacks ────────────────────────────────────────────────────────────

    def _status_cb(self, msg: RobotStatus):
        if msg.servo_on:
            self._tcp = list(msg.current_tcp)

    # ── helpers ──────────────────────────────────────────────────────────────

    def _wait_tcp(self, timeout: float = 5.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._tcp and any(v != 0.0 for v in self._tcp[:3]):
                return True
        return False

    def _move_circle(self, via: list, end: list) -> bool:
        """DSR move_circle 서비스 호출 (SYNC — 완료까지 블로킹)."""
        if not self._cli_circle.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('move_circle 서비스 없음 — launch 실행 여부 확인')
            return False

        via_msg      = Float64MultiArray()
        via_msg.data = [float(v) for v in via]
        end_msg      = Float64MultiArray()
        end_msg.data = [float(v) for v in end]

        req            = MoveCircle.Request()
        req.pos        = [via_msg, end_msg]
        req.vel        = [VEL, VEL]
        req.acc        = [ACC, ACC]
        req.time       = 0.0
        req.radius     = 0.0
        req.ref        = 0    # DR_BASE
        req.mode       = 0    # ABSOLUTE
        req.angle1     = 0.0
        req.angle2     = 0.0
        req.blend_type = 0
        req.sync_type  = 0    # SYNC

        fut = self._cli_circle.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=60.0)
        resp = fut.result()
        if resp is None:
            self.get_logger().error('move_circle 타임아웃')
            return False
        return bool(resp.success)

    def _move_l(
        self, x, y, z, rx, ry, rz,
        blend_radius_mm: float = 0.0, relative: bool = False,
    ) -> bool:
        """MoveL (SYNC — 완료까지 블로킹). relative=True면 현재 TCP 기준 상대이동."""
        self._movel.wait_for_server()

        goal = MoveL.Goal()
        goal.x, goal.y, goal.z      = float(x), float(y), float(z)
        goal.rx, goal.ry, goal.rz   = float(rx), float(ry), float(rz)
        goal.linear_velocity_mm_s   = VEL
        goal.angular_velocity_deg_s = VEL
        goal.linear_accel_mm_s2     = ACC
        goal.angular_accel_deg_s2   = ACC
        goal.blend_radius_mm        = float(blend_radius_mm)
        goal.reference_frame        = 0      # DR_BASE
        goal.relative                = relative

        send_fut = self._movel.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_fut)
        handle = send_fut.result()
        if not handle.accepted:
            self.get_logger().error('MoveL 거부 — Servo ON 여부 확인')
            return False

        res_fut = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        result = res_fut.result().result
        if result.success:
            return True
        self.get_logger().error(f'MoveL 실패: {result.message}')
        return False

    def _move_c_retreat(
        self, x0: float, y0: float, z0: float, r: float, ry0: float,
    ) -> bool:
        """direction=2: 피킹 후 후퇴 — 진짜 원호(단일 반지름 r)로 TCP 자체를 이동.

        원 중심 = 현재 TCP에서 +Y 방향으로 r만큼 (Z는 그대로) — C = (x0, y0+r, z0).
        시작점(현재 TCP)은 이 원 위에서 중심 기준 -Y 방향(θ=0)에 있고, 원주를
        따라 θ: 0°→90°로 진행하며 Y, Z 둘 다 단조 증가한다.
          P(θ) = (x0, y0 + r·(1-cosθ), z0 + r·sinθ)
          θ=90°에서: y = y0+r = 원 중심의 y와 같음, z = z0+r
        즉 "TCP에서 +Y 방향으로 r 떨어진 곳이 원 중심"이라면, 이동이 끝났을 때
        TCP의 y값이 정확히 그 원 중심의 y값이 된다 (z는 r만큼 더 올라간 상태).

        자세: Rx=90, Rz=0 고정 (motion_sequence.py의 _move_c, test_movel.py,
        approach/contact 포즈 등 코드베이스 전체에서 쓰는 컨벤션과 동일), Ry만
        실제 측정된 시작값 ry0에서 +90°만큼 보낸다 (예: 180 → -90, 코드베이스의
        "Y축 정렬" 포즈 (90,-90,0)과 정확히 일치).
          Rx = 90      (고정)
          Ry = ry0 + θ  (θ: 0→90, 결과가 180을 넘으면 -360 wrap)
          Rz = 0       (고정)
        ry0는 하드코딩하지 않고 실제 현재 TCP에서 읽어, 짐벌락(180/-180) 근처라도
        실제 자세에 자연스럽게 이어지게 한다.

        MoveCircle 서비스(2점) 단일 호출은 실제 원호를 보장하지 않는 것으로
        확인되어, motion_sequence.py의 _move_c와 동일하게 N개 점을 직접 계산해
        MoveL을 연속 호출 — TCP가 원주를 그대로 따라가게 한다.

        참고: 이 동작은 그리퍼 끝이 아니라 TCP(플랜지) 좌표 자체가 원호를 그린다
        — 그리퍼 길이 보정은 적용하지 않는다 (기존 단순 MoveC와 같은 방식).

        블렌드 반경은 ARC_BLEND_MM을 상한으로 쓰되, 인접한 두 구간(들어오는 구간/
        나가는 구간) 중 더 짧은 쪽 길이의 ARC_BLEND_FRACTION만큼으로도 한 번 더 제한한다.
        radius가 작아 구간 길이가 짧을 때 ARC_BLEND_MM을 그대로 쓰면 블렌드 반경이
        구간보다 커져 DSR이 블렌드를 거부하거나 경로가 깨질 수 있기 때문.
        """
        N = ARC_SEGMENTS

        waypoints = []
        for i in range(N + 1):
            theta_deg = i * 90.0 / N
            theta_rad = math.radians(theta_deg)
            px = x0
            py = y0 + r * (1.0 - math.cos(theta_rad))
            pz = z0 + r * math.sin(theta_rad)
            r_x = 90.0
            r_y_raw = ry0 + theta_deg
            r_y = r_y_raw - 360.0 if r_y_raw > 180.0 else r_y_raw
            r_z = 0.0
            waypoints.append((px, py, pz, r_x, r_y, r_z))

        # 추가 직선 구간: 동작 전체의 결과가 시작점 기준 정확히
        # (RETREAT_FINAL_Y_OFFSET_MM, RETREAT_FINAL_Z_OFFSET_MM) 상대이동이
        # 되도록, 원호 끝점 뒤에 마지막 목표점 하나를 이어붙인다. 자세는
        # 원호 끝 자세를 그대로 유지(추가 회전 없음).
        ry_arc_end = waypoints[-1][4]
        final_y = y0 + RETREAT_FINAL_Y_OFFSET_MM
        final_z = z0 + RETREAT_FINAL_Z_OFFSET_MM
        waypoints.append((x0, final_y, final_z, 90.0, ry_arc_end, 0.0))

        last_idx = len(waypoints) - 1
        seg_lengths = [
            math.dist(waypoints[i][:3], waypoints[i + 1][:3])
            for i in range(last_idx)
        ]

        for i in range(1, last_idx + 1):
            px, py, pz, r_x, r_y, r_z = waypoints[i]

            if i == last_idx:
                blend = 0.0
            else:
                neighbor_len = min(seg_lengths[i - 1], seg_lengths[i])
                blend = min(ARC_BLEND_MM, neighbor_len * ARC_BLEND_FRACTION)

            if i <= N:
                label = f'θ={i * 90.0 / N:.0f}°'
            else:
                label = '추가 이동(최종 목표)'

            self.get_logger().info(
                f'  [{i}/{last_idx}] {label}'
                f'  pos=({px:.1f},{py:.1f},{pz:.1f})'
                f'  ori=({r_x:.0f},{r_y:.0f},{r_z:.0f})'
                f'  blend={blend:.1f}mm'
            )

            if not self._move_l(px, py, pz, r_x, r_y, r_z, blend_radius_mm=blend):
                self.get_logger().error(f'후퇴 이동 실패 [{i}/{last_idx}] {label}')
                return False

        return True

    def _move_c_descend(
        self, x0: float, y0: float, z0: float, r: float, y_distance: float,
        ry0: float,
    ) -> bool:
        """direction=3: 플랜지(TCP) 좌표 기준으로 Y는 증가, Z는 감소하는 비대칭
        호. 사용자가 실제 로봇을 조그해 찍은 예시
          시작 (398.11, -182.47, 666.76, 90, 180, 0)
          끝   (398.11,   40.55, 539.75, 90, -90, 0)
        를 그대로 일반화한 동작 — Δy≈+223, Δz≈-127, 자세 (90,180,0)→(90,-90,0).
        d=2(_move_c_retreat)와 같은 자세 공식(Rx=90,Rz=0 고정, Ry만 ry0→ry0+90)을
        쓰지만, 이건 d=2처럼 "그리퍼 끝 기준"으로 보정하지 않고 사용자가 찍은
        예시 그대로 플랜지 좌표를 직접 따라간다 (의도가 "정확히 이 모양대로
        움직여라"였기 때문 — 보정하면 예시와 다른 플랜지 경로가 나옴).

        P(θ) = (x0, y0 + y_distance·(1-cosθ), z0 - radius·sinθ),  θ: 0°→90°
          → Y 증가, Z 감소, 초반 Z 위주·후반 Y 위주인 비대칭(뚱뚱한) 호.

        radius/y_distance는 CLI에서 그대로 받는다 — 예시를 정확히 재현하려면
        radius=127, y_distance=223으로 호출.
        """
        N = ARC_SEGMENTS

        waypoints = []
        for i in range(N + 1):
            theta_deg = i * 90.0 / N
            theta_rad = math.radians(theta_deg)
            px = x0
            py = y0 + y_distance * (1.0 - math.cos(theta_rad))
            pz = z0 - r * math.sin(theta_rad)
            r_x = 90.0
            r_y_raw = ry0 + theta_deg
            r_y = r_y_raw - 360.0 if r_y_raw > 180.0 else r_y_raw
            r_z = 0.0
            waypoints.append((px, py, pz, r_x, r_y, r_z))

        seg_lengths = [
            math.dist(waypoints[i][:3], waypoints[i + 1][:3]) for i in range(N)
        ]

        for i in range(1, N + 1):
            px, py, pz, r_x, r_y, r_z = waypoints[i]
            theta_deg = i * 90.0 / N

            if i == N:
                blend = 0.0
            else:
                neighbor_len = min(seg_lengths[i - 1], seg_lengths[i])
                blend = min(ARC_BLEND_MM, neighbor_len * ARC_BLEND_FRACTION)

            self.get_logger().info(
                f'  [{i}/{N}] θ={theta_deg:.0f}°'
                f'  pos=({px:.1f},{py:.1f},{pz:.1f})'
                f'  ori=({r_x:.0f},{r_y:.0f},{r_z:.0f})'
                f'  blend={blend:.1f}mm'
            )

            if not self._move_l(px, py, pz, r_x, r_y, r_z, blend_radius_mm=blend):
                self.get_logger().error(f'하강 호 이동 실패 θ={theta_deg:.0f}°')
                return False

        return True

    # ── main sequence ────────────────────────────────────────────────────────

    def run(self):
        # 1. 현재 TCP 취득
        self.get_logger().info('현재 TCP 위치 취득 중...')
        if not self._wait_tcp():
            self.get_logger().error('TCP 위치 미수신 — Servo ON 여부 확인')
            return

        r   = self._radius
        d   = self._direction   # +1 or -1
        x0, y0, z0 = self._tcp[0], self._tcp[1], self._tcp[2]
        self.get_logger().info(f'현재 TCP : ({x0:.1f}, {y0:.1f}, {z0:.1f}) mm')

        a45 = math.radians(45)

        if d == 1:
            # ── +1: 구 중심 = 현재 TCP 아래(-Z) ─────────────────────────────
            # P(θ) = (x0, y0 + r·sinθ, (z0-r) + r·cosθ)
            # θ: 0°→90°  →  +Y 방향, Z 감소
            # 방향: Ry = 180 - θ°  (공구가 항상 아래쪽 중심을 향함)
            Cx, Cy_c, Cz_c = x0, y0, z0 - r
            via = [Cx, Cy_c + r * math.sin(a45), Cz_c + r * math.cos(a45),
                   -90.0, 135.0, 0.0]
            end = [Cx, Cy_c + r,                  Cz_c,
                   -90.0,  90.0, 0.0]
            desc = '+Y 방향, Z 감소'
            self.get_logger().info(
                f'구 중심  : ({Cx:.1f}, {Cy_c:.1f}, {Cz_c:.1f}) mm  (현재 TCP 아래)'
            )

        elif d == -1:
            # ── -1: 구 중심 = 현재 TCP의 -Y 방향 ────────────────────────────
            # 현재 TCP를 90° 지점으로 보고 반대 방향(0°)으로 복귀하는 호.
            # P(θ) = (x0, (y0-r) + r·sinθ, z0 + r·cosθ)
            # θ: 0°→90°  →  -Y 방향, Z 증가  (+1 경로의 역방향)
            # 방향: Ry = 135(via)→180(end)  (공구가 항상 -Y 쪽 중심을 향함)
            Cx, Cy_c, Cz_c = x0, y0 - r, z0
            via = [Cx, Cy_c + r * math.sin(a45), Cz_c + r * math.cos(a45),
                   -90.0, 135.0, 0.0]
            end = [Cx, Cy_c,                      Cz_c + r,
                   -90.0, 180.0, 0.0]
            desc = '-Y 방향, Z 증가 (복귀)'
            self.get_logger().info(
                f'구 중심  : ({Cx:.1f}, {Cy_c:.1f}, {Cz_c:.1f}) mm  (현재 TCP의 -Y)'
            )

        elif d == 2:
            # ── 2: 구 중심 = 현재 TCP에서 +Y 방향으로 r — 피킹 후 후퇴 ───────
            # 현재 TCP(Z축 정렬, ry≈180)를 θ=0 지점으로 보고, Rx=90,Rz=0 고정,
            # Ry만 +90° 전환(예: 180→-90). 진짜 원호를 따라 Y·Z 모두 단조 증가,
            # θ=90°에서 TCP의 y값이 원 중심의 y값(y0+r)과 같아진다.
            # MoveCircle(2점) 단일 호출은 실제 원호를 보장하지 않으므로,
            # MoveL N분할로 원주를 직접 따라간다 (motion_sequence.py의 _move_c와 동일 기법).
            ry0 = self._tcp[4]
            ry_end_raw = ry0 + 90.0
            ry_end = ry_end_raw - 360.0 if ry_end_raw > 180.0 else ry_end_raw
            self.get_logger().info(
                f'경로     : 피킹 후 후퇴 (MoveL×{ARC_SEGMENTS}, 진짜 원호 r={r:.0f}mm)'
            )
            self.get_logger().info(
                f'원 중심  : ({x0:.1f}, {y0 + r:.1f}, {z0:.1f}) mm'
            )
            self.get_logger().info(
                f'원호 끝  : ({x0:.1f}, {y0 + r:.1f}, {z0 + r:.1f}) mm'
                f'  자세: rx=90 rz=0 고정, ry {ry0:.1f}→{ry_end:.1f}'
            )
            self.get_logger().info(
                f'최종 목표: ({x0:.1f}, {y0 + RETREAT_FINAL_Y_OFFSET_MM:.1f}, '
                f'{z0 + RETREAT_FINAL_Z_OFFSET_MM:.1f}) mm'
                f'  (시작점 기준 상대이동 y{RETREAT_FINAL_Y_OFFSET_MM:+.0f}mm, '
                f'z{RETREAT_FINAL_Z_OFFSET_MM:+.0f}mm)'
            )
            if self._move_c_retreat(x0, y0, z0, r, ry0):
                self.get_logger().info('MoveC(후퇴) 완료')
            else:
                self.get_logger().error('MoveC(후퇴) 실패')
            return

        elif d == 3:
            # ── 3: 플랜지 기준 Y 증가·Z 감소 비대칭 호 (사용자 실측 예시 일반화) ──
            # 시작 (x0,y0,z0,90,180,0) → 끝 (x0,y0+y_distance,z0-r,90,-90,0).
            # Rx=90,Rz=0 고정, Ry만 +90° 전환 — d=2와 같은 자세 공식.
            y_dist = self._y_distance
            ry0 = self._tcp[4]
            ry_end_raw = ry0 + 90.0
            ry_end = ry_end_raw - 360.0 if ry_end_raw > 180.0 else ry_end_raw
            self.get_logger().info(
                f'경로     : 하강 호 (MoveL×{ARC_SEGMENTS})  '
                f'Y+{y_dist:.0f}mm  Z-{r:.0f}mm'
            )
            self.get_logger().info(
                f'목표     : ({x0:.1f}, {y0 + y_dist:.1f}, {z0 - r:.1f}) mm'
                f'  자세: rx=90 rz=0 고정, ry {ry0:.1f}→{ry_end:.1f}'
            )
            if self._move_c_descend(x0, y0, z0, r, y_dist, ry0):
                self.get_logger().info('MoveC(하강) 완료')
            else:
                self.get_logger().error('MoveC(하강) 실패')
            return

        else:
            # ── 4: MoveL 상대이동 (위치+자세) — 원호 없이 단일 구간 ──────────
            # 고정 상대이동값 [x,y,z,rx,ry,rz] = RELATIVE_MOVE — 현재 TCP를
            # 기준으로 위치뿐 아니라 자세(rx,ry,rz)까지 상대로 더해서 이동.
            dx, dy, dz, drx, dry, drz = RELATIVE_MOVE
            self.get_logger().info(
                f'경로     : MoveL 상대이동  Δ=({dx:.0f},{dy:.0f},{dz:.0f}) mm'
                f'  Δ자세=({drx:.0f},{dry:.0f},{drz:.0f})°'
            )
            if self._move_l(dx, dy, dz, drx, dry, drz, relative=True):
                self.get_logger().info('MoveL(상대이동) 완료')
            else:
                self.get_logger().error('MoveL(상대이동) 실패')
            return

        self.get_logger().info(f'경로     : {desc}  r={r:.0f} mm')
        self.get_logger().info(
            f'Via (45°): ({via[0]:.1f}, {via[1]:.1f}, {via[2]:.1f})'
            f'  rx={via[3]:.0f} ry={via[4]:.0f} rz={via[5]:.0f}'
        )
        self.get_logger().info(
            f'End      : ({end[0]:.1f}, {end[1]:.1f}, {end[2]:.1f})'
            f'  rx={end[3]:.0f} ry={end[4]:.0f} rz={end[5]:.0f}'
        )

        # 4. MoveC 실행
        self.get_logger().info('MoveC 실행 중...')
        if self._move_circle(via, end):
            self.get_logger().info('MoveC 완료')
        else:
            self.get_logger().error('MoveC 실패')


def main(args=None):
    radius_mm = DEFAULT_RADIUS
    direction = DEFAULT_DIRECTION
    y_distance_mm = None   # None → direction=2에서 radius_mm와 동일하게 사용

    positional = []
    for arg in sys.argv[1:]:
        if arg == '--ros-args':
            break
        try:
            positional.append(float(arg))
        except ValueError:
            continue

    if len(positional) >= 1:
        radius_mm = positional[0]
    if len(positional) >= 2:
        raw = int(positional[1])
        direction = raw if raw in (1, -1, 2, 3, 4) else (1 if raw >= 0 else -1)
    if len(positional) >= 3:
        y_distance_mm = positional[2]

    dir_str = {
        1: '+Y', -1: '-Y', 2: '후퇴(원호 Y+·Z+)', 3: '하강(Y+·Z-)',
        4: f'MoveL 상대이동 {RELATIVE_MOVE}',
    }[direction]
    print(f'[test_moveC] 반지름: {radius_mm:.0f} mm  방향: {dir_str}'
          + (f'  Y거리: {y_distance_mm:.0f} mm' if direction == 3 and y_distance_mm is not None else ''))
    rclpy.init(args=args)
    node = TestMoveCNode(radius_mm, direction, y_distance_mm)
    try:
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
