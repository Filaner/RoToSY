"""
hybrid_ik_node.py  -  MoveL middleware: DSR IK -> MoveJ + final MoveL

Intercepts /arm/move_l and replaces long moves with:
  1. DSR ikin service -> joint angles for a pre-approach point before target,
     computed with the SAME orientation (rx,ry,rz) as the target
  2. MoveJ -> fast joint-space motion to pre-approach
  3. MoveL -> precise final approach via /arm/move_l_real

Passthrough cases (forwarded directly to /arm/move_l_real):
  - short moves
  - blended / relative moves
  - DSR ikin fails for all 8 solution spaces

Launch file wires up:
  arm_controller: remaps /arm/move_l -> /arm/move_l_real
  hybrid_ik_node: provides /arm/move_l (public face)
"""

import asyncio
import math
import time

import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from dsr_msgs2.srv import Ikin
from std_msgs.msg import Float64MultiArray
from robot_arm_interfaces.action import MoveJ, MoveL
from robot_arm_interfaces.msg import RobotStatus


class HybridIKNode(Node):

    def __init__(self):
        super().__init__('hybrid_ik_node')
        cbg = ReentrantCallbackGroup()

        self._joints       = None   # list[6] deg
        self._tcp          = None   # list[6] [x,y,z mm, rx,ry,rz deg]
        self._tcp_rt       = None   # TF-based TCP from tcp_monitor
        self._tcp_rt_stamp = 0.0
        self._status_stamp = 0.0

        self.declare_parameter('robot_ns', 'dsr01')
        self.declare_parameter('enabled', True)
        self.declare_parameter('pre_approach_distance_mm', 10.0)
        self.declare_parameter('min_hybrid_distance_mm', 30.0)
        self.declare_parameter('max_status_age_sec', 0.5)
        self.declare_parameter('post_movej_settle_sec', 0.2)
        self.declare_parameter('prefer_tcp_monitor_pose', True)

        ns = self.get_parameter('robot_ns').get_parameter_value().string_value
        self._enabled = bool(self.get_parameter('enabled').value)
        self._pre_approach_mm = max(
            1.0,
            float(self.get_parameter('pre_approach_distance_mm').value),
        )
        self._min_hybrid_distance_mm = max(
            self._pre_approach_mm + 1.0,
            float(self.get_parameter('min_hybrid_distance_mm').value),
        )
        self._max_status_age_sec = max(
            0.1,
            float(self.get_parameter('max_status_age_sec').value),
        )
        self._post_movej_settle_sec = max(
            0.0,
            float(self.get_parameter('post_movej_settle_sec').value),
        )
        self._prefer_tcp_monitor_pose = bool(
            self.get_parameter('prefer_tcp_monitor_pose').value
        )

        self.create_subscription(
            RobotStatus, '/arm/status', self._status_cb, 10,
            callback_group=cbg,
        )
        self.create_subscription(
            Float64MultiArray, '/arm/tcp_pose', self._tcp_pose_cb, 10,
            callback_group=cbg,
        )
        self._ikin_cli = self.create_client(
            Ikin, f'/{ns}/motion/ikin',
            callback_group=cbg,
        )
        self._j_cli = ActionClient(self, MoveJ, '/arm/move_j',     callback_group=cbg)
        self._l_cli = ActionClient(self, MoveL, '/arm/move_l_real', callback_group=cbg)

        ActionServer(
            self, MoveL, '/arm/move_l',
            execute_callback=self._execute,
            callback_group=cbg,
        )
        self.get_logger().info(
            'hybrid_ik_node ready  '
            f'ns={ns}  enabled={self._enabled}  '
            f'pre={self._pre_approach_mm:.1f}mm  '
            f'min_distance={self._min_hybrid_distance_mm:.1f}mm'
        )

    # ── callbacks ──────────────────────────────────────────────────────────────

    def _status_cb(self, msg: RobotStatus):
        self._joints       = list(msg.current_joints_deg)
        self._tcp          = list(msg.current_tcp)
        self._status_stamp = time.monotonic()

    def _tcp_pose_cb(self, msg: Float64MultiArray):
        if len(msg.data) >= 6:
            self._tcp_rt = list(msg.data[:6])
            self._tcp_rt_stamp = time.monotonic()

    def _best_tcp(self):
        now = time.monotonic()
        if (
            self._prefer_tcp_monitor_pose
            and self._tcp_rt is not None
            and now - self._tcp_rt_stamp <= self._max_status_age_sec
        ):
            return self._tcp_rt, 'tcp_monitor'
        return self._tcp, 'status'

    # ── DSR IK ─────────────────────────────────────────────────────────────────

    async def _dsr_ik(self, pose6: list) -> tuple:
        """Call DSR ikin for all 8 solution spaces; return joints closest to current.

        Returns:
            (joint_angles_deg, '') on success
            (None, reason_str)    on failure
        """
        if not self._ikin_cli.wait_for_service(timeout_sec=2.0):
            return None, 'ikin service unavailable'

        best_joints = None
        best_dist   = float('inf')

        for sol in range(8):
            req          = Ikin.Request()
            req.pos      = [float(v) for v in pose6]
            req.sol_space = sol
            req.ref      = 0   # BASE frame

            fut = self._ikin_cli.call_async(req)
            await fut
            resp = fut.result()
            if resp is None or not resp.success:
                continue

            joints = list(resp.conv_posj)
            if self._joints is not None:
                dist = sum((a - b) ** 2 for a, b in zip(joints, self._joints))
            else:
                dist = 0.0

            if dist < best_dist:
                best_dist   = dist
                best_joints = joints

        if best_joints is None:
            return None, 'all 8 sol_space values failed'
        return best_joints, ''

    # ── action helpers ─────────────────────────────────────────────────────────

    async def _send_j(self, goal: MoveJ.Goal) -> tuple:
        self._j_cli.wait_for_server()
        log = self.get_logger()
        for attempt in range(3):
            log.info(
                f'hybrid_ik: MoveJ attempt {attempt+1}/3  '
                f'joints={[round(j, 1) for j in goal.joint_angles_deg]}'
            )
            gh = await self._j_cli.send_goal_async(goal)
            if gh.accepted:
                log.info(f'hybrid_ik: MoveJ accepted (attempt {attempt+1})')
                res = await gh.get_result_async()
                log.info(
                    f'hybrid_ik: MoveJ result  success={res.result.success} '
                    f'msg={res.result.message!r}'
                )
                return res.result.success, res.result.message
            log.warning(f'hybrid_ik: MoveJ rejected (attempt {attempt+1}/3)')
            if attempt < 2:
                await asyncio.sleep(0.3)
        return False, 'MoveJ rejected (3 attempts)'

    async def _send_l(self, goal: MoveL.Goal) -> tuple:
        self._l_cli.wait_for_server()
        log = self.get_logger()
        for attempt in range(3):
            gh = await self._l_cli.send_goal_async(goal)
            if gh.accepted:
                log.info(f'hybrid_ik: MoveL_real accepted (attempt {attempt+1})')
                res = await gh.get_result_async()
                log.info(
                    f'hybrid_ik: MoveL_real result  success={res.result.success} '
                    f'msg={res.result.message!r}'
                )
                return res.result.success, res.result.message, res.result.execution_time_sec
            log.warning(f'hybrid_ik: MoveL_real rejected (attempt {attempt+1}/3)')
            if attempt < 2:
                await asyncio.sleep(0.3)
        return False, 'MoveL_real rejected (3 attempts)', 0.0

    @staticmethod
    def _copy_l_goal(src: MoveL.Goal) -> MoveL.Goal:
        dst = MoveL.Goal()
        dst.x  = src.x;  dst.y  = src.y;  dst.z  = src.z
        dst.rx = src.rx; dst.ry = src.ry; dst.rz = src.rz
        dst.linear_velocity_mm_s   = src.linear_velocity_mm_s
        dst.angular_velocity_deg_s = src.angular_velocity_deg_s
        dst.linear_accel_mm_s2     = src.linear_accel_mm_s2
        dst.angular_accel_deg_s2   = src.angular_accel_deg_s2
        dst.blend_radius_mm        = src.blend_radius_mm
        dst.reference_frame        = src.reference_frame
        dst.relative               = src.relative
        return dst

    @staticmethod
    def _result(success: bool, msg: str, t: float = 0.0) -> MoveL.Result:
        r = MoveL.Result()
        r.success = success; r.message = msg; r.execution_time_sec = t
        return r

    # ── execute ────────────────────────────────────────────────────────────────

    async def _execute(self, goal_handle):
        g   = goal_handle.request
        log = self.get_logger()

        if not self._enabled:
            return await self._passthrough(goal_handle, g)

        if g.blend_radius_mm > 0.0 or g.relative:
            return await self._passthrough(goal_handle, g)

        # yield so pending _status_cb can run before we read state
        await asyncio.sleep(0.05)

        if self._tcp is None or self._joints is None:
            log.warning('hybrid_ik: no /arm/status yet — passthrough')
            return await self._passthrough(goal_handle, g)

        age = time.monotonic() - self._status_stamp
        if age > self._max_status_age_sec:
            log.warning(f'hybrid_ik: status {age:.2f}s old — passthrough')
            return await self._passthrough(goal_handle, g)

        tcp, tcp_source = self._best_tcp()
        cur  = tcp[:3]
        tgt  = [g.x, g.y, g.z]
        dist = math.dist(cur, tgt)
        log.info(
            f'hybrid_ik: cur=({cur[0]:.1f},{cur[1]:.1f},{cur[2]:.1f}) mm '
            f'source={tcp_source}  '
            f'target=({g.x:.0f},{g.y:.0f},{g.z:.0f}) mm  dist={dist:.1f} mm  '
            f'orient=({g.rx:.0f},{g.ry:.0f},{g.rz:.0f}) deg'
        )

        if dist <= self._min_hybrid_distance_mm:
            log.info('hybrid_ik: short move → passthrough')
            return await self._passthrough(goal_handle, g)

        # Pre-approach before target along travel direction.
        # Use the TARGET orientation so the wrist is already aligned before MoveL.
        dx, dy, dz = tgt[0]-cur[0], tgt[1]-cur[1], tgt[2]-cur[2]
        pre = [
            tgt[0] - self._pre_approach_mm * dx / dist,
            tgt[1] - self._pre_approach_mm * dy / dist,
            tgt[2] - self._pre_approach_mm * dz / dist,
        ]
        pre_pose6 = [pre[0], pre[1], pre[2], g.rx, g.ry, g.rz]

        ik_joints, err_msg = await self._dsr_ik(pre_pose6)
        if ik_joints is None:
            log.warning(f'hybrid_ik: DSR IK failed ({err_msg}) — passthrough')
            return await self._passthrough(goal_handle, g)

        log.info(
            f'hybrid_ik: IK → {[round(j, 1) for j in ik_joints]}  '
            f'pre-approach=({pre[0]:.0f},{pre[1]:.0f},{pre[2]:.0f}) mm'
        )

        # 1. MoveJ to pre-approach (wrist already in correct orientation)
        vel = g.angular_velocity_deg_s if g.angular_velocity_deg_s > 0 else 30.0
        acc = g.angular_accel_deg_s2   if g.angular_accel_deg_s2   > 0 else 60.0

        gj = MoveJ.Goal()
        gj.joint_angles_deg    = [float(a) for a in ik_joints]
        gj.velocity_deg_s      = vel
        gj.acceleration_deg_s2 = acc
        gj.blend_radius_mm     = 0.0
        gj.relative            = False

        ok_j, msg_j = await self._send_j(gj)
        if not ok_j:
            log.error(f'hybrid_ik: MoveJ failed ({msg_j})')
            goal_handle.abort()
            return self._result(False, f'MoveJ failed: {msg_j}')

        if self._post_movej_settle_sec > 0.0:
            await asyncio.sleep(self._post_movej_settle_sec)

        # 2. Final MoveL to exact target
        gl = self._copy_l_goal(g)
        gl.blend_radius_mm = 0.0

        ok_l, msg_l, t = await self._send_l(gl)
        if ok_l:
            goal_handle.succeed()
        else:
            log.error(f'hybrid_ik: final MoveL failed ({msg_l})')
            goal_handle.abort()
        return self._result(ok_l, msg_l, t)

    async def _passthrough(self, goal_handle, g: MoveL.Goal):
        ok, msg, t = await self._send_l(self._copy_l_goal(g))
        if ok:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return self._result(ok, msg, t)


# ── entry point ────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = HybridIKNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
