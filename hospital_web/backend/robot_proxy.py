"""
Compatibility adapter for robot-arm control.

Historically hospital_web called web_interface over HTTP at localhost:8000.
The robot bridge now lives in-process, so this module keeps the old proxy API
while dispatching directly to hospital_web.backend.robot_bridge.
"""

import asyncio
from typing import Optional


def _node():
    try:
        from . import robot_bridge
        return robot_bridge.get_node()
    except Exception:
        return None


def get_robot_state() -> dict:
    node = _node()
    return node.get_state() if node else {}


def is_online() -> bool:
    node = _node()
    if node is None:
        return False
    state = node.get_state()
    return bool(state.get('arm_ready') or state.get('robot_state', -1) != -1)


async def poll_loop() -> None:
    """Kept for main.py compatibility; robot state now arrives via ROS callbacks."""
    while True:
        await asyncio.sleep(1.0)


async def post(path: str, body: Optional[dict] = None) -> dict:
    """Dispatch a legacy web_interface-style API path to the in-process bridge."""
    node = _node()
    if node is None:
        return {'success': False, 'message': 'Robot ROS bridge not initialized'}

    body = body or {}

    try:
        if path == '/api/servo':
            return await node.call_servo(bool(body.get('enable')))
        if path == '/api/jog':
            return await node.call_jog(body.get('joint_index', 0), body.get('speed', 0.0))
        if path == '/api/move_j':
            return await node.call_movej(
                body.get('joints', []),
                body.get('velocity', 30.0),
                body.get('acceleration', 60.0),
            )
        if path == '/api/move_l':
            return await node.call_movel(
                body.get('pose', []),
                body.get('velocity', 50.0),
                body.get('acceleration', 100.0),
            )
        if path == '/api/home':
            return await node.call_home()
        if path == '/api/teaching':
            return await node.call_teaching(bool(body.get('enable')))
        if path == '/api/estop':
            return await node.call_estop()
        if path in ('/api/recover', '/api/recovery'):
            return await node.call_safety_recovery()

        if path == '/api/motion/start':
            node.start_sequence(int(body.get('marker_id', 0)))
            return {'success': True}
        if path == '/api/motion/start_batch':
            node.start_sequence_batch(
                int(body.get('marker_id', 0)), int(body.get('count', 1))
            )
            return {'success': True}
        if path == '/api/motion/next':
            node.next_step()
            return {'success': True}
        if path == '/api/motion/stop':
            node.stop_sequence()
            return {'success': True}
        if path == '/api/motion/reset':
            node.reset_sequence()
            return {'success': True}

        if path == '/api/temp_motion/start':
            node.start_temp_sequence(int(body.get('marker_id', 0)))
            return {'success': True}
        if path == '/api/temp_motion/next':
            node.next_temp_step()
            return {'success': True}
        if path == '/api/temp_motion/stop':
            node.stop_temp_sequence()
            return {'success': True}

        if path == '/api/gripper/on':
            return await node.call_magnet(True)
        if path == '/api/gripper/off':
            return await node.call_magnet(False)

        if path == '/api/inverter/freq':
            node.publish_inverter_freq(int(body.get('freq', 0)))
            return {'success': True}
        if path == '/api/inverter/run':
            node.publish_inverter_run(bool(body.get('run')))
            return {'success': True}

        if path == '/api/jog_cart':
            axis = body.get('axis', 0)
            if isinstance(axis, str):
                axis = {'x': 0, 'y': 1, 'z': 2, 'rx': 3, 'ry': 4, 'rz': 5}.get(axis, 0)
            direction = int(body.get('direction', 1))
            step = float(body.get('distance', body.get('speed', 5.0)))
            ok = await node.call_jog_cart_step(int(axis), direction, step)
            return {'success': ok, 'message': 'OK' if ok else 'Cartesian jog failed'}

        return {'success': False, 'message': f'Unsupported robot path: {path}'}
    except Exception as exc:
        return {'success': False, 'message': f'Robot bridge error: {exc}'}
