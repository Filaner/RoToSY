from geometry_msgs.msg import TransformStamped

from rotosy_calibration.transform_math import matrix_to_quaternion


def transform_to_msg(transform, parent_frame, child_frame, stamp):
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = parent_frame
    msg.child_frame_id = child_frame
    msg.transform.translation.x = float(transform[0, 3])
    msg.transform.translation.y = float(transform[1, 3])
    msg.transform.translation.z = float(transform[2, 3])
    qx, qy, qz, qw = matrix_to_quaternion(transform[:3, :3])
    msg.transform.rotation.x = float(qx)
    msg.transform.rotation.y = float(qy)
    msg.transform.rotation.z = float(qz)
    msg.transform.rotation.w = float(qw)
    return msg
