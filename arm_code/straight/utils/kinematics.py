"""
kinematics.py  –  Straight (coaxial) end-effector kinematics and pose helpers

The straight end effector is mounted coaxially on panda_link8: the magnet tip
lies on link8's +z axis, `length` metres from the flange
(config_straight.json -> end_effector.length_mm).  Tip and flange therefore share the
same orientation.

All Euler angles use the 'rxyz' (rotating-frame X-Y-Z) convention used
throughout this project:  R = Rx(roll) @ Ry(pitch) @ Rz(yaw).
Quaternions are [x, y, z, w] (tf.transformations / geometry_msgs order).
"""

from math import atan2, degrees

import numpy as np
import tf.transformations as ttf


def euler_to_quat(roll, pitch, yaw):
    """rxyz Euler angles -> quaternion [x, y, z, w]."""
    return ttf.quaternion_from_euler(roll, pitch, yaw, 'rxyz')


def quat_to_euler(q):
    """Quaternion [x, y, z, w] -> rxyz Euler angles (roll, pitch, yaw)."""
    return ttf.euler_from_quaternion(q, axes='rxyz')


def rxyz_matrix(roll, pitch, yaw):
    """3x3 rotation matrix of rxyz Euler angles."""
    return np.array(ttf.quaternion_matrix(euler_to_quat(roll, pitch, yaw)))[:3, :3]


def link_back_straight(end_x, end_y, end_z, end_roll, end_pitch, end_yaw, length):
    """Magnet-tip pose -> panda_link8 pose.

    Same maths as arm_code/new/utils/link_back.py::link_back:
        p_link = p_end - R @ [0, 0, length]

    Returns [x, y, z, roll, pitch, yaw] of panda_link8 (metres, radians).
    """
    R = rxyz_matrix(end_roll, end_pitch, end_yaw)
    p_link = np.array([end_x, end_y, end_z], dtype=float) - R @ np.array([0.0, 0.0, length])
    return [float(p_link[0]), float(p_link[1]), float(p_link[2]),
            float(end_roll), float(end_pitch), float(end_yaw)]


def link_forward_straight(link_x, link_y, link_z, link_roll, link_pitch, link_yaw, length):
    """panda_link8 pose -> magnet-tip pose (inverse of link_back_straight).

        p_end = p_link + R @ [0, 0, length]

    Returns [x, y, z, roll, pitch, yaw] of the magnet tip (metres, radians).
    """
    R = rxyz_matrix(link_roll, link_pitch, link_yaw)
    p_end = np.array([link_x, link_y, link_z], dtype=float) + R @ np.array([0.0, 0.0, length])
    return [float(p_end[0]), float(p_end[1]), float(p_end[2]),
            float(link_roll), float(link_pitch), float(link_yaw)]


def get_link8_euler(group):
    """Current panda_link8 pose from MoveIt -> (x, y, z, roll, pitch, yaw)."""
    pose = group.get_current_pose().pose
    q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    roll, pitch, yaw = quat_to_euler(q)
    return (pose.position.x, pose.position.y, pose.position.z, roll, pitch, yaw)


def link_pose_msg(link_pose, arm_acc):
    """[x, y, z, roll, pitch, yaw] of panda_link8 -> geometry_msgs/Pose.

    The position is rounded to the arm accuracy `arm_acc` (metres), as in the
    original code.
    """
    from geometry_msgs.msg import Pose  # imported here so the maths above has no ROS msg dependency

    pose = Pose()
    pose.position.x = np.round(link_pose[0] / arm_acc, 0) * arm_acc
    pose.position.y = np.round(link_pose[1] / arm_acc, 0) * arm_acc
    pose.position.z = np.round(link_pose[2] / arm_acc, 0) * arm_acc
    q = euler_to_quat(link_pose[3], link_pose[4], link_pose[5])
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]
    return pose


def rot_err_deg(q1, q2):
    """Angle (deg) of the rotation between two quaternions [x, y, z, w]."""
    q1 = np.asarray(q1, dtype=float)
    q2 = np.asarray(q2, dtype=float)
    q_rel = ttf.quaternion_multiply(ttf.quaternion_inverse(q1 / np.linalg.norm(q1)),
                                    q2 / np.linalg.norm(q2))
    return degrees(2.0 * atan2(np.linalg.norm(q_rel[:3]), abs(q_rel[3])))
