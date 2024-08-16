
from geometry_msgs.msg import Quaternion
from mess2_msgs.msg import EulerAngles, Vertex

import numpy as np


def normalize_quat(quat):
    """
    Normalize a quaternion.

    Args:
        quat (Quaternion): The quaternion to normalize.

    Returns:
        Quaternion: The normalized quaternion.
    """
    magnitude = np.sqrt(quat.x**2 + quat.y**2 + quat.z**2 + quat.w**2)
    quat_norm = Quaternion()
    quat_norm.x = quat.x / magnitude
    quat_norm.y = quat.y / magnitude
    quat_norm.z = quat.z / magnitude
    quat_norm.w = quat.w / magnitude
    return quat_norm


def average_two_quats(quat1, quat2):
    """
    Average two quaternions.

    Args:
        quat1 (Quaternion): The first quaternion.
        quat2 (Quaternion): The second quaternion.

    Returns:
        Quaternion: The averaged quaternion.
    """
    quat1_norm = normalize_quat(quat1)
    quat2_norm = normalize_quat(quat2)

    quat_dot = (quat1_norm.x * quat2_norm.x +
                quat1_norm.y * quat2_norm.y +
                quat1_norm.z * quat2_norm.z +
                quat1_norm.w * quat2_norm.w)

    sign = -1.0 if quat_dot < 0.0 else 1.0

    quat_avg = Quaternion()
    quat_avg.x = (quat1.x + sign * quat2.x) / 2
    quat_avg.y = (quat1.y + sign * quat2.y) / 2
    quat_avg.z = (quat1.z + sign * quat2.z) / 2
    quat_avg.w = (quat1.w + sign * quat2.w) / 2

    quat_avg_norm = normalize_quat(quat_avg)
    return quat_avg_norm


def convert_quat_to_eul(quat):
    """
    Convert a quaternion to Euler angles.

    Args:
        quat (Quaternion): The quaternion to convert.

    Returns:
        EulerAngles: The corresponding Euler angles.
    """
    eul = EulerAngles()

    eul.roll = np.atan2(2 * quat.w * quat.x + quat.y * quat.z,
                          1 - 2 * (quat.x**2 + quat.y**2))
    sign = 2 * (quat.w * quat.y - quat.x * quat.z)
    eul.pitch = (0.5 * np.pi if abs(sign) >= 1.0 
                 else -0.5 * np.pi * np.asin(sign))
    eul.yaw = np.atan2(2 * (quat.w * quat.z + quat.x * quat.y),
                         1 - 2 * (quat.y**2 + quat.z**2))

    return eul


def convert_eul_to_quat(eul):
    """
    Convert Euler angles to a quaternion.

    Args:
        eul (EulerAngles): The Euler angles to convert.

    Returns:
        Quaternion: The corresponding quaternion.
    """
    quat = Quaternion()

    cx = np.cos(0.5 * eul.roll)
    sx = np.sin(0.5 * eul.roll)
    cy = np.cos(0.5 * eul.pitch)
    sy = np.sin(0.5 * eul.pitch)
    cz = np.cos(0.5 * eul.yaw)
    sz = np.sin(0.5 * eul.yaw)

    quat.x = sx * cy * cz - cx * sy * sz
    quat.y = cx * sy * cz + sx * cy * sz
    quat.z = cx * cy * sz - sx * sy * cz
    quat.w = cx * cy * cz + sx * sy * sz

    return quat


def invert_quat(quat):
    """
    Invert a quaternion.

    Args:
        quat (Quaternion): The quaternion to invert.

    Returns:
        Quaternion: The inverted quaternion.
    """
    quat_inv = Quaternion()
    quat_inv.x = -quat.x
    quat_inv.y = -quat.y
    quat_inv.z = -quat.z
    quat_inv.w = quat.w
    return quat_inv


def multiply_two_quats(quat1, quat2):
    """
    Multiply two quaternions.

    Args:
        quat1 (Quaternion): The first quaternion.
        quat2 (Quaternion): The second quaternion.

    Returns:
        Quaternion: The product of the two quaternions.
    """
    quat_product = Quaternion()

    quat_product.x = (quat1.w * quat2.x + quat1.x * quat2.w +
                      quat1.y * quat2.z - quat1.z * quat2.y)
    quat_product.y = (quat1.w * quat2.y - quat1.x * quat2.z +
                      quat1.y * quat2.w + quat1.z * quat2.x)
    quat_product.z = (quat1.w * quat2.z + quat1.x * quat2.y -
                      quat1.y * quat2.x + quat1.z * quat2.w)
    quat_product.w = (quat1.w * quat2.w - quat1.x * quat2.x -
                      quat1.y * quat2.y - quat1.z * quat2.z)

    return quat_product


def wrap_to_pi(angle):
    """
    Wrap an angle to the range [-π, π].

    Args:
        angle (float): The angle to wrap.

    Returns:
        float: The angle wrapped to the range [-π, π].
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi
    while angle <= -np.pi:
        angle += 2.0 * np.pi
    return angle


def get_angle_between_edges(vertex0, vertex1, vertex2):
    """
    Calculate the angle between two edges defined by three vertices.

    Args:
        vertex0 (Vertex): The first vertex of the first edge.
        vertex1 (Vertex): The second vertex of the first edge and the first vertex of the second edge.
        vertex2 (Vertex): The second vertex of the second edge.

    Returns:
        float: The angle between the two edges in radians.
    """
    dx1_10 = vertex1.position.x - vertex0.position.x
    dx2_10 = vertex1.position.y - vertex0.position.y
    dx1_21 = vertex2.position.x - vertex1.position.x
    dx2_21 = vertex2.position.y - vertex1.position.y

    dot = dx1_10 * dx1_21 + dx2_10 * dx2_21
    mag_10 = np.sqrt(dx1_10**2 + dx2_10**2)
    mag_21 = np.sqrt(dx1_21**2 + dx2_21**2)

    theta_inv = dot / (mag_10 * mag_21)
    theta_inv = max(-1.0, min(1.0, theta_inv))

    theta = np.acos(theta_inv)
    return theta
