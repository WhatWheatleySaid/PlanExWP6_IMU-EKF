import numpy as np
from numpy.linalg import norm
import math as math


def quaternion_from_gyr(gyr, rate):
    """
    compute quaternion from single gyro data (x,y,z).
    """

    gyrnorm = norm(gyr)
    if gyrnorm == 0:
        axis = np.array([0, 0, 0])
    else:
        axis = gyr / gyrnorm
    angle = gyrnorm * (1 / rate)
    quat_curr_prev = np.zeros(4)
    quat_curr_prev[0] = np.cos(angle / 2)
    quat_curr_prev[1:] = axis * np.sin(angle / 2)
    return quat_curr_prev


def quaternion_from_accmag(acc, mag):
    """
    compute quaternion from single acc & mag data (x,y,z).
    """

    z_earth_imu = acc / norm(acc)
    y_earth_imu = np.cross(z_earth_imu, mag) / norm(np.cross(z_earth_imu, mag))
    x_earth_imu = np.cross(y_earth_imu, z_earth_imu) / norm(np.cross(y_earth_imu, z_earth_imu))

    quat_imu_earth = quaternion_from_rotmat(np.array([x_earth_imu.T, y_earth_imu.T, z_earth_imu.T]))
    return quat_imu_earth


def quaternion_from_rotmat(r):
    """
    Creates quaternion from a rotation matrix.
    :param r: rotation matrix
    :type r: ndarray
    :return: q
    """
    w_sq = (1 + r[0, 0] + r[1, 1] + r[2, 2]) / 4
    x_sq = (1 + r[0, 0] - r[1, 1] - r[2, 2]) / 4
    y_sq = (1 - r[0, 0] + r[1, 1] - r[2, 2]) / 4
    z_sq = (1 - r[0, 0] - r[1, 1] + r[2, 2]) / 4

    w = np.sqrt(w_sq)
    x = math.copysign(np.sqrt(x_sq), r[2, 1] - r[1, 2])
    y = math.copysign(np.sqrt(y_sq), r[0, 2] - r[2, 0])
    z = math.copysign(np.sqrt(z_sq), r[1, 0] - r[0, 1])
    return np.array([w, x, y, z])


def quaternion_multiply(q1, q2):
    """
    Multiplies quaternion, returns quaternion with same shape. q1 and q2 must have shape 4x1
    :param q1: ndarray(4)
    :param q2: ndarray(4)
    :return q3: ndarray(4)
    """
    q3 = np.zeros_like(q1)
    q3[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3]
    q3[1] = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2]
    q3[2] = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1]
    q3[3] = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0]
    return q3


def quaternion_rotate(q, vec):
    """
    Rotate the vectors v by the quaternions q

    :param q:
    :param vec:
    :return:
    """
    qinv = quaternion_invert(q)
    foo = np.hstack(np.array([0, vec])).flatten()

    qv = quaternion_multiply(quaternion_multiply(q, foo), qinv)
    v = qv[1:]
    return v


def quaternion_invert(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat2euler(quat):
    alpha = np.arctan2(2 * (quat[0]*quat[3] + quat[1] * quat[2]), 1 - 2 * (quat[2]**2 + quat[3]**2))
    beta = np.arcsin(2 * quat[0] * quat[2] - quat[3] * quat[1])
    gamma = np.arctan2(2 * (quat[0]*quat[1] + quat[2] * quat[3]), 1 - 2 * (quat[1]**2 + quat[2]**2))

    return np.array([alpha, beta, gamma])