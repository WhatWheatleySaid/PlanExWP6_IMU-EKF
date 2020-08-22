"""
Work Package 6 - IMU Filtering
Authors: G. Liang, A. Bauer, N. Meyners, L. Mueller
Course: PlanEx SoSe 20
License: MIT

Description:
Contains multiple functions for quaternion operations, transformations and rotations.
"""

import numpy as np
from numpy.linalg import norm
import math as math


def quaternion_from_gyr(gyr, rate):
    """
    compute quaternion from single gyro data (x,y,z).
    """
    assert gyr.shape == (3,)

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

    assert acc.shape == (3,)
    assert mag.shape == (3,)
    z_earth_imu = acc / norm(acc)
    y_earth_imu = np.cross(z_earth_imu, mag) / norm(np.cross(z_earth_imu, mag))
    x_earth_imu = np.cross(y_earth_imu, z_earth_imu) / norm(np.cross(y_earth_imu, z_earth_imu))

    # x_earth_imu = np.cross(mag, z_earth_imu) / norm(np.cross(mag, z_earth_imu))
    # y_earth_imu = np.cross(z_earth_imu, x_earth_imu) / norm(np.cross(z_earth_imu, x_earth_imu))

    quat_imu_earth = quaternion_from_rotmat(np.array([x_earth_imu.T, y_earth_imu.T, z_earth_imu.T]))

    return quat_imu_earth


def quaternion_from_rotmat(r):
    """
    Creates quaternion from a rotation matrix.
    :param r: rotation matrix
    :type r: ndarray
    :return: q
    """
    assert r.shape == (3,3)

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

    assert q1.shape == (4,)
    assert q2.shape == (4,)

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
    assert q.shape == (4,)
    assert vec.shape == (3,)

    qinv = quaternion_invert(q)
    foo = np.hstack(np.array([0, vec])).flatten()

    qv = quaternion_multiply(quaternion_multiply(q, foo), qinv)
    v = qv[1:]
    return v


def quaternion_invert(q):
    assert q.shape == (4,)
    return np.array([q[0], -q[1], -q[2], -q[3]])

def vec_rotate_x(alph, vec):
    """
    Rotate around x axis with given angle (alph).

    :param alph: angle in rad
    :type alph: float
    :param vec: the vector that should be rotated
    :type vec: ndarray(3,)
    :return: rotated vector
    :rtype: ndarray(3,)
    """

    rotmat = np.array([[1, 0, 0],
                       [0, np.cos(alph), -np.sin(alph)],
                       [0, np.sin(alph), np.cos(alph)]])
    return rotmat.dot(vec)

def vec_rotate_y(alph, vec):
    """
    Rotate around y axis with given angle (alph).

    :param alph: angle in rad
    :type alph: float
    :param vec: the vector that should be rotated
    :type vec: ndarray(3,)
    :return: rotated vector
    :rtype: ndarray(3,)
    """

    rotmat = np.array([[np.cos(alph), 0, np.sin(alph)],
                       [0, 1, 0],
                       [-np.sin(alph), 0, np.cos(alph)]])
    return rotmat.dot(vec)

def vec_rotate_z(alph, vec):
    """
    Rotate around z axis with given angle (alph).

    :param alph: angle in rad
    :type alph: float
    :param vec: the vector that should be rotated
    :type vec: ndarray(3,)
    :return: rotated vector
    :rtype: ndarray(3,)
    """

    rotmat = np.array([[np.cos(alph), -np.sin(alph), 0],
                       [np.sin(alph), np.cos(alph), 0],
                       [0, 0, 1]])
    return rotmat.dot(vec)


def quat2euler(quat):
    """
    Transforms quaternion into euler angles roll/alpha, pitch/beta, yaw/gamma
    :param quat: the quaternion that should be transformed to euler angles
    :return: roll, pitch, yaw in rad
    :rtype: ndarray(3,)
    """
    assert quat.shape == (4,)

    alpha = np.arctan2(2 * (quat[0]*quat[3] + quat[1] * quat[2]), 1 - 2 * (quat[2]**2 + quat[3]**2))
    beta = np.arcsin(2 * quat[0] * quat[2] - quat[3] * quat[1])
    gamma = np.arctan2(2 * (quat[0]*quat[1] + quat[2] * quat[3]), 1 - 2 * (quat[1]**2 + quat[2]**2))

    return np.array([alpha, beta, gamma])