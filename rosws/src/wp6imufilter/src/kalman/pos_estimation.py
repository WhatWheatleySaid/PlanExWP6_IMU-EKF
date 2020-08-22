"""
Work Package 6 - IMU Filtering
Authors: G. Liang, A. Bauer, N. Meyners, L. Mueller
Course: PlanEx SoSe 20
License: MIT

Description:
Position estimation from given acc and quaternion.
"""

from kalman import quaternion_tools as qtool
import numpy as np


def pos_estimation(rate, quat, acc, pos_sum):
    """
    Compute position from quaternion and acc measurements and rate. Returns Position.

    :param quat: quaternion
    :type quat: ndarray
    :param ds_acc: x,y,z from accelerometer readings
    :type ds_acc: ndarray
    :return:
    """
    # initial first state from accmag, both rotated 180deg around z (imu heading is wrong)
    acc = qtool.vec_rotate_z(np.pi, acc)
    quat[1] = 0
    quat[2] = 0
    acc_robot_earth = qtool.quaternion_rotate(quat, acc)
    acc_robot_earth = acc_robot_earth - [0, 0, 9.8]
    velocity = acc_robot_earth/rate
    pos = pos_sum + velocity/rate
    pos[2] = 0

    return pos, velocity
