from kalman import quaternion_tools as qtool
import numpy as np


def pos_estimation(rate, quat, acc, vel_sum, pos_sum):
    """
    Compute position from quaternion and acc measurements and rate. Returns Position.

    :param quat: quaternion
    :type quat: ndarray
    :param ds_acc: x,y,z from accelerometer readings
    :type ds_acc: ndarray
    :return:
    """
    # acc = qtool.vec_rotate_z(np.pi / 2, qtool.vec_rotate_x(np.pi, acc))
    acc = qtool.vec_rotate_z(np.pi, acc)

    acc = acc - [0, 0, 9.8]
    # acc = np.array([1, 0, 0]) # real acc data are not constant at all
    quat[1] = 0
    quat[2] = 0
    acc_robot_earth = qtool.quaternion_rotate(quat, acc)
    velocity = acc_robot_earth/rate
    pos = pos_sum + velocity/rate
    pos[2] = 0

    return pos, velocity
