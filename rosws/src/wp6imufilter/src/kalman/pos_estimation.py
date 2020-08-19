from kalman import quaternion_tools as qtool
import numpy as np


def pos_estimation(rate, quat, ds_acc):
    """
    Compute position from quaternion and acc measurements and rate. Returns Position.

    :param quat: quaternion
    :type quat: ndarray
    :param ds_acc: x,y,z from accelerometer readings
    :type ds_acc: ndarray
    :return:
    """
    ds_acc = ds_acc + [0, 0, 9.8]
    acc_robot_earth = np.array([qtool.quaternion_rotate(quat[i,], ds_acc[i,]) for i in range(quat.shape[0])])
    velocity = np.cumsum(acc_robot_earth, axis=0) / rate
    pos_kin = np.cumsum(velocity, axis=0) / rate
    return pos_kin



