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
    acc = qtool.vec_rotate_z(np.pi / 2, qtool.vec_rotate_x(np.pi, acc))
    acc = acc + [0, 0, 9.8]
    # if quat.shape[0] == 1:
    acc_robot_earth = qtool.quaternion_rotate(quat, acc)
    # else:
    #     acc_robot_earth = np.array([qtool.quaternion_rotate(quat[i,], acc[i,]) for i in range(quat.shape[0])])
    # velocity = np.cumsum(acc_robot_earth, axis=0) / rate
    # pos_kin = np.cumsum(velocity, axis=0) / rate
    velocity = vel_sum + acc_robot_earth/rate
    pos = pos_sum + velocity/rate

    return pos, velocity
