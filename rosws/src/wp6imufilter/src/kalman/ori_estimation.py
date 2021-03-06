"""
Work Package 6 - IMU Filtering
Authors: G. Liang, A. Bauer, N. Meyners, L. Mueller
Course: PlanEx SoSe 20
License: MIT

Description:
Orientation estimation with Extended Kalman Filter and Inertial Sensor Fusion of (acc,mag) and (gyro). Adjust tuning
parameters if needed.
"""

from kalman import quaternion_tools as qtool
import numpy as np

def ekf_ori_estimation(P, rate, gyr_pre, quat_pre, acc, mag):
    """
    EKF from sensor data. Returns orientation as quaternion and uncertainty of current estimation P.

    :param P: Uncertainty of estimation of previous iteration
    :type P: ndarray(4,4)
    :param rate: sampling rate
    :type rate: float
    :param gyr_pre: gyroscope readings from previous iteration
    :type gyr_pre: ndarray(3,)
    :param quat_pre: estimated orientation x_hat from previous
    :type quat_pre: ndarray(4,)
    :param acc: accelerometer readings from current iteration
    :type acc: ndarray(3,)
    :param mag: magnetometer readings from current iteration
    :type mag: ndarray(3,)

    :returns: x_hat, P
    :rtype x_hat: ndarray(4,)
    :rtype P: ndarray(4,4)
    """

    # rotate sensor inputs 180deg around z
    gyr_pre = qtool.vec_rotate_z(np.pi, gyr_pre)
    acc = qtool.vec_rotate_z(np.pi, acc)
    mag = qtool.vec_rotate_z(np.pi, mag)

    # tuning parameters/covariance matrices
    V = np.eye(4) * 10 # gyro uncertainty
    W = np.zeros((6, 6))
    W[:3, :3] = np.eye(3) * 1000  # acc uncertainty
    W[3:6, 3:6] = np.eye(3) * 100  # mag uncertainty

    q_gyr = qtool.quaternion_from_gyr(gyr_pre, rate)

    g1, g2, g3, g4 = q_gyr
    Fx = np.zeros((4, 4))
    Fx[0, :] = [g1, -g2, -g3, -g4]
    Fx[1, :] = [g2, g1, g4, -g3]
    Fx[2, :] = [g3, -g4, g1, g2]
    Fx[3, :] = [g4, g3, -g2, g1]
    x_hat_pre = qtool.quaternion_multiply(quat_pre, q_gyr).T

    # compute Hesse matrices
    q1, q2, q3, q4 = x_hat_pre
    h1 = np.zeros((3, 4))
    h2 = np.zeros((3, 4))
    h1[0, :] = [-q3, q4, -q1, q2]
    h1[1, :] = [q2, q1, q4, q3]
    h1[2, :] = [q1, -q2, -q3, q4]
    h1 = h1 * 2 * 9.8

    h2[0, :] = [q4, q3, q2, q1]
    h2[1, :] = [q1, -q2, q3, -q4]
    h2[2, :] = [-q2, -q1, q4, q3]
    hx1 = qtool.quaternion_rotate(qtool.quaternion_invert(x_hat_pre.T), np.array([0, 0, 9.8]))
    hx2 = qtool.quaternion_rotate(qtool.quaternion_invert(x_hat_pre.T), np.array([0, 1, 0]))

    # EKF algorithm
    # predition step
    p_pre = Fx * P * Fx.T + V
    H = np.vstack([h1, h2])

    # kalman gain
    K = p_pre.dot(H.T).dot(np.linalg.inv(H.dot(p_pre.dot(H.T)) + W))
    x_hat = x_hat_pre + K.dot((np.vstack([acc, mag]).flatten() -
                               np.vstack([hx1, hx2]).flatten()).T)

    # update step
    P = (np.eye(4) - K.dot(H)).dot(p_pre.dot((np.eye(4) - K.dot(H)).T)) + K.dot(W.dot(K.T))
    # update quat
    return np.array(x_hat.T), P