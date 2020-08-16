#!/bin/python
import numpy as np
from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

from kalman import quaternion_tools as qtool


class EkfEstimation:
    def __init__(self, df):
        self.df = self.cleanup_df(df)
        self.real_quat = self.real_df(df)

        self.motion_start = 70
        self.motion_end = 500
        self.rate = 50

        self.gyro_bias()

        # extended kalman filter, orientation estimation
        self.ekf_wrapper()
        self.pos_estimation()

    def _return(self):
        return self.quat

    def real_df(self, df):
        df_real = pd.DataFrame(data={
            "time": df.time_seconds + df.time_nseconds / 1e9 - df.time_seconds.iloc[0],
            "quat_w": df.orientation_w,
            "quat_x": df.orientation_x,
            "quat_y": df.orientation_y,
            "quat_z": df.orientation_z
        })
        return df_real

    def cleanup_df(self, df):
        # determine length
        n = df.angular_velocity_x.size
        df_clean = pd.DataFrame(data={
            "time": df.time_seconds + df.time_nseconds / 1e9 - df.time_seconds.iloc[0],
            "gyro_x": df.angular_velocity_x,
            "gyro_y": df.angular_velocity_y,
            "gyro_z": df.angular_velocity_z,
            "acc_x": df.linear_acceleration_x,
            "acc_y": df.linear_acceleration_y,
            "acc_z": df.linear_acceleration_z,
            # generate mag data (until simulation data is available)
            "mag_x": df.mag_x,
            "mag_y": df.mag_y,
            "mag_z": df.mag_z})
        return df_clean

    def gyro_bias(self):
        # gyr bias is 0
        self.ds_gyrbias = pd.DataFrame(data={
            "gyro_x": np.zeros(self.df.gyro_x.shape[0]),
            "gyro_y": np.zeros(self.df.gyro_x.shape[0]),
            "gyro_z": np.zeros(self.df.gyro_x.shape[0])
        })
        self.df.gyro_x = self.df.gyro_x - self.ds_gyrbias.gyro_x
        self.df.gyro_y = self.df.gyro_y - self.ds_gyrbias.gyro_y
        self.df.gyro_z = self.df.gyro_z - self.ds_gyrbias.gyro_z

    def quat2euler(self, quat):
        alpha = np.arctan2(2 * (quat[0]*quat[3] + quat[1] * quat[2]), 1 - 2 * (quat[2]**2 + quat[3]**2))
        beta = np.arcsin(2 * quat[0] * quat[2] - quat[3] * quat[1])
        gamma = np.arctan2(2 * (quat[0]*quat[1] + quat[2] * quat[3]), 1 - 2 * (quat[1]**2 + quat[2]**2))

        return np.array([alpha, beta, gamma])


    def ekf_wrapper(self):
        n = self.df.acc_x.shape[0]
        ds_gyr = self.df[['gyro_x', 'gyro_y', 'gyro_z']].to_numpy()
        ds_acc = self.df[['acc_x', 'acc_y', 'acc_z']].to_numpy()
        ds_mag = self.df[['mag_x', 'mag_y', 'mag_z']].to_numpy()
        quat = np.array([[1, 0, 0, 0]])
        euler = np.array([self.quat2euler(quat[0])])
        # quat[0,] = qtool.quaternion_from_accmag(ds_acc[0,], ds_mag[0,]).T

        for i in range(1, ds_gyr.shape[0]):
            if i == 1:
                quat_post = self.ekf_ori_estimation(self.rate, ds_gyr[i - 1,], quat[0,], ds_acc[i,], ds_mag[i,])
            else:
                quat_post = self.ekf_ori_estimation(self.rate, ds_gyr[i-1,], quat[i-1,], ds_acc[i,], ds_mag[i,])
            quat = np.append(quat, [quat_post], axis=0)
            euler = np.append(euler, [self.quat2euler(quat_post)], axis=0)
        self.quat = quat
        self.euler = euler
        0

    def ekf_ori_estimation(self, rate, gyr_pre, quat_pre, acc, mag):
        P = np.eye(4)
        V = np.eye(4) * 0.1
        W = np.zeros((6, 6))
        W[:3, :3] = np.eye(3) * 100000  # acc uncertanty
        W[3:6, 3:6] = np.eye(3) * 100000  # mag uncertanty

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
        # predition
        p_pre = Fx * P * Fx.T + V
        H = np.vstack([h1, h2])
        # kalman gain
        K = p_pre.dot(H.T).dot(np.linalg.inv(H.dot(p_pre.dot(H.T)) + W))
        x_hat = x_hat_pre + K.dot((np.vstack([acc, mag]).flatten() -
                                   np.vstack([hx1, hx2]).flatten()).T)
        # update P
        P = (np.eye(4) - K.dot(H)).dot(p_pre.dot((np.eye(4) - K.dot(H)).T)) + K.dot(W.dot(K.T))
        # update quat
        # quat[i,] = x_hat.T
        return x_hat.T

    def pos_estimation(self):
        # position estimation
        est_gravity = np.array([0, 0, np.mean(self.df.acc_z[0:self.motion_start])])
        ds_acc = self.df[['acc_x', 'acc_y', 'acc_z']].to_numpy()
        ds_acc_corr = np.array(
            [qtool.quaternion_rotate(self.quat[i,], ds_acc[i,]) for i in range(0, self.quat.shape[0])])
        # no gravity
        ds_acc_corr = ds_acc_corr - np.ones(ds_acc_corr.shape) * est_gravity
        velocity = np.cumsum(ds_acc_corr, axis=1) / self.rate
        self.pos_kin = np.cumsum(velocity, axis=1) / self.rate

        # 3D position plot
        fig = plt.figure()
        axes = plt.axes(projection='3d')
        axes.set_xlabel('x')
        axes.set_ylabel('y')
        axes.set_zlabel('z')
        # sp = axes.scatter3D(start_pt[0], start_pt[1], start_pt[2], 'red')
        line = axes.plot3D(self.pos_kin[:, 0], self.pos_kin[:, 1], self.pos_kin[:, 2], 'green')
        plt.axis('equal')
        plt.show()
        0


if __name__ == "__main__":
    fpath = Path("data")
    # fname = "linear_vel2.csv"
    fname = "test_data2.csv"
    df = pd.read_csv(fpath / fname, sep="\t")

    call = EkfEstimation(df)
    foo = call._return()
    0
