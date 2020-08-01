#!/bin/python
import numpy as np
from pathlib import Path
import pandas as pd

from kalman import quaternion_tools as qtool


class EkfEstimation:
    def __init__(self, df):
        self.df = self.cleanup_df(df)

        self.motion_start = 0
        self.motion_end = self.df.shape[0]
        self.rate = 10

        self.gyro_bias()

        self.ekf_ori_estimation()
        0

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
            "mag_x": np.random.normal(1, 0.05, n),
            "mag_y": np.random.normal(0.05, 0.01, n),
            "mag_z": np.random.normal(0.01, 0.001, n)})
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

    def ekf_ori_estimation(self):
        n = self.df.acc_x.shape[0]
        quat = np.zeros((n, 4))
        P = np.eye(4)
        V = np.eye(4) * 0.00001
        W = np.zeros((6, 6))
        W[:3, :3] = np.eye(3) * 10
        W[3:6, 3:6] = np.eye(3) * 0.01
        ds_gyr = self.df[['gyro_x', 'gyro_y', 'gyro_z']].to_numpy()
        ds_acc = self.df[['acc_x', 'acc_y', 'acc_z']].to_numpy()
        ds_mag = self.df[['mag_x', 'mag_y', 'mag_z']].to_numpy()

        Fx = np.zeros((4, 4))
        for i in range(1, ds_gyr.shape[0]):
            q_gyr = qtool.quaternion_from_gyr(ds_gyr[i - 1,], self.rate)
            # while waiting for sim data, generate gyr quat
            q_gyr = np.hstack(np.array(
                [np.random.random(1), np.random.random(1), np.random.random(1), np.random.random(1)])).flatten()
            g1, g2, g3, g4 = q_gyr
            Fx[0, :] = [g1, -g2, -g3, -g4]
            Fx[1, :] = [g2, g1, g4, -g3]
            Fx[2, :] = [g3, -g4, g1, g2]
            Fx[3, :] = [g4, g3, -g2, g1]
            q_accmag = qtool.quaternion_from_accmag(ds_acc[i,], ds_mag[i,])

            # while waiting for sim data, generate accmag quat
            q_accmag = np.hstack(np.array(
                [np.random.random(1), np.random.random(1), np.random.random(1), np.random.random(1)])).flatten()
            x_hat_pre = qtool.quaternion_multiply(quat[i - 1,], q_gyr)

            # compute Hesse matrix
            q1, q2, q3, q4 = x_hat_pre
            h1 = np.zeros((3, 4))
            h2 = np.zeros((3, 4))
            h1[0, :] = [-q3, q4, -q1, q2]
            h1[1, :] = [q2, q1, q4, q3]
            h1[2, :] = [q1, -q2, -q3, q4]
            h1 = h1 * 2 * 9.81

            h2[0, :] = [q4, q3, q2, q1]
            h2[1, :] = [q1, -q2, q3, -q4]
            h2[2, :] = [-q2, -q1, -q4, q3]
            hx1 = qtool.quaternion_rotate(qtool.quaternion_invert(x_hat_pre.T), np.array([0, 0, 9.8]))
            hx2 = qtool.quaternion_rotate(qtool.quaternion_invert(x_hat_pre.T), np.array([0, 1, 0]))

            # EKF algorithm
            p_pre = Fx * P * Fx.T + V
            H = np.vstack([h1, h2])
            # kalman gain
            K = p_pre.dot(H.T).dot(np.linalg.inv(H.dot(p_pre.dot(H.T)) + W))
            x_hat = x_hat_pre + K.dot((np.vstack([ds_acc[i,], ds_mag[i, ]]).flatten() -
                                     np.vstack([hx1, hx2]).flatten()).T)
            # update P
            P = (np.eye(4) - K.dot(H)).dot(p_pre.dot((np.eye(4) - K.dot(H)).T)) + K.dot(W.dot(K.T))
            # update quat
            quat[i,] = x_hat.T
        self.quat = quat



if __name__ == "__main__":
    fpath = Path("kalman/data")
    df = pd.read_csv(fpath / "test_ds.csv", sep="\t")

    EkfEstimation(df)
    0
