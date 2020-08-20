#!/bin/python
import numpy as np
from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

from kalman import quaternion_tools as qtool
from kalman import ori_estimation
from kalman import pos_estimation


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

        self.pos_wrapper()

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

    def ekf_wrapper(self):
        n = self.df.acc_x.shape[0]
        ds_gyr = self.df[['gyro_x', 'gyro_y', 'gyro_z']].to_numpy()
        ds_acc = self.df[['acc_x', 'acc_y', 'acc_z']].to_numpy()
        ds_mag = self.df[['mag_x', 'mag_y', 'mag_z']].to_numpy()
        # initial first state from accmag

        quat = np.array([qtool.quaternion_from_accmag(ds_acc[1,], ds_mag[1,]).T])
        # quat = np.array([qtool.quaternion_from_accmag(
        #     qtool.vec_rotate_z(np.pi / 2, qtool.vec_rotate_x(np.pi, ds_acc[1,])),
        #     qtool.vec_rotate_z(np.pi / 2, qtool.vec_rotate_x(np.pi, ds_mag[1,]))).T])
        # euler = np.array([qtool.quat2euler(quat[0])])

        for i in range(1, ds_gyr.shape[0]):
            if i == 1:
                P = np.eye(4)
                quat_post, P = ori_estimation.ekf_ori_estimation(P, self.rate, ds_gyr[i - 1,], quat[0,], ds_acc[i,],
                                                                 ds_mag[i,])
            else:
                quat_post, P = ori_estimation.ekf_ori_estimation(P, self.rate, ds_gyr[i - 1,], quat[i - 1,], ds_acc[i,],
                                                                 ds_mag[i,])
            quat = np.append(quat, [quat_post], axis=0)
            # euler = np.append(euler, [qtool.quat2euler(quat_post)], axis=0)
        self.quat = quat
        0
        # self.euler = euler

    def pos_wrapper(self):
        acc = self.df[['acc_x', 'acc_y', 'acc_z']].to_numpy()
        vel_list = []
        pos_list = []
        vel_sum = np.array([0, 0, 0])
        pos_sum = np.array([0, 0, 0])
        quat = self.quat
        # quat = self.real_quat.drop("time",axis=1).to_numpy()
        for i in range(quat.shape[0]):
            pos_sum, vel_sum = pos_estimation.pos_estimation(self.rate, quat[i,], acc[i,], vel_sum, pos_sum)
            vel_list.append(vel_sum)
            pos_list.append(pos_sum)
        vel_list = np.array(vel_list)
        pos_list = np.array(pos_list)

        # plot
        # 3D position plot
        fig = plt.figure()
        axes = plt.axes()
        axes.set_xlabel('x')
        axes.set_ylabel('y')

        line = axes.plot(pos_list[:, 0], pos_list[:, 1], 'green')
        plt.axis('equal')
        plt.show()
        0


if __name__ == "__main__":
    fpath = Path("data")
    # fname = "linear_vel2.csv"
    # fname = "test_data.csv"
    fname = "turtle2.csv"
    df = pd.read_csv(fpath / fname, sep="\t")

    call = EkfEstimation(df)
    foo = call._return()
    0
