#!/bin/python
import csv
import numpy as np
from pathlib import Path
import pandas as pd

class EkfEstimation:
    def __init__(self, df):
        self.df = self.cleanup_df(df)

        self.motion_start = 0
        self.motion_end = self.df.shape[0]

        self.gyro_bias()

        0

    def cleanup_df(self, df):
        # determine length
        n = df.angular_velocity_x.size
        df_clean = pd.DataFrame(data={
            "time": df.time_seconds+df.time_nseconds/1e9 - df.time_seconds.iloc[0],
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


if __name__ == "__main__":
    fpath = Path("kalman/data")
    df = pd.read_csv(fpath / "test_ds.csv", sep="\t")


    EkfEstimation(df)
    0