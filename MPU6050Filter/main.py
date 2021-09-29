from mpu6050 import mpu6050
import math
import time

class MPU:
    RAD2DEG = 180/math.pi
    DEG2RAD = math.pi/180

    def __init__(self):
        self.mpu = mpu6050(0x68)
        #Set Kalman Init Angle
        #-----------#

    def get_acc(self):
        acc = [.0,.0,.0]

        accel_data = self.mpu.get_accel_data()
        acc[0] = accel_data['x']
        acc[1] = accel_data['y']
        acc[2] = accel_data['z']

        return acc

    def get_gyr(self):
        gyr = [.0, .0, .0]

        gyro_data = self.mpu.get_gyro_data()
        gyr[0] = gyro_data['x']
        gyr[1] = gyro_data['y']
        gyr[2] = gyro_data['z']

        return gyr

    def get_angle(self):
        ori = [.0,.0,.0]

        acc = self.get_acc()

        ori[0] = math.atan2(-acc[1], acc[2]) * self.RAD2DEG
        ori[1] = math.atan2(acc[0], acc[2]) * self.RAD2DEG

        return ori

    def calibrate(self):
        gyr_cal = [.0,.0,.0]

        for i in range(100):
            gyr = self.get_gyr()
            gyr_cal[0] -= gyr[0]
            gyr_cal[1] -= gyr[1]
            gyr_cal[2] -= gyr[2]
            time.sleep(0.05)

        gyr_cal /= 100

        return gyr_cal