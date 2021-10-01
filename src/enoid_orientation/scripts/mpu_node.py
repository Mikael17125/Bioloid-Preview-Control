#! /usr/bin/python3

import time

import rospy
from geometry_msgs.msg import Vector3

from kalman import *
from mpu import *

mpu = MPU(0x68)
kalman = Kalman()


def main():
    offset = calibrate()
    dt = 0
    angle_hat = [.0, .0, .0]

    rospy.init_node('enoid_orientation', anonymous=False)
    rate = rospy.Rate(30)
    pub = rospy.Publisher('orientation_data', Vector3, queue_size=1)
    msg = Vector3()

    while not rospy.is_shutdown():
        start = time.time()

        gyro, acc, angle = read_data(offset)

        roll_dot = gyro[0] + sin(angle_hat[0]) * tan(angle_hat[1]) * gyro[1] + cos(angle_hat[0]) * tan(angle_hat[1]) * \
                   gyro[2]
        pitch_dot = cos(angle_hat[0]) * gyro[1] - sin(angle_hat[0]) * gyro[2]

        u = np.array([[roll_dot], [pitch_dot]])
        z = np.array([[angle[0]], [angle[1]]])

        kalman.state_prediction(dt, u)
        kalman.state_update(z)

        result = kalman.get_state()
        angle_hat[0] = result[0]
        angle_hat[1] = result[2]

        msg.x = angle_hat[0]
        msg.y = angle_hat[1]

        pub.publish(msg)
        rate.sleep()
        stop = time.time()
        dt = (stop - start)


def read_data(offset):
    gyro = mpu.get_gyro_data()
    accel = mpu.get_accel_data()
    angle = mpu.get_angle_data()

    gyr = [gyro['x'], gyro['y'], gyro['z']]
    acc = [accel['x'], accel['y'], accel['z']]
    ang = [angle['x'] - offset['x'], angle['y'] - offset['y'], angle['z'] - offset['z']]

    return gyr, acc, ang


def calibrate():
    x, y, z = .0

    for i in range(100):
        angle = mpu.get_angle_data()
        x -= angle['x']
        y -= angle['y']
        z -= angle['z']
        time.sleep(0.05)

    x /= 100
    y /= 100
    z /= 100

    return {'x': x, 'y': y, 'z': z}


if __name__ == '__main__':
    main()
