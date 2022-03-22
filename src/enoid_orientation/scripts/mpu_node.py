#! /usr/bin/python3

import time
from geometry_msgs import msg

import rospy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool

from kalman import *
from mpu import *

mpu = MPU()
kalman = Kalman()

calibrate_status = False

def calibrate_callback(msg):
    global calibrate_status
    calibrate_status = msg.data


def main():
    global calibrate_status

    offset = calibrate()
    dt = 0
    angle_hat = [.0, .0, .0]
    set_point = [.0, .0, .0]

    cnt = 1
    rospy.init_node('enoid_orientation', anonymous=False)
    rate = rospy.Rate(30)
    ori_pub = rospy.Publisher('ori_data', Vector3, queue_size=1)
    gyr_pub = rospy.Publisher('gyr_data', Vector3, queue_size=1)
    acc_pub = rospy.Publisher('acc_data', Vector3, queue_size=1)
    cal_status = rospy.Subscriber("calibrate_status", Bool, calibrate_callback)

    ori_msg = Vector3()
    gyr_msg = Vector3()
    acc_msg = Vector3()
    rospy.loginfo("E-NOID ORIENTATION")

    while not rospy.is_shutdown():

        if(calibrate_status == False):
            start = time.time()

            gyro, acc, angle = read_data(offset)
            
            roll_dot = gyro[0] + sin(angle_hat[0]) * tan(angle_hat[1]) * gyro[1] + cos(angle_hat[0]) * tan(angle_hat[1]) * gyro[2]
            pitch_dot = cos(angle_hat[0]) * gyro[1] - sin(angle_hat[0]) * gyro[2]

            u = np.array([[roll_dot], [pitch_dot]])
            z = np.array([[angle[0]], [angle[1]]])

            kalman.state_prediction(dt, u)
            kalman.state_update(z)

            result = kalman.get_state()
            angle_hat[0] = result[0]
            angle_hat[1] = result[2]

            gyr_msg.x = gyro[0]
            gyr_msg.y = gyro[1]
            gyr_msg.z = gyro[2]

            acc_msg.x = acc[0]
            acc_msg.y = acc[1]
            acc_msg.z = acc[2]
    
            if(cnt < 100):
                set_point[0] +=  angle_hat[0]
                set_point[1] +=  angle_hat[1]
                cnt += 1
            elif(cnt == 100):
                set_point[0] /= 100
                set_point[1] /= 100
                cnt += 1
            else:
                ori_msg.x = (angle_hat[0] - set_point[0])
                ori_msg.y = (angle_hat[1] - set_point[1])

            ori_pub.publish(ori_msg)
            gyr_pub.publish(gyr_msg)
            acc_pub.publish(acc_msg)
            rate.sleep()
            stop = time.time()
            dt = (stop - start)
        else:
            offset = calibrate()


def read_data(offset):
    gyro = mpu.get_gyro_data()
    accel = mpu.get_accel_data()
    angle = mpu.get_angle_data()

    gyr = [gyro['x'], gyro['y'], gyro['z']]
    acc = [accel['x'], accel['y'], accel['z']]
    ang = [angle['x'] - offset['x'], angle['y'] - offset['y'], angle['z'] - offset['z']]

    return gyr, acc, ang


def calibrate():
    global calibrate_status

    rospy.loginfo("CALIBRATING IMU")
    x = 0.0
    y = 0.0
    z = 0.0

    for i in range(100):
        angle = mpu.get_angle_data()
        x -= angle['x']
        y -= angle['y']
        z -= angle['z']
        time.sleep(0.05)

    x /= 100
    y /= 100
    z /= 100
    rospy.loginfo("CALIBRATING FINISHED")

    calibrate_status = False

    return {'x': x, 'y': y, 'z': z}


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass