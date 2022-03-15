#! /usr/bin/python3

import warnings
warnings.simplefilter("ignore", UserWarning)

import rospy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool
import numpy as np
import pickle

ori_data = np.array([.0, .0, .0])
gyr_data = np.array([.0, .0, .0])
acc_data = np.array([.0, .0, .0])

push_msg = Bool()

def ori_callback(msg):
    ori_data[0] = msg.x
    ori_data[1] = msg.y
    ori_data[2] = msg.z


def gyr_callback(msg):
    gyr_data[0] = msg.x 
    gyr_data[1] = msg.y
    gyr_data[2] = msg.z

def acc_callback(msg):
    acc_data[0] = msg.x 
    acc_data[1] = msg.y
    acc_data[2] = msg.z

def main():
    global ori_data, acc_data, gyr_data
    rospy.init_node('enoid_fall', anonymous=False)
    rospy.Subscriber("ori_data", Vector3, ori_callback)
    rospy.Subscriber("gyr_data", Vector3, gyr_callback)
    rospy.Subscriber("acc_data", Vector3, acc_callback)
    push_pub = rospy.Publisher('push_data', Bool, queue_size=1)


    rate = rospy.Rate(30)

    model = pickle.load(open('/home/mikael/catkin_ws/src/enoid_detection/scripts/KNN', 'rb'))

    rospy.loginfo("E-NOID FALL DETECTION")
    push = 0

    while not rospy.is_shutdown():
        
        predict = model.predict([[ori_data[0], ori_data[1], gyr_data[0], gyr_data[1], gyr_data[2], acc_data[0], acc_data[1], acc_data[2]]])

        # print(predict)
        if(predict == 0):
            push = 0
        elif(predict == 1):
            push = 0
        elif(predict == 2):
            push += 1

        if push >= 7:
            # print("PUSH")
            push_msg.data = True
        else:
            push_msg.data = False

        push_pub.publish(push_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
