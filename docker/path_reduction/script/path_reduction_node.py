#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose
import numpy as np
import os

ori_path_msg = Path()
proc_path_msg = Path()
red_path_msg = Path()

ori_path_pub = rospy.Publisher('/path_original_pub', Path, queue_size=10)
proc_path_pub = rospy.Publisher('/proc_original_pub', Path, queue_size=10)
red_path_pub = rospy.Publisher('/red_original_pub', Path, queue_size=10)

x_max_tmp = -999
y_max_tmp = -999
x_min_tmp = 999
y_min_tmp = 999

n = int(os.environ["N_POINT"])
def reduction_path(ori_path_msg):

    filter_path(ori_path_msg)
    step = len(proc_path_msg.poses)// n
    if(len(proc_path_msg.poses) >= n):
        for i in range(0, len(proc_path_msg.poses), int(step)):
            red_path_msg.header.frame_id = "vslam"
            red_path_msg.header.stamp = rospy.Time.now()
            red_path_msg.poses.append(proc_path_msg.poses[i]) 
        red_path_pub.publish(red_path_msg)
        red_path_msg.poses.clear()

def filter_path(ori_path_msg):
    global x_max_tmp,y_max_tmp,x_min_tmp,y_min_tmp

    n_ori = len(ori_path_msg.poses)

    for i in range(n_ori):
        if(x_max_tmp < ori_path_msg.poses[i].pose.position.x) or (x_min_tmp > ori_path_msg.poses[i].pose.position.x) or (y_max_tmp < ori_path_msg.poses[i].pose.position.y) or (y_min_tmp > ori_path_msg.poses[i].pose.position.y):
            if(x_max_tmp < ori_path_msg.poses[i].pose.position.x):
                x_max_tmp = ori_path_msg.poses[i].pose.position.x
            if(x_min_tmp > ori_path_msg.poses[i].pose.position.x):
                x_min_tmp = ori_path_msg.poses[i].pose.position.x
            if(y_max_tmp < ori_path_msg.poses[i].pose.position.y):
                y_max_tmp = ori_path_msg.poses[i].pose.position.y
            if(y_min_tmp > ori_path_msg.poses[i].pose.position.y):
                y_min_tmp = ori_path_msg.poses[i].pose.position.y
            proc_path_msg.header.frame_id = "vslam"
            proc_path_msg.header.stamp = rospy.Time.now()
            proc_path_msg.poses.append(ori_path_msg.poses[i])        
 
    
def pose_callback(data):
    ori_path_msg.header = data.header
    ori_path_msg.poses.append(data)
    reduction_path(ori_path_msg)

def main():

    rospy.Subscriber("/vslam2d_pose", PoseStamped, pose_callback)
    rospy.init_node('path_reduction_node', anonymous=True)
    rate = rospy.Rate(10)
     
    while not rospy.is_shutdown():
        proc_path_pub.publish(proc_path_msg)
        ori_path_pub.publish(ori_path_msg)
       
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass