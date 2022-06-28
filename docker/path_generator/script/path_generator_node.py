#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from lagrange import *
from cubic_spline import *
import numpy as np
import os

dt = np.array([0,1,2,3,4])
x = np.array([-3, 2, -2, -1, 1])
y = np.array([3, 2,  1, -1, 5])

dt1 = np.array([0,1,2,3,4])
x1 = np.array([0, 4, -2, 1, 0])
y1 = np.array([1, 2,  -4, 1, -3])

def main():
    global x,y
    path_pub = rospy.Publisher('/path_publisher', Path, queue_size=10)
    marker_pub = rospy.Publisher('/marker_publisher', Marker, queue_size=10)

    rospy.init_node('path_generator_node', anonymous=True)
    rate = rospy.Rate(10)
    

    path_msg = Path()
    if(os.environ["SPLINE"] == "0"):
        rospy.loginfo("LAGRANGE INTERPOLATION")
        if(os.environ["COORD"] == "0"):
            spline = Lagrange(dt,x,y)
            path_msg = spline.lagrange_interpolation()
        else:
            spline = Lagrange(dt1,x1,y1)
            path_msg = spline.lagrange_interpolation()
    else:
        rospy.loginfo("CUBIC SPLINE")
        if(os.environ["COORD"] == "0"):
            spline = Spline(dt,x,y)
            path_msg = spline.cubic_spline()
        else:
            spline = Spline(dt1,x1,y1)
            path_msg = spline.cubic_spline()

    marker_msg = Marker()
 
    while not rospy.is_shutdown():
        for i in range(0,len(dt)):
            marker_msg.header.frame_id = "map"
            marker_msg.header.stamp = rospy.Time.now()
            marker_msg.ns = "ns"
            marker_msg.id = i
            marker_msg.type = 2
            marker_msg.action = 0
            if(os.environ["COORD"] == "0"):
                marker_msg.pose.position.x = x[i]
                marker_msg.pose.position.y = y[i]
            else:
                marker_msg.pose.position.x = x1[i]
                marker_msg.pose.position.y = y1[i]
            marker_msg.pose.position.z = 0
            marker_msg.pose.orientation.x = 0
            marker_msg.pose.orientation.y = 0
            marker_msg.pose.orientation.z = 0
            marker_msg.pose.orientation.w = 1
            marker_msg.scale.x = 0.25
            marker_msg.scale.y = 0.25
            marker_msg.scale.z = 0.25
            marker_msg.color.a = 1.0 
            marker_msg.color.r = 1.0
            marker_msg.color.g = 0.0
            marker_msg.color.b = 0.0
            marker_pub.publish(marker_msg)

        path_pub.publish(path_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
