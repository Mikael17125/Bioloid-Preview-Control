#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped,Pose,Point, Quaternion

class Lagrange():
    def __init__(self, dt, x, y):
        self.n = len(dt)
        self.dt = dt
        self.x = x
        self.y = y

    def lagrange(self,t,point):
        yp = 0
        for i in range(self.n):
                p = 1
                for j in range(self.n):
                    if i != j:
                        p = p * (t - self.dt[j]) / (self.dt[i] - self.dt[j])

                yp = yp + p * point[i]
                
        return yp

    def lagrange_interpolation(self):
        dt = np.linspace(0,self.n-1,100)

        seq = 0
        x_c = 0
        y_c = 0
        path = Path()
        
        for t in dt:
            xc = self.lagrange(t,self.x)
            yc = self.lagrange(t,self.y)

            poses = PoseStamped()
            poses.header.seq = seq
            poses.header.stamp = rospy.Time.now()
            poses.header.frame_id = "map"
            poses.pose.position.x = xc
            poses.pose.position.y = yc
            poses.pose.position.z = 0
            
            path.header.seq = seq
            path.header.stamp = rospy.Time.now()
            path.header.frame_id = "map"

            path.poses.append(poses)

            seq += 1
        
        return path
