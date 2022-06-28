"""
Cubic Spline library on python

Based On Atsushi Sakai

"""
import bisect
import math
import numpy as np
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped,Pose,Point, Quaternion

class Spline:
    """
    Cubic Spline class
    """

    def __init__(self, dt, x, y,):
        self.bx, self.by, self.cx,self.cy,self.dx, self.dy, self.w = [],[],[],[],[],[],[]

        self.x = x
        self.y = y
        self.dt = dt

        self.nx = len(dt)  # dimension of x
        h = np.diff(dt)

        # calc coefficient c
        self.ax = [ix for ix in x]
        self.ay = [iy for iy in y]

        # calc coefficient c
        A = self.__calc_A(h)
        Bx = self.__calc_Bx(h)
        By = self.__calc_By(h)
        self.cx = np.linalg.solve(A, Bx)
        self.cy = np.linalg.solve(A, By)

        # calc spline coefficient b and d
        for i in range(self.nx - 1):
            self.dx.append((self.cx[i + 1] - self.cx[i]) / (3.0 * h[i]))
            self.dy.append((self.cy[i + 1] - self.cy[i]) / (3.0 * h[i]))
            tbx = (self.ax[i + 1] - self.ax[i]) / h[i] - h[i] * \
                 (self.cx[i + 1] + 2.0 * self.cx[i]) / 3.0
            tby = (self.ay[i + 1] - self.ay[i]) / h[i] - h[i] * \
                 (self.cy[i + 1] + 2.0 * self.cy[i]) / 3.0
            self.bx.append(tbx)
            self.by.append(tby)

    def calc_x(self, t):
        """
        Calc position

        if t is outside of the input x, return None

        """

        if t < self.dt[0]:
            return None
        elif t > self.dt[-1]:
            return None

        i = self.__search_index(t)
        dtime = t - self.dt[i]
        result = self.ax[i] + self.bx[i] * dtime + self.cx[i] * dtime ** 2.0 + self.dx[i] * dtime ** 3.0

        return result

    def calc_y(self, t):
        """
        Calc position

        if t is outside of the input x, return None

        """

        if t < self.dt[0]:
            return None
        elif t > self.dt[-1]:
            return None

        i = self.__search_index(t)

        dtime = t - self.dt[i]
        result = self.ay[i] + self.by[i] * dtime + self.cy[i] * dtime ** 2.0 + self.dy[i] * dtime ** 3.0

        return result

    def __search_index(self, dtime):
        return bisect.bisect(self.dt, dtime) - 1

    def __calc_A(self, h):
        A = np.zeros((self.nx, self.nx))
        A[0, 0] = 1.0
        for i in range(self.nx - 1):
            if i != (self.nx - 2):
                A[i + 1, i + 1] = 2.0 * (h[i] + h[i + 1])
            A[i + 1, i] = h[i]
            A[i, i + 1] = h[i]

        A[0, 1] = 0.0
        A[self.nx - 1, self.nx - 2] = 0.0
        A[self.nx - 1, self.nx - 1] = 1.0
        #  print(A)
        return A

    def __calc_Bx(self, h):
        """
        calc matrix B for spline coefficient c
        """
        Bx = np.zeros(self.nx)
        for i in range(self.nx - 2):
            Bx[i + 1] = 3.0 * (self.ax[i + 2] - self.ax[i + 1]) / \
                       h[i + 1] - 3.0 * (self.ax[i + 1] - self.ax[i]) / h[i]
        #  print(B)
        return Bx

    def __calc_By(self, h):
        """
        calc matrix B for spline coefficient c
        """
        By = np.zeros(self.nx)
        for i in range(self.nx - 2):
            By[i + 1] = 3.0 * (self.ay[i + 2] - self.ay[i + 1]) / \
                       h[i + 1] - 3.0 * (self.ay[i + 1] - self.ay[i]) / h[i]
        #  print(B)
        return By

    def cubic_spline(self):
        dt = np.arange(0,self.nx-1,0.1)

        seq = 0
        x_c = 0
        y_c = 0

        path = Path()
        
        for t in dt:
            xc = self.calc_x(t)
            yc = self.calc_y(t)

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