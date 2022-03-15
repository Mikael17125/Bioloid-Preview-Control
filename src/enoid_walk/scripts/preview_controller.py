import numpy as np
import control
import scipy.linalg as la
import matplotlib.pyplot as plt
from pytransform3d.trajectories import *

debug = 1

class PreviewControl:
    def __init__(self, com_height, com_swing, x_offset, foot_distance):

        self.dt = 0.01
        self.previewStepNum = 120
        self.zc = com_height
        self.g = 9.8
        self.x_offset = x_offset
        self.com_swing = com_swing
        self.foot_y = foot_distance + (0.0385)
        self.first = False

        # Kajita's Book P:144 Eq:4.72
        self.A = np.matrix([[1,self.dt, (self.dt ** 2) / 2],
                            [0, 1, self.dt],
                            [0, 0, 1]])
        self.B = np.matrix([(self.dt ** 3) / 6, (self.dt ** 2) / 2, self.dt]).T
        self.C = np.matrix([1, 0, -self.zc / self.g])

        # Kajita's Book P:145 Eq:4.77
        self.A_tilde = np.hstack((np.matrix([1, 0, 0, 0]).T, np.vstack((self.C * self.A, self.A))))
        self.B_tilde = np.vstack((self.C * self.B, self.B))
        self.C_tilde = np.matrix([1., 0., 0., 0.])

        self.Q = np.matrix([[1,0,0,0],
                            [0,0,0,0],
                            [0,0,0,0],
                            [0,0,0,0]])

        self.R = np.matrix([1e-7])

        # Kajita's Book P:144 Eq:4.75
        P, _, K = control.dare(self.A_tilde, self.B_tilde, self.Q,self.R)

        # Kajita's Book P:146 Eq:4.80
        self.Ks = K[0,0]
        self.Kx = K[0,1:4]

        ######==========GAIN=============######
        self.G = np.zeros(self.previewStepNum)
        self.G[0] =-self.Ks

        Ac_tilde = self.A_tilde - self.B_tilde * K
        I_tilde = np.matrix([1., 0., 0., 0.]).T
        X_tilde = -Ac_tilde.T * P * I_tilde

        for i in range(1, self.previewStepNum):
            self.G[i] = la.inv(self.R + self.B_tilde.T * P * self.B_tilde) * self.B_tilde.T * X_tilde
            X_tilde = Ac_tilde.T * X_tilde
        ######===========================######

        self.cnt = 1
        #Jarak dari Hip ke CoM
        self.hip_offset = 0.0385 + self.com_swing

        #Prediksi State
        self.x = np.matrix(np.zeros(3)).T
        self.y = np.matrix(np.zeros(3)).T

        #ZMP State
        self.px_ref = []
        self.py_ref = []

        #Foot Step (Kenapa Butuh 3)
        self.footstep = [[0.0, -self.hip_offset],
                         [0.0, self.hip_offset],
                         [0.0, -self.hip_offset]]

        self.support_foot = -1

        self.com_pose = np.matrix([0.0, 0.0, 0.0], dtype=float)
        self.l_foot_pose = np.matrix([0.0, self.hip_offset, 0.0], dtype=float)
        self.r_foot_pose = np.matrix([0.0, -self.hip_offset, 0.0], dtype=float)
        self.cur_l_foot_pose = np.matrix([0.0, self.hip_offset, 0.0], dtype=float)
        self.cur_r_foot_pose = np.matrix([0.0, -self.hip_offset, 0.0], dtype=float)
        self.r = []
        self.l = []
        # Bezier Control Point
        self.p_start = np.matrix([0.0, 0.0, 0.0], dtype=float)
        self.p_cnt = np.matrix([0.0, 0.0, 0.0], dtype=float)
        self.p_end = np.matrix([0.0, 0.0, 0.0], dtype=float)

        #Parameter
        self.cmd_x = 0.025
        self.cmd_y = 0.0
        self.cmd_a = np.radians(0)

        self.sx = 0.0
        self.sy = 0.0
        self.sa = 0.0

        self.swing_height = 0.035

        # Walking Timing Pzrameter
        self.t_step = 0.3
        self.dsp_ratio = 0.1
        self.t_dsp = self.dsp_ratio * self.t_step
        self.t_ssp = (1.0 - self.dsp_ratio) * self.t_step
        self.t = 0.0

        self.t_bez = 0.0
        self.dt_bez = 1 / (self.t_ssp / self.dt)

        # 0 : DSP, 1 : SSP
        self.walking_phase = 0

        self.walking_ready = False

    def update_foot_trajectory(self):
        # State Untuk Update Start dan End Foot Trajectory
        if self.t == 0:
            if self.support_foot == 1: #Kiri
                self.p_start = self.cur_l_foot_pose
                self.p_end = np.matrix([self.footstep[1][0], self.footstep[1][1], 0])
                self.p_cnt = np.matrix([self.p_start[0, 0] + (self.p_end[0, 0] - self.p_start[0, 0]) / 2, -self.hip_offset,
                                        2 * self.swing_height])
            else:
                self.p_start = self.cur_r_foot_pose
                self.p_end = np.matrix([self.footstep[1][0], self.footstep[1][1], 0])
                self.p_cnt = np.matrix([self.p_start[0, 0] + (self.p_end[0, 0] - self.p_start[0, 0]) / 2, self.hip_offset,
                                        2 * self.swing_height])

                            
        # State Untuk Reset Fase Walking Bezier
        if self.t < (self.t_dsp / 2.0) or self.t >= (self.t_dsp / 2.0 + self.t_ssp):
            self.walking_phase = 0
            self.t_bez = 0
            if self.walking_ready:
                if self.support_foot == 1:
                    self.first = True
        else:

            self.walking_phase = 1
            if self.support_foot == 1: # KIRI
                self.cur_l_foot_pose[0, 0] = self.footstep[0][0]
                self.cur_l_foot_pose[0, 1] = -self.footstep[0][1]
                self.cur_l_foot_pose[0, 2] = 0

                self.r_foot_pose = self.update_foot_path()
            else:

                self.cur_r_foot_pose[0, 0] = self.footstep[0][0]
                self.cur_r_foot_pose[0, 1] = -self.footstep[0][1]
                self.cur_r_foot_pose[0, 2] = 0

                self.l_foot_pose = self.update_foot_path()

        self.l.append([self.l_foot_pose[0,0], self.foot_y, self.l_foot_pose[0,2]])
        self.r.append([self.r_foot_pose[0,0], -self.foot_y, self.r_foot_pose[0,2]])

        self.t_bez += self.dt_bez
        if(len(self.l) == self.previewStepNum):
            self.l.pop(0)
            self.r.pop(0)

        self.r_foot = np.matrix([self.r[0]])
        self.l_foot = np.matrix([self.l[0]])

    def update_foot_path(self):
        if(self.t_bez >= 1):
            self.t_bez = 1

        t = np.matrix([1,  self.t_bez,  self.t_bez**2])
        coef = np.matrix([[ 1, 0, 0],
                          [-2, 2, 0],
                          [ 1,-2, 1]])
        point = np.vstack((self.p_start, self.p_cnt, self.p_end))

        path = t * coef * point
     
        return path

    def swap_support_foot(self):
        # 1 => Kiri # -1 => Kanan
        if self.support_foot == 1:
            self.support_foot = -1
        else:
            self.support_foot = 1

    def update_footstep(self):
        if self.cnt % int(self.t_step / self.dt) == 0:
            self.footstep.pop(0)

            if self.support_foot == 1:
                self.sx = self.cmd_x
                self.sy = -2 * self.hip_offset + self.cmd_y
                self.sa += self.cmd_a
                # Kajita's Book P:132 Eq:4.61
                dx = self.footstep[-1][0] + np.cos(self.sa) * self.sx + (-np.sin(self.sa) * self.sy)
                dy = self.footstep[-1][1] + np.sin(self.sa) * self.sx + np.cos(self.sa) * self.sy
                self.footstep.append([dx, dy])
                # print(self.footstep[-1][0])
            else:
                self.sx = self.cmd_x
                self.sy = 2 * self.hip_offset + self.cmd_y
                self.sa += self.cmd_a
                # Kajita's Book P:132 Eq:4.61
                dx = self.footstep[-1][0] + np.cos(self.sa) * self.sx + (-np.sin(self.sa) * self.sy)
                dy = self.footstep[-1][1] + np.sin(self.sa) * self.sx + np.cos(self.sa) * self.sy
                self.footstep.append([dx, dy])

            self.swap_support_foot()

        self.cnt += 1

    def update_pose(self):
        # Perbaharui Trajektori Kaki
        self.update_foot_trajectory()

        # Hapus Refrensi Paling Depan Kalau Refrensi Sudah Sebanyak Yang Diinginkan
        if len(self.px_ref) == self.previewStepNum:
            self.px_ref.pop(0)
            self.py_ref.pop(0)

        # Tambah Refrensi Paling Belakang dari Footstep
        self.px_ref.append(self.footstep[0][0])
        self.py_ref.append(self.footstep[0][1])

    def update_preview_control(self):

        #Perbaharui Trajektori CoM
        xe = self.px_ref[0] - self.C * self.x
        ye = self.py_ref[0] - self.C * self.y

        G_x, G_y = 0,0

        for j in range(0, self.previewStepNum):
            G_x += self.G[j] * self.px_ref[j]
            G_y += self.G[j] * self.py_ref[j]
            self.walking_ready = True

        # Kajita's Book P:146 Eq:4.80
        ux = -self.Ks * xe - self.Kx * self.x - G_x
        uy = -self.Ks * ye - self.Kx * self.y - G_y

        # Kajita's Book P:145 Eq:4.77
        self.x = self.A * self.x + self.B * ux
        self.y = self.A * self.y + self.B * uy

        self.com_pose[0,0], self.com_pose[0,1], self.com_pose[0,2] = self.x[0,0] + self.x_offset, self.y[0,0], self.zc
        if(not self.first):
            self.com_pose -= 0.0065
    def update_walking_pattern(self):

        self.update_pose()

        if len(self.px_ref) == self.previewStepNum:
            self.update_preview_control()

        self.t += self.dt
        if self.t > self.t_step:
            self.t = 0
        # Perbaharui ZMP
        self.update_footstep()

    def run(self):

        t_sim = 8
        t = 0
        com_x = []
        com_y = []

        px = []
        py = []

        com_trajectory = []
        r_trajectory = []
        l_trajectory = []

        while t < t_sim:
            self.update_walking_pattern()
            t += self.dt
            com_x.append(self.x[0, 0])
            com_y.append(self.y[0, 0])

            px.append(self.px_ref[0])
            py.append(self.py_ref[0])

            com_trajectory.append([self.x[0, 0],self.y[0, 0],0.19,0,0,0,0])

            r_trajectory.append([self.r_foot[0,0], self.r_foot[0,1], self.r_foot[0,2],0,0,0,0])
            l_trajectory.append([self.l_foot[0,0], self.l_foot[0,1], self.l_foot[0,2],0,0,0,0])

        if debug:

            plt.figure(0)
            plt.plot(px)
            plt.plot(com_x)

            plt.figure(1)
            plt.plot(py)
            plt.plot(com_y)

            plt.figure(2)
            plt.plot(com_x,com_y)
            plt.plot(px, py)

            fig = plt.figure(3)
            ax = fig.add_subplot(111, projection='3d')
            ax.set_xlim3d(-0.1, 0.5)
            ax.set_ylim3d(-0.1, 0.5)
            ax.set_zlim3d(0, 0.2)

            com_trajectory = np.array(com_trajectory)
            l_trajectory = np.array(l_trajectory)
            r_trajectory = np.array(r_trajectory)

            plot_trajectory(ax=ax, P=com_trajectory, s=0.02, show_direction=False)
            plot_trajectory(ax=ax, P=r_trajectory,  s=0.02, show_direction=False)
            plot_trajectory(ax=ax, P=l_trajectory,  s=0.02, show_direction=False)

            plt.show()

def main():
    pc = PreviewControl()
    pc.run()

if __name__ == "__main__":
    main()