#! /usr/bin/python3

import numpy as np
from math import sin, cos, tan, pi
class Kalman:
    def __init__(self):
        self.A = np.eye(4)
        self.B = np.zeros((4,2))
        self.C = np.array([[1,0,0,0],
                           [0,0,1,0]])
        self.P = np.eye(4)
        self.Q = np.eye(4) * 0.001
        self.R = np.eye(2) * 0.1
        self.x_pred = np.zeros((4,1))
        self.y_pred = np.zeros((2,1))
        print(self.Q,self.R)

    def state_prediction(self, dt, u):
        self.A = np.array([[1, -dt, 0, 0], [0, 1, 0, 0], [0, 0, 1, -dt], [0, 0, 0, 1]])
        self.B = np.array([[dt, 0], [0, 0], [0, dt], [0, 0]])

        self.x_pred = np.dot(self.A,self.x_pred) + np.dot(self.B,u)
        self.P = np.dot(np.dot(self.A,self.P), np.transpose(self.A)) + self.Q


    def state_update(self, z_pred):
        self.y_pred = z_pred - np.dot(self.C, self.x_pred)
        S = np.dot(np.dot(self.C,self.P), np.transpose(self.C)) + self.R
        K = np.dot(np.dot(self.P, np.transpose(self.C)), np.linalg.inv(S))

        self.x_pred = self.x_pred + np.dot(K, self.y_pred)
        self.P = np.dot((np.eye(4) - np.dot(K,self.C)),self.P)

    def get_state(self):
        return self.x_pred