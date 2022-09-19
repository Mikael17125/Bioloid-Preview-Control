import time

from random import randrange
import pybullet as p
import pybullet_data

import numpy as np

COM_HIP_Z = 0.031
COM_HIP_Y = 0.0385

HIP_KNEE = 0.075
KNEE_ANKLE = 0.075
ANKLE_FOOT = 0.0325

KNEE_VERT = 0.015

def Rx(theta):
    return np.matrix([[1, 0, 0],
                      [0, np.cos(theta), -np.sin(theta)],
                      [0, np.sin(theta), np.cos(theta)]])


def Ry(theta):
    return np.matrix([[np.cos(theta), 0, np.sin(theta)],
                      [0, 1, 0],
                      [-np.sin(theta), 0, np.cos(theta)]])


def Rz(theta):
    return np.matrix([[np.cos(theta), -np.sin(theta), 0],
                      [np.sin(theta), np.cos(theta), 0],
                      [0, 0, 1]])

def inverse_kinematic(COM, LEG, isLeft):
    if isLeft:
        COM_HIP_Y = 38.5 / 1000
    else:
        COM_HIP_Y = -38.5 / 1000
    p_FOOT = np.matrix([-LEG[0,0], LEG[0,1], LEG[0,2]]).T
    R_FOOT = Rz(.0) * Ry(.0) * Rx(.0)
    p_COM =  np.matrix([-COM[0,0], COM[0,1], COM[0,2]]).T
    R_COM = Rz(.0) * Ry(.0) * Rx(.0)

    p_COM_FOOT = p_COM - p_FOOT
    p_ANKLE_FOOT = np.matrix([.0, .0, -ANKLE_FOOT]).T

    p_HIP_PELVIS = np.matrix([.0, COM_HIP_Y, -COM_HIP_Z]).T
    p_COM_ANKLE = p_ANKLE_FOOT + p_COM_FOOT
    p_ANKLE_HIP = (p_COM_ANKLE + (R_COM * p_HIP_PELVIS))


    q17 = (-np.arctan2(p_ANKLE_HIP[1,0], p_ANKLE_HIP[2,0])).item(0)

    p_LEG = np.sqrt(p_ANKLE_HIP[0,0] ** 2 + p_ANKLE_HIP[1,0] ** 2 + p_ANKLE_HIP[2,0] ** 2)
    Lf = np.sqrt(KNEE_VERT ** 2 + HIP_KNEE ** 2)
    alpha = np.arctan2(KNEE_VERT, HIP_KNEE)
    Cy = -((p_LEG ** 2) - (2 * Lf ** 2)) / (2 * Lf ** 2)
    gamma = np.arctan2(np.sqrt(1 - Cy ** 2), Cy)
    q13 = (np.pi - (gamma + 2 * alpha)).item(0)
    r_Leg_X = np.sqrt(p_ANKLE_HIP[2,0] ** 2 + p_ANKLE_HIP[0,0] ** 2)
    beta = -np.arctan2(p_ANKLE_HIP[0,0], r_Leg_X)
    q15 = np.pi / 2 + ((beta - gamma / 2 - alpha)).item(0)

    R_x = Rx(-q17)
    R_y = Ry(q15 - q13)

    R = R_COM.transpose().dot(R_FOOT.dot(R_x.dot(R_y)))

    q7 = np.arctan2(-R[0, 1], R[1, 1])
    cz = np.cos(q7)
    sz = np.sin(q7)
    q9 = np.arctan2(R[2, 1], -R[0, 1] * sz + R[1, 1] * cz)
    q11 = np.arctan2(-R[2, 0], R[2, 2])

    return np.array([[q7], [q9], [q11], [q13], [q15], [q17]], dtype=float)

l_axis = [1,1,-1,-1,-1,-1]
r_axis = [1,1,1,1,1,-1]

p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

p.setGravity(0, 0, -9.8)
planeId = p.loadURDF("plane.urdf")

#<=======BIOLOID=========>
bioloid_init_pos = [0, 0, 0.3]
bioloid_init_ori = p.getQuaternionFromEuler([0, 0, 0])
bioloid_model = p.loadURDF("assets/bioloid.urdf", bioloid_init_pos, bioloid_init_ori)
index = {}

#INIT POSITION
for id in range(p.getNumJoints(bioloid_model)):
    if (p.getJointInfo(bioloid_model, id)[2] == 0):
        index[p.getJointInfo(bioloid_model, id)[12].decode('UTF-8')] = id

for id in index:
    p.setJointMotorControl2(bioloid_model, index[id], p.POSITION_CONTROL, 0.0)

p.setJointMotorControl2(bioloid_model, 2, p.POSITION_CONTROL, -np.pi)
p.setJointMotorControl2(bioloid_model, 3, p.POSITION_CONTROL, np.pi)

p.setJointMotorControl2(bioloid_model, 6, p.POSITION_CONTROL, np.pi)
p.setJointMotorControl2(bioloid_model, 7, p.POSITION_CONTROL, np.pi)

time.sleep(5)

start = time.time()
while True:
    time.sleep(1/240)
    p.stepSimulation()

    COM = np.matrix([0.0,0.02,0.19])
    LEFT = np.matrix([0.0,COM_HIP_Y,0.0])
    RIGHT = np.matrix([0,-COM_HIP_Y,0.08])

    left_joint = inverse_kinematic(COM,LEFT, True)
    right_joint = inverse_kinematic(COM, RIGHT, False)

    for i in range(6):
        p.setJointMotorControl2(bioloid_model, 17 + i, p.POSITION_CONTROL, left_joint[i] * l_axis[i])
        p.setJointMotorControl2(bioloid_model, 10 + i, p.POSITION_CONTROL, right_joint[i] * r_axis[i])

p.disconnect()

