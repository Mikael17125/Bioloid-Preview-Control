import numpy as np

COM_HIP_Z = 31 / 10
COM_HIP_Y = 38.5 / 10

HIP_KNEE = 75 / 10
KNEE_ANKLE = 75 / 10
ANKLE_FOOT = 32.5 / 10

KNEE_VERT = 15 / 10

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

def inverse_kinematic(COM, LEG):

    p_HIP_PELVIS = np.array([[.0],[COM_HIP_Y],[COM_HIP_Z]])

    R_ANKLE_PELVIS = Rz(.0) * Ry(.0) * Rx(.0)
    p_ANKLE_PELVIS = COM - LEG
    p_ANKLE_HIP = p_ANKLE_PELVIS - (R_ANKLE_PELVIS * p_HIP_PELVIS)

    q17 = (-np.arctan2(p_ANKLE_HIP[1][0], p_ANKLE_HIP[2][0])).item(0)

    p_LEG = np.sqrt(np.power(p_ANKLE_HIP[0][0],2) + np.power(p_ANKLE_HIP[1][0],2) + np.power(p_ANKLE_HIP[2][0],2))
    Lf = np.sqrt(KNEE_VERT**2 + HIP_KNEE**2)
    alpha = np.arctan2(KNEE_VERT, HIP_KNEE)
    Cy = -((p_LEG**2) - (2*Lf**2))/(2*Lf**2)
    gamma = np.arctan2(np.sqrt(1-Cy**2),Cy)

    q13 = (np.pi-(gamma + 2*alpha)).item(0)

    r_Leg_X = np.sqrt(p_ANKLE_HIP[2][0]**2 + p_ANKLE_HIP[1][0]**2)
    beta = np.arctan2(p_ANKLE_HIP[2][0],r_Leg_X)

    q15 = (np.pi/4 + (beta - gamma/2 - alpha)).item(0)
    R_BODY = Rz(.0) * Ry(.0) * Rx(.0)
    R_FOOT = Rz(.0) * Ry(.0) * Rx(.0)

    R_x = Rx(q17)
    R_y = Ry(-q15-q13)

    R = R_BODY.transpose().dot(R_FOOT.dot(R_x.dot(R_y)))

    q7=np.arctan2(-R[0,1],R[1,1])
    cz = np.cos(q7)
    sz = np.sin(q7)
    q9 = np.arctan2(R[2,1], -R[0,1] * sz + R[1,1]*cz)
    q11 = np.arctan2(-R[2,0], R[2,2])

    print(q17*180/np.pi)
    print(q15*180/np.pi)
    print(q13*180/np.pi)
    print(q11*180/np.pi)
    print(q9*180/np.pi)
    print(q7*180/np.pi)


# 0 dari tanah

COM_ = np.array([[.0],[.0],[181/10]])
RIGHT_ = np.array([[.0],[-COM_HIP_Y],[.0]])
LEFT_ = np.array([[.0],[COM_HIP_Y],[.0],[.0],[.0],[.0]])

inverse_kinematic(COM_, RIGHT_)