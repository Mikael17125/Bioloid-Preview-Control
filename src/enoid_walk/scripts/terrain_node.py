#! /usr/bin/python3

import rospy
from geometry_msgs.msg import Vector3, Twist
from std_msgs.msg import Int32, Bool

from inverse_kinematic import *
from servo_controller import *
import math

dZ = 0

def tinggi_Kaki(miringbidang):
    dZ = math.tan((miringbidang*math.pi)/180) * 24/1000

    return dZ

def bezier_curve_2D(phase, p1, p2):
    
    x = (1-phase) * p1[0,0] + phase * p2[0,0]
    y = (1-phase) * p1[0,1] + phase * p2[0,1]
    z = (1-phase) * p1[0,2] + phase * p2[0,2]

    return np.matrix([x,y,z])

def bezier_curve_4D(phase, p1, p2, p3, p4):
    
    x = ((1 - phase)**3 * p1[0,0]) + (3 * (1 - phase)**2 * phase * p2[0,0]) + (3 * (1-phase) * (phase**2) * p3[0,0]) + (phase**3 * p4[0,0])
    y = ((1 - phase)**3 * p1[0,1]) + (3 * (1 - phase)**2 * phase * p2[0,1]) + (3 * (1-phase) * (phase**2) * p3[0,1]) + (phase**3 * p4[0,1])
    z = ((1 - phase)**3 * p1[0,2]) + (3 * (1 - phase)**2 * phase * p2[0,2]) + (3 * (1-phase) * (phase**2) * p3[0,2]) + (phase**3 * p4[0,2])

    return np.matrix([x,y,z])

ori_data = np.array([.0, .0, .0])
gyr_data = np.array([.0, .0, .0])

FOOT_DISTANCE = 6.5 / 1000
COM_HEIGHT = 230 / 1000
X_OFFSET = 5 / 1000
COM_SWING = 115.5 / 1000
COM = np.matrix([X_OFFSET, 0.0, COM_HEIGHT])
LEFT = np.matrix([0.0, 38.5 / 1000 + FOOT_DISTANCE, 0.0])
RIGHT = np.matrix([0.0, -38.5 / 1000 - FOOT_DISTANCE, 0.0])


igl_data = 0.0

cmd_vel = Vector3()
gain_ctrl = Twist()
feedback_mode = Int32()
walk_mode = Int32()
walk_cmd = Int32()
push_data = Bool()
com_msg = Vector3()

finish = False
step = False

def ori_callback(msg):
    ori_data[0] = msg.x
    ori_data[1] = msg.y-5*np.pi/180
    # print(ori_data[1]/np.pi*180)

def gyr_callback(msg):
    gyr_data[0] = msg.x
    gyr_data[1] = msg.y

def vel_callback(msg):
    cmd_vel = msg

def walk_mode_callback(msg):
    global walk_mode
    walk_mode = msg

def cmd_callback(msg):
    global walk_cmd
    walk_cmd = msg

def feedback_callback(msg):
    global feedback_mode
    feedback_mode = msg

def gain_callback(msg):
    global gain_ctrl
    gain_ctrl = msg

def stand(ik):
    global JOINTS, finish, step, delta_step, COM, LEFT, RIGHT,  com_msg
    COM = np.matrix([X_OFFSET, 0.0, COM_HEIGHT])
    LEFT = np.matrix([0.0, 38.5 / 1000 + FOOT_DISTANCE, 0.0])
    RIGHT = np.matrix([0.0, -38.5 / 1000 - FOOT_DISTANCE, 0.0])
    JOINTS = ik.solve(COM, LEFT, RIGHT)
    finish = False
    step = False
    delta_step = 0.0
    com_msg.x = COM[0,0]
    com_msg.y = COM[0,1]
    com_msg.z = COM[0,2]

igl_data_roll = 0.0

def main():
    global igl_data_roll
    rospy.init_node('enoid_terrain', anonymous=False)
    rospy.Subscriber("ori_data", Vector3, ori_callback)
    rospy.Subscriber("gyr_data", Vector3, gyr_callback)
    rospy.Subscriber("feedback_status", Int32, feedback_callback)
    rospy.Subscriber("gain_control", Twist, gain_callback)
    com_pub = rospy.Publisher('com_data', Vector3, queue_size=1)
    rate = rospy.Rate(30)

    com_msg = Vector3()

    ik = InverseKinematic()
    sc = ServoController()
    
   # AXIS  = np.array([-1,-1,-1,-1,1,-1,1,1,1,1])
    COM   = np.matrix([0.0, 0.0, 0.23])
    LEFT  = np.matrix([0.0, 48 / 1000, 0.0])
    RIGHT = np.matrix([0.0, -48 / 1000, 0.0])
    phase = 0

    uCOM   = np.matrix([0.0, 0.0, 0.23])
    uLEFT  = np.matrix([0.0, 48 / 1000, 0.0])
    uRIGHT = np.matrix([0.0, -48 / 1000, 0.0])
    
    time_start = rospy.Time.now().to_sec()
    phase = 0.0

    state_time = np.array([10,1,2,1,2,1,1,2,1,2,1,1,2,1,2,1,1,2,1,2,1,1,2,1,2,1])#10,1,2,5,2,5

    state = 0
    
    teta_bidang = 20
    status_kaki = 1 # 0 = kaki kanan diatas, 1 = kaki kanan dibawah
    if status_kaki == 0:
        dZ = tinggi_Kaki(teta_bidang)
    else:
        dZ = - tinggi_Kaki(teta_bidang)
    
    while not rospy.is_shutdown():
        
        print("ini besar kemiringan bidang : ")    
        print(dZ)
 
        if phase >= 1:
            time_start = rospy.Time.now().to_sec()
            phase = 0
            if state == 25 :
                finish = True
                break
            state = state + 1

            uCOM = COM
            uRIGHT = RIGHT
            uLEFT = LEFT
        else:
            time_now = rospy.Time.now().to_sec() - time_start
            phase = time_now / state_time[state]

#..........................................Bidang Miring.................................................
        #3 derajat

        if state == 0:
            print("WAIT")
        if state == 1:
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.025, 0.045, 0.23]))#[0.025, 0.052, 0.23]
            #COM = bezier_curve_2D(phase, uCOM, np.matrix([0.025, -0.040, 0.23]))
            RIGHT =  bezier_curve_2D(phase, uRIGHT,  np.matrix([0.0, -48 / 1000, 0.0 ]))
            LEFT =   bezier_curve_2D(phase, uLEFT,  np.matrix([0.0, 48 / 1000, 0.0 ]))
        elif state == 2:
            #print("phase 1")
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.015, 0.075, 0.20]))
            RIGHT =   bezier_curve_4D(phase, uRIGHT, np.matrix([0.0, -48 / 1000, 0.01 ]), np.matrix([0.03, -48 / 1000, 0.015]), np.matrix([0.02, -48 / 1000, 0.005]))
            LEFT =   bezier_curve_4D(phase, uLEFT, np.matrix([0.0, 48 / 1000, 0.02]), np.matrix([0.00, 48 / 1000, 0.03]), np.matrix([0.0, 48 / 1000, 0.005]))
        elif state == 3:
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.035,0.055, 0.23]))#[0.045, -0.045, 0.23]
            ik.TILT = 10
        """elif state == 4:
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.055, -0.030, 0.22]))
            LEFT =   bezier_curve_4D(phase, uLEFT, np.matrix([0.0, 48 / 1000, 0.02]), np.matrix([0.06, 48 / 1000, 0.03]), np.matrix([0.04, 48 / 1000, 0.005]))
        elif state == 5:
            ik.TILT = 10
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.055, 0.0 , 0.23]))"""
#..............................................Langkah 2..............................................................
        """elif state == 6:
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.055, 0.052, 0.23]))
            #RIGHT =  bezier_curve_2D(phase, uRIGHT,  np.matrix([0.0, -48 / 1000, 0.0]))
            #LEFT =   bezier_curve_2D(phase, uLEFT,  np.matrix([0.0, 48 / 1000, 0.0]))
        elif state == 7:
            #print("phase 1")
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.055, 0.052, 0.22]))
            RIGHT =   bezier_curve_4D(phase, uRIGHT, np.matrix([0.02, -48 / 1000, 0.02]), np.matrix([0.08, -48 / 1000, 0.03]), np.matrix([0.06, -48 / 1000, 0.005]))
        elif state == 8:
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.095, -0.052, 0.23]))#[0.14, -0.05, 0.23]
            ik.TILT = 10
        elif state == 9:
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.105, -0.052, 0.22]))
            LEFT =   bezier_curve_4D(phase, uLEFT, np.matrix([0.04, 48 / 1000, 0.02]), np.matrix([0.1, 48 / 1000, 0.03]), np.matrix([0.08, 48 / 1000, 0.005]))
        elif state == 10:
            ik.TILT = 10.0
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.105, 0.0 , 0.23]))
#................................................Langkah 3...........................................................        
        elif state == 11:
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.115, 0.052, 0.23]))
            #COM = bezier_curve_2D(phase, uCOM, np.matrix([0.025, -0.040, 0.23]))
            #RIGHT =  bezier_curve_2D(phase, uRIGHT,  np.matrix([0.0, -48 / 1000, 0.0]))
            #LEFT =   bezier_curve_2D(phase, uLEFT,  np.matrix([0.0, 48 / 1000, 0.0]))
        elif state == 12:
            #print("phase 1")
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.115, 0.052, 0.22]))
            RIGHT =   bezier_curve_4D(phase, uRIGHT, np.matrix([0.06, -48 / 1000, 0.02]), np.matrix([0.12, -48 / 1000, 0.03]), np.matrix([0.1, -48 / 1000, 0.005]))
        elif state == 13:
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.125, -0.052, 0.23]))
            ik.TILT = 10
        elif state == 14:
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.125, -0.052, 0.22]))
            LEFT =   bezier_curve_4D(phase, uLEFT, np.matrix([0.08, 48 / 1000, 0.02]), np.matrix([0.14, 48 / 1000, 0.03]), np.matrix([0.12, 48 / 1000, 0.005]))
        elif state == 15:
            ik.TILT = 10.0
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.125, 0.0 , 0.23]))
#................................................tangga 4...........................................................   
        elif state == 16:
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.135, 0.052, 0.23]))
            #COM = bezier_curve_2D(phase, uCOM, np.matrix([0.025, -0.040, 0.23]))
            #RIGHT =  bezier_curve_2D(phase, uRIGHT,  np.matrix([0.0, -48 / 1000, 0.0]))
            #LEFT =   bezier_curve_2D(phase, uLEFT,  np.matrix([0.0, 48 / 1000, 0.0]))
        elif state == 17:
            #print("phase 1")
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.155, 0.052, 0.22]))
            RIGHT =   bezier_curve_4D(phase, uRIGHT, np.matrix([0.1, -48 / 1000, 0.02]), np.matrix([0.16, -48 / 1000, 0.03]), np.matrix([0.14, -48 / 1000, 0.005]))
        elif state == 18:
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.165, -0.052, 0.23]))
            ik.TILT = 10
        elif state == 19:
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.165, -0.052, 0.22]))
            LEFT =   bezier_curve_4D(phase, uLEFT, np.matrix([0.12, 48 / 1000, 0.02]), np.matrix([0.18, 48 / 1000, 0.03]), np.matrix([0.16, 48 / 1000, 0.005]))
        elif state == 20:
            ik.TILT = 10.0
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.165, 0.0 , 0.23]))
#................................................Langkah 5...........................................................   
        elif state == 21:
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.185, 0.052, 0.23]))
            #COM = bezier_curve_2D(phase, uCOM, np.matrix([0.025, -0.040, 0.23]))
            #RIGHT =  bezier_curve_2D(phase, uRIGHT,  np.matrix([0.0, -48 / 1000, 0.0]))
            #LEFT =   bezier_curve_2D(phase, uLEFT,  np.matrix([0.0, 48 / 1000, 0.0]))
        elif state == 22:
            #print("phase 1")
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.205, 0.052, 0.22]))
            RIGHT =   bezier_curve_4D(phase, uRIGHT, np.matrix([0.14, -48 / 1000, 0.02]), np.matrix([0.2, -48 / 1000, 0.03]), np.matrix([0.18, -48 / 1000, 0.005]))
        elif state == 23:
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.215, -0.052, 0.23]))
            ik.TILT = 10
        elif state == 24:
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.215, -0.052, 0.22]))
            LEFT =   bezier_curve_4D(phase, uLEFT, np.matrix([0.16, 48 / 1000, 0.02]), np.matrix([0.22, 48 / 1000, 0.03]), np.matrix([0.2, 48 / 1000, 0.005]))
        elif state == 25:7i
            ik.TILT = 10.0
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.215, 0.0 , 0.23]))"""


#...........................................Bidang Datar............................................................
        """if state == 0:
            print("WAIT")
        if state == 1:
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.025, 0.045, 0.23]))#[0.025, 0.052, 0.23]
            #COM = bezier_curve_2D(phase, uCOM, np.matrix([0.025, -0.040, 0.23]))
            RIGHT =  bezier_curve_2D(phase, uRIGHT,  np.matrix([0.0, -48 / 1000, 0.0 - dZ]))
            LEFT =   bezier_curve_2D(phase, uLEFT,  np.matrix([0.0, 48 / 1000, 0.0 + dZ]))
        elif state == 2:
            #print("phase 1")
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.015, 0.052, 0.22]))
            RIGHT =   bezier_curve_4D(phase, uRIGHT, np.matrix([0.0, -48 / 1000, 0.02 - dZ]), np.matrix([0.03, -48 / 1000, 0.03 - dZ]), np.matrix([0.02, -48 / 1000, 0.005 - dZ]))
        elif state == 3:
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.055,-0.035, 0.23]))#[0.045, -0.045, 0.23]
            ik.TILT = 10
        elif state == 4:
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.055, -0.052, 0.22]))
            LEFT =   bezier_curve_4D(phase, uLEFT, np.matrix([0.0, 48 / 1000, 0.02 + dZ]), np.matrix([0.06, 48 / 1000, 0.03 + dZ]), np.matrix([0.04, 48 / 1000, 0.005 + dZ]))
        elif state == 5:
            ik.TILT = 10
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.055, 0.0 , 0.23]))
#..............................................Langkah 2..............................................................
        elif state == 6:
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.055, 0.052, 0.23]))
            #RIGHT =  bezier_curve_2D(phase, uRIGHT,  np.matrix([0.0, -48 / 1000, 0.0]))
            #LEFT =   bezier_curve_2D(phase, uLEFT,  np.matrix([0.0, 48 / 1000, 0.0]))
        elif state == 7:
            #print("phase 1")
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.055, 0.052, 0.22]))
            RIGHT =   bezier_curve_4D(phase, uRIGHT, np.matrix([0.02, -48 / 1000, 0.02]), np.matrix([0.08, -48 / 1000, 0.03]), np.matrix([0.06, -48 / 1000, 0.005]))
        elif state == 8:
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.095, -0.052, 0.23]))#[0.14, -0.05, 0.23]
            ik.TILT = 10
        elif state == 9:
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.105, -0.052, 0.22]))
            LEFT =   bezier_curve_4D(phase, uLEFT, np.matrix([0.04, 48 / 1000, 0.02]), np.matrix([0.1, 48 / 1000, 0.03]), np.matrix([0.08, 48 / 1000, 0.005]))
        elif state == 10:
            ik.TILT = 10.0
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.105, 0.0 , 0.23]))
#................................................Langkah 3...........................................................        
        elif state == 11:
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.115, 0.052, 0.23]))
            #COM = bezier_curve_2D(phase, uCOM, np.matrix([0.025, -0.040, 0.23]))
            #RIGHT =  bezier_curve_2D(phase, uRIGHT,  np.matrix([0.0, -48 / 1000, 0.0]))
            #LEFT =   bezier_curve_2D(phase, uLEFT,  np.matrix([0.0, 48 / 1000, 0.0]))
        elif state == 12:
            #print("phase 1")
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.115, 0.052, 0.22]))
            RIGHT =   bezier_curve_4D(phase, uRIGHT, np.matrix([0.06, -48 / 1000, 0.02]), np.matrix([0.12, -48 / 1000, 0.03]), np.matrix([0.1, -48 / 1000, 0.005]))
        elif state == 13:
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.125, -0.052, 0.23]))
            ik.TILT = 10
        elif state == 14:
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.125, -0.052, 0.22]))
            LEFT =   bezier_curve_4D(phase, uLEFT, np.matrix([0.08, 48 / 1000, 0.02]), np.matrix([0.14, 48 / 1000, 0.03]), np.matrix([0.12, 48 / 1000, 0.005]))
        elif state == 15:
            ik.TILT = 10.0
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.125, 0.0 , 0.23]))
#................................................tangga 4...........................................................   
        elif state == 16:
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.135, 0.052, 0.23]))
            #COM = bezier_curve_2D(phase, uCOM, np.matrix([0.025, -0.040, 0.23]))
            #RIGHT =  bezier_curve_2D(phase, uRIGHT,  np.matrix([0.0, -48 / 1000, 0.0]))
            #LEFT =   bezier_curve_2D(phase, uLEFT,  np.matrix([0.0, 48 / 1000, 0.0]))
        elif state == 17:
            #print("phase 1")
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.155, 0.052, 0.22]))
            RIGHT =   bezier_curve_4D(phase, uRIGHT, np.matrix([0.1, -48 / 1000, 0.02]), np.matrix([0.16, -48 / 1000, 0.03]), np.matrix([0.14, -48 / 1000, 0.005]))
        elif state == 18:
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.165, -0.052, 0.23]))
            ik.TILT = 10
        elif state == 19:
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.165, -0.052, 0.22]))
            LEFT =   bezier_curve_4D(phase, uLEFT, np.matrix([0.12, 48 / 1000, 0.02]), np.matrix([0.18, 48 / 1000, 0.03]), np.matrix([0.16, 48 / 1000, 0.005]))
        elif state == 20:
            ik.TILT = 10.0
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.165, 0.0 , 0.23]))
#................................................Langkah 5...........................................................   
        elif state == 21:
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.185, 0.052, 0.23]))
            #COM = bezier_curve_2D(phase, uCOM, np.matrix([0.025, -0.040, 0.23]))
            #RIGHT =  bezier_curve_2D(phase, uRIGHT,  np.matrix([0.0, -48 / 1000, 0.0]))
            #LEFT =   bezier_curve_2D(phase, uLEFT,  np.matrix([0.0, 48 / 1000, 0.0]))
        elif state == 22:
            #print("phase 1")
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.205, 0.052, 0.22]))
            RIGHT =   bezier_curve_4D(phase, uRIGHT, np.matrix([0.14, -48 / 1000, 0.02]), np.matrix([0.2, -48 / 1000, 0.03]), np.matrix([0.18, -48 / 1000, 0.005]))
        elif state == 23:
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.215, -0.052, 0.23]))
            ik.TILT = 10
        elif state == 24:
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.215, -0.052, 0.22]))
            LEFT =   bezier_curve_4D(phase, uLEFT, np.matrix([0.16, 48 / 1000, 0.02]), np.matrix([0.22, 48 / 1000, 0.03]), np.matrix([0.2, 48 / 1000, 0.005]))
        elif state == 25:
            ik.TILT = 10.0
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.215, 0.0 , 0.23]))"""
            
        
        JOINTS = ik.solve(COM,LEFT, RIGHT)

        #K = np.array([-0.04, -0.05])#gain proposional, gain derivatife (pitch)0.06, 0.01
        #Kr = np.array([-0.085, -0.06])#gain proposional, gain derivatife (roll)0.06, 0.06    -0.065, -0.05
        #igl_data_roll += ori_data[0]
        #Kr = np.array([0.04, 0.05])
        #K = np.array([0.04, 0.1])
        if ((feedback_mode.data == 1) or (feedback_mode.data == 2)) and not(finish):
            igl_data[1] +=  ori_data[1]/30
            #delta_pitch = K[0] * ori_data[1] + K[1] * -gyr_data[1]
            #delta_roll = Kr[0] * ori_data[0] + Kr[1] * -gyr_data[0]
            delta_roll = gain_ctrl.linear.x * ori_data[0] + gain_ctrl.angular.x * gyr_data[0]
            delta_pitch = -gain_ctrl.linear.y * ori_data[1] + -gain_ctrl.angular.y * gyr_data[1] + gain_ctrl.angular.x * igl_data[1]
            print((delta_roll * 180/np.pi))
            JOINTS[3] += delta_pitch
            JOINTS[8] -= delta_pitch 

            JOINTS[4] += delta_roll
            JOINTS[9] -= delta_roll

            #JOINTS[4] += (delta_roll + ((0*math.pi)/180))
            #JOINTS[9] -= (delta_roll - ((0*math.pi)/180))

        

        sc.sync_write_pos(JOINTS )#* AXIS

        com_msg.x = COM[0,0]
        com_msg.y = COM[0,1]
        com_msg.z = COM[0,2]

        com_pub.publish(com_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass