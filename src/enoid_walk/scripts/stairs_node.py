#! /usr/bin/python3

import rospy
from geometry_msgs.msg import Vector3

from inverse_kinematic import *
from servo_controller import *

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

igl_data = 0.0

def ori_callback(msg):
    ori_data[0] = msg.x
    ori_data[1] = msg.y-5*np.pi/180
    # print(ori_data[1]/np.pi*180)

def gyr_callback(msg):
    gyr_data[0] = msg.x
    gyr_data[1] = msg.y

igl_data_roll = 0.0

def main():
    global igl_data_roll
    rospy.init_node('enoid_stairs', anonymous=False)
    rospy.Subscriber("ori_data", Vector3, ori_callback)
    rospy.Subscriber("gyr_data", Vector3, gyr_callback)
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

    state_time = np.array([10,1,2,3.5,2,2,1,2,3.5,2,2,1,2,3.5,2,2,1,2,3.5,2,2,1,2,3.5,2,2])#10,1,10,2,10,3.5,10,2,10,2

    state = 0

    while not rospy.is_shutdown():
        
 
        if phase >= 1:
            time_start = rospy.Time.now().to_sec()
            phase = 0
            state = state + 1

            uCOM = COM
            uRIGHT = RIGHT
            uLEFT = LEFT
        else:
            time_now = rospy.Time.now().to_sec() - time_start
            phase = time_now / state_time[state]

        if state == 0:
            print("WAIT")
        if state == 1:
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.025, 0.052, 0.23]))
            #COM = bezier_curve_2D(phase, uCOM, np.matrix([0.025, -0.040, 0.23]))
            RIGHT =  bezier_curve_2D(phase, uRIGHT,  np.matrix([0.0, -48 / 1000, 0.0]))
            LEFT =   bezier_curve_2D(phase, uLEFT,  np.matrix([0.0, 48 / 1000, 0.0]))
        elif state == 2:
            #print("phase 1")
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.015, 0.052, 0.22]))
            RIGHT =   bezier_curve_4D(phase, uRIGHT, np.matrix([0.0, -48 / 1000, 0.05]), np.matrix([0.10, -48 / 1000, 0.07]), np.matrix([0.10, -48 / 1000, 0.022]))
        elif state == 3:
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.14, -0.05, 0.23]))
            ik.TILT = 18.5
        elif state == 4:
            #COM = bezier_curve_2D(phase, uCOM, np.matrix([0.11, -0.030, 0.22]))
            LEFT =   bezier_curve_4D(phase, uLEFT, np.matrix([0.0, 48 / 1000, 0.07]), np.matrix([0.10, 48 / 1000, 0.08]), np.matrix([0.10, 48 / 1000, 0.022]))
        elif state == 5:
            ik.TILT = 10.0
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.11, 0.0 , 0.25]))
#..............................................tangga 2..............................................................
        elif state == 6:
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.025, 0.052, 0.23]))
            #COM = bezier_curve_2D(phase, uCOM, np.matrix([0.025, -0.040, 0.23]))
            RIGHT =  bezier_curve_2D(phase, uRIGHT,  np.matrix([0.0, -48 / 1000, 0.0]))
            LEFT =   bezier_curve_2D(phase, uLEFT,  np.matrix([0.0, 48 / 1000, 0.0]))
        elif state == 7:
            #print("phase 1")
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.015, 0.052, 0.22]))
            RIGHT =   bezier_curve_4D(phase, uRIGHT, np.matrix([0.0, -48 / 1000, 0.05]), np.matrix([0.10, -48 / 1000, 0.07]), np.matrix([0.10, -48 / 1000, 0.022]))
        elif state == 8:
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.14, -0.05, 0.23]))
            ik.TILT = 18.5
        elif state == 9:
            #COM = bezier_curve_2D(phase, uCOM, np.matrix([0.11, -0.030, 0.22]))
            LEFT =   bezier_curve_4D(phase, uLEFT, np.matrix([0.0, 48 / 1000, 0.07]), np.matrix([0.10, 48 / 1000, 0.08]), np.matrix([0.10, 48 / 1000, 0.022]))
        elif state == 10:
            ik.TILT = 10.0
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.11, 0.0 , 0.25]))
#................................................tangga 3...........................................................        
        elif state == 11:
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.025, 0.052, 0.23]))
            #COM = bezier_curve_2D(phase, uCOM, np.matrix([0.025, -0.040, 0.23]))
            RIGHT =  bezier_curve_2D(phase, uRIGHT,  np.matrix([0.0, -48 / 1000, 0.0]))
            LEFT =   bezier_curve_2D(phase, uLEFT,  np.matrix([0.0, 48 / 1000, 0.0]))
        elif state == 12:
            #print("phase 1")
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.015, 0.052, 0.22]))
            RIGHT =   bezier_curve_4D(phase, uRIGHT, np.matrix([0.0, -48 / 1000, 0.05]), np.matrix([0.10, -48 / 1000, 0.07]), np.matrix([0.10, -48 / 1000, 0.022]))
        elif state == 13:
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.14, -0.05, 0.23]))
            ik.TILT = 18.5
        elif state == 14:
            #COM = bezier_curve_2D(phase, uCOM, np.matrix([0.11, -0.030, 0.22]))
            LEFT =   bezier_curve_4D(phase, uLEFT, np.matrix([0.0, 48 / 1000, 0.07]), np.matrix([0.10, 48 / 1000, 0.08]), np.matrix([0.10, 48 / 1000, 0.022]))
        elif state == 15:
            ik.TILT = 10.0
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.11, 0.0 , 0.25]))
#................................................tangga 4...........................................................   
        elif state == 16:
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.025, 0.052, 0.23]))
            #COM = bezier_curve_2D(phase, uCOM, np.matrix([0.025, -0.040, 0.23]))
            RIGHT =  bezier_curve_2D(phase, uRIGHT,  np.matrix([0.0, -48 / 1000, 0.0]))
            LEFT =   bezier_curve_2D(phase, uLEFT,  np.matrix([0.0, 48 / 1000, 0.0]))
        elif state == 17:
            #print("phase 1")
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.015, 0.052, 0.22]))
            RIGHT =   bezier_curve_4D(phase, uRIGHT, np.matrix([0.0, -48 / 1000, 0.05]), np.matrix([0.10, -48 / 1000, 0.07]), np.matrix([0.10, -48 / 1000, 0.022]))
        elif state == 18:
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.14, -0.05, 0.23]))
            ik.TILT = 18.5
        elif state == 19:
            #COM = bezier_curve_2D(phase, uCOM, np.matrix([0.11, -0.030, 0.22]))
            LEFT =   bezier_curve_4D(phase, uLEFT, np.matrix([0.0, 48 / 1000, 0.07]), np.matrix([0.10, 48 / 1000, 0.08]), np.matrix([0.10, 48 / 1000, 0.022]))
        elif state == 20:
            ik.TILT = 10.0
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.11, 0.0 , 0.25]))
#................................................tangga 5...........................................................   
        elif state == 21:
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.025, 0.052, 0.23]))
            #COM = bezier_curve_2D(phase, uCOM, np.matrix([0.025, -0.040, 0.23]))
            RIGHT =  bezier_curve_2D(phase, uRIGHT,  np.matrix([0.0, -48 / 1000, 0.0]))
            LEFT =   bezier_curve_2D(phase, uLEFT,  np.matrix([0.0, 48 / 1000, 0.0]))
        elif state == 22:
            #print("phase 1")
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.015, 0.052, 0.22]))
            RIGHT =   bezier_curve_4D(phase, uRIGHT, np.matrix([0.0, -48 / 1000, 0.05]), np.matrix([0.10, -48 / 1000, 0.07]), np.matrix([0.10, -48 / 1000, 0.022]))
        elif state == 23:
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.14, -0.05, 0.23]))
            ik.TILT = 18.5
        elif state == 24:
            #COM = bezier_curve_2D(phase, uCOM, np.matrix([0.11, -0.030, 0.22]))
            LEFT =   bezier_curve_4D(phase, uLEFT, np.matrix([0.0, 48 / 1000, 0.07]), np.matrix([0.10, 48 / 1000, 0.08]), np.matrix([0.10, 48 / 1000, 0.022]))
        elif state == 25:
            ik.TILT = 10.0
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.11, 0.0 , 0.25]))
            
        
        JOINTS = ik.solve(COM,LEFT, RIGHT)

        K = np.array([-0.04, -0.05])#gain proposional, gain derivatife (pitch)0.06, 0.01
        Kr = np.array([-0.085, -0.06])#gain proposional, gain derivatife (roll)0.06, 0.06    -0.065, -0.05
        igl_data_roll += ori_data[0]
        #Kr = np.array([0.04, 0.05])
        #K = np.array([0.04, 0.1])

        delta_pitch = K[0] * ori_data[1] + K[1] * -gyr_data[1]
        delta_roll = Kr[0] * ori_data[0] + Kr[1] * -gyr_data[0]
       
        print((delta_roll * 180/np.pi))
        JOINTS[3] += delta_pitch
        JOINTS[8] -= delta_pitch 

        JOINTS[4] += delta_roll
        JOINTS[9] -= delta_roll

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