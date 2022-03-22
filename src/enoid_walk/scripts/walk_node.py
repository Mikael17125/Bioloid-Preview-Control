#! /usr/bin/python3

import rospy
from geometry_msgs.msg import Vector3, Twist
from std_msgs.msg import Int32, Bool

from preview_controller import *
from inverse_kinematic import *
from servo_controller import *
from fuzzy_logic import *

ori_data = np.array([.0, .0, .0])
gyr_data = np.array([.0, .0, .0])
igl_data = np.array([.0, .0, .0])

cmd_vel = Vector3()
gain_ctrl = Twist()
feedback_mode = Int32()
walk_mode = Int32()
walk_cmd = Int32()
push_data = Bool()
com_msg = Vector3()

FOOT_DISTANCE = 6.5 / 1000
COM_HEIGHT = 230 / 1000
X_OFFSET = 5 / 1000
COM_SWING = 115.5 / 1000

COM = np.matrix([X_OFFSET, 0.0, COM_HEIGHT])
LEFT = np.matrix([0.0, 38.5 / 1000 + FOOT_DISTANCE, 0.0])
RIGHT = np.matrix([0.0, -38.5 / 1000 - FOOT_DISTANCE, 0.0])

uCOM = np.matrix([X_OFFSET, 0.0, COM_HEIGHT])
uLEFT = np.matrix([0.0, 38.5 / 1000 + FOOT_DISTANCE, 0.0])
uRIGHT = np.matrix([0.0, -38.5 / 1000 - FOOT_DISTANCE, 0.0])

debug = False
flag = False
step = False
finish = False
JOINTS = np.zeros(12)

start = 0.0
now = 0.0
delta_step = 0.0


def ori_callback(msg):
    ori_data[0] = msg.x
    ori_data[1] = msg.y 


def gyr_callback(msg):
    gyr_data[0] = msg.x 
    gyr_data[1] = msg.y


def vel_callback(msg):
    cmd_vel = msg


def cmd_callback(msg):
    global walk_cmd
    walk_cmd = msg


def walk_mode_callback(msg):
    global walk_mode
    walk_mode = msg


def feedback_callback(msg):
    global feedback_mode
    feedback_mode = msg

def gain_callback(msg):
    global gain_ctrl
    gain_ctrl = msg

def push_callback(msg):
    global push_data
    push_data = msg

def bezier_curve(phase, p_start, p_cnt, p_end):
    if(phase >= 1):
        phase = 1

    t = np.matrix([1, phase,  phase**2])
    coef = np.matrix([[1, 0, 0],
                      [-2, 2, 0],
                      [1, -2, 1]])
    point = np.vstack((p_start, p_cnt, p_end))

    path = t * coef * point

    return path

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


def walk_test(pc, ik):
    global JOINTS, COM, LEFT, RIGHT, com_msg

    pc.update_walking_pattern()

    if pc.walking_ready and not debug:
        JOINTS = ik.solve(pc.com_pose, pc.l_foot, pc.r_foot)
        com_msg.x = pc.com_pose[0,0]
        com_msg.y = pc.com_pose[0,1]
        com_msg.z = pc.com_pose[0,2]
    else:
        JOINTS = ik.solve(COM, LEFT, RIGHT)
    
        com_msg.x = COM[0,0]
        com_msg.y = COM[0,1]
        com_msg.z = COM[0,2]


def push_test(fz, ik):
    global flag, finish, delta_step,step, start, now, JOINTS, COM, LEFT, RIGHT,uCOM, uLEFT, uRIGHT, com_msg
    time_step = .1

    if finish == False:

        if not(push_data.data) and not(step):
            # print("IN 1")
            JOINTS = ik.solve(COM, LEFT, RIGHT)
            start = rospy.Time.now().to_sec()
            delta_step = fz.compute(-ori_data[1] * 180 /
                                    np.pi, gyr_data[1] * 180 / np.pi)/100
            uCOM = COM
            uLEFT = LEFT
            uRIGHT = RIGHT
            # delta_step = 0.07
        elif not(finish):
            # print("STEP")
            now = rospy.Time.now().to_sec() - start
            phase = now / time_step
            
            if phase >= 1:
                phase = 1
                finish = True

            COM = bezier_curve(phase, uCOM, np.matrix(
                [X_OFFSET + (delta_step / 4.0), -0.03, COM_HEIGHT]), np.matrix([X_OFFSET + (delta_step / 2.0), -0.005, COM_HEIGHT]))

            if True:
                LEFT = bezier_curve(phase, uLEFT, np.matrix(
                    [(delta_step / 2.0), 48/1000, 0.08]), np.matrix([(delta_step), 58/1000, 0.0]))
            else:
                RIGHT = bezier_curve(phase, uRIGHT, np.matrix(
                    [(delta_step / 2.0), -48/1000, 0.05]), np.matrix([(delta_step), -48/1000, 0.05]))

            JOINTS = ik.solve(COM, LEFT, RIGHT)

            step = True

        com_msg.x = COM[0,0]
        com_msg.y = COM[0,1]
        com_msg.z = COM[0,2]

    
def final_test(pc, fz, ik):
    global flag, finish, delta_step,step, start, now, JOINTS, COM, LEFT, RIGHT, uCOM, uLEFT, uRIGHT, push_data, com_msg
    time_step = .1
    pc.update_walking_pattern()
    if finish == False:
        if pc.walking_ready and not debug:
            if not(push_data.data) and not(step):
                # print("WALK")
                JOINTS = ik.solve(pc.com_pose, pc.l_foot, pc.r_foot)
                start = rospy.Time.now().to_sec()
                delta_step = fz.compute(-ori_data[1] * 180 /
                                                np.pi, gyr_data[1] * 180 / np.pi)/100
                uCOM = pc.com_pose
                uLEFT = pc.l_foot
                uRIGHT = pc.r_foot

                com_msg.x = uCOM[0,0]
                com_msg.y = uCOM[0,1]
                com_msg.z = uCOM[0,2]

            elif not(finish):
                # print("STEP")
                now = rospy.Time.now().to_sec() - start
                phase = now / time_step
                
                if phase >= 1:
                    phase = 1
                    finish = True

                COM = bezier_curve(phase, uCOM, np.matrix(
                    [X_OFFSET + (delta_step / 4.0), -0.03, COM_HEIGHT]), np.matrix([X_OFFSET + (delta_step / 2.0), 0.01, COM_HEIGHT]))

                if pc.support_foot == -1:
                    LEFT = bezier_curve(phase, uLEFT, np.matrix(
                        [(delta_step / 2.0),  48/1000, 0.08]), np.matrix([(delta_step),  62/1000, 0.0]))
                else:
                    RIGHT = bezier_curve(phase, uRIGHT, np.matrix(
                        [(delta_step / 2.0), -48/1000, 0.08]), np.matrix([(delta_step), -62/1000, 0.0]))

                JOINTS = ik.solve(COM, LEFT, RIGHT)

                step = True

                com_msg.x = COM[0,0]
                com_msg.y = COM[0,1]
                com_msg.z = COM[0,2]
        else:
            JOINTS = ik.solve(COM, LEFT, RIGHT)
            com_msg.x = COM[0,0]
            com_msg.y = COM[0,1]
            com_msg.z = COM[0,2]

def main():
    global X_OFFSET, COM, com_msg
    rospy.init_node('enoid_walk', anonymous=False)
    rospy.Subscriber("ori_data", Vector3, ori_callback)
    rospy.Subscriber("gyr_data", Vector3, gyr_callback)
    rospy.Subscriber("walk_vel", Vector3, vel_callback)
    rospy.Subscriber("walk_cmd_status", Int32, cmd_callback)
    rospy.Subscriber("walk_mode_status", Int32, walk_mode_callback)
    rospy.Subscriber("feedback_status", Int32, feedback_callback)
    rospy.Subscriber("gain_control", Twist, gain_callback)
    rospy.Subscriber("push_data", Bool, push_callback)
    com_pub = rospy.Publisher('com_data', Vector3, queue_size=1)

    rate = rospy.Rate(30)

    pc = PreviewControl(COM_HEIGHT, COM_SWING, X_OFFSET, FOOT_DISTANCE)
    ik = InverseKinematic()
    sc = ServoController()
    fz = FuzzyLogic()
    
    stand(ik)

    rospy.loginfo("E-NOID WALK")

    while not rospy.is_shutdown():
        
        if walk_cmd.data == -1:

            # print("RESET WALK")
            pc = PreviewControl(COM_HEIGHT, COM_SWING, X_OFFSET, FOOT_DISTANCE)
            stand(ik)

        elif walk_cmd.data == 1:

            if walk_mode.data == 1 and (feedback_mode.data == 2):
                ik.TILT = 15
                X_OFFSET = 18 /1000
                final_test(pc, fz, ik)
            elif walk_mode.data == 1:
                ik.TILT = 15
                X_OFFSET = 18 /1000
                walk_test(pc, ik)
            elif (feedback_mode.data == 2):
                ik.TILT = 10
                push_test(fz, ik)
            else :
                stand(ik)
                ik.TILT = 10
                X_OFFSET = 5 /1000

                
            if ((feedback_mode.data == 1) or (feedback_mode.data == 2)) and not(finish):
                # print(ori_data[1] * 180/np.pi)
                # print(gyr_data[1] * 180/np.pi)
                
                # print("ANKLE")
                igl_data[1] +=  ori_data[1]/30

                delta_roll = 0.04 * ori_data[0] + 0.036 * gyr_data[0]
                delta_pitch = -gain_ctrl.linear.y * ori_data[1] + -gain_ctrl.angular.y * gyr_data[1] + gain_ctrl.angular.x * igl_data[1]
                JOINTS[3] += delta_pitch
                JOINTS[8] -= delta_pitch
                # JOINTS[4] += delta_roll
                JOINTS[9] -= delta_roll

        sc.sync_write_pos(JOINTS)

        com_pub.publish(com_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
