#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Bool
import subprocess, shlex, psutil, os

record = False

def callback(msg):
    global record
    if(msg.data == 1 and record == False):
        record = True
        command = "rosbag record /rosout"
        command = shlex.split(command)
        callback.rosbag_proc = subprocess.Popen(command)
        rospy.loginfo("START RECORDING")

      
    elif(msg.data == 0 and record == True):
        record = False
        list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
        list_output = list_cmd.stdout.read().decode()
        retcode = list_cmd.wait()
        assert retcode == 0, "List command returned %d" % retcode

        # print(type(list_output))
        for str in list_output.split("\n"):
            if (str.startswith("/record")):
                os.system("rosnode kill " + str)

        callback.rosbag_proc.send_signal(subprocess.signal.SIGINT)
        rospy.loginfo("STOP RECORDING")


def main():
    rospy.init_node('enoid_record', anonymous=False)
    sub = rospy.Subscriber("/record_status", Bool, callback)
    rate = rospy.Rate(10)
    rospy.loginfo("E-NOID RECORD")
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

        
