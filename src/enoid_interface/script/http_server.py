#! /usr/bin/env python3
import os
import rospy
from http.server import HTTPServer, CGIHTTPRequestHandler

hostName = "0.0.0.0"
serverPort = 8000

if __name__ == "__main__":
    os.chdir("/home/mikael/catkin_ws/src/enoid_interface")
    rospy.init_node("enoid_interface", anonymous=False)
    rate = rospy.Rate(30)
    rospy.loginfo("E-NOID INTERFACE")

    while not rospy.is_shutdown():
        webServer = HTTPServer((hostName, serverPort), RequestHandlerClass=CGIHTTPRequestHandler)
        print("Server started http://%s:%s" % (hostName, serverPort))

        try:
            webServer.serve_forever()
        except rospy.ROSInterruptException:
            pass

        webServer.server_close()
        print("Server stopped.")