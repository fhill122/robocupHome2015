#!/usr/bin/env python
import numpy as np
import argparse
import struct
import sys
import rospy
from math import *
from baxter_interface import CHECK_VERSION
import os
from common_functions import *


from sensor_msgs.msg import PointCloud


from std_msgs.msg import Header

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

closest_pos = [-1,-1,-1]

def main():
    rospy.init_node("sonar_detect")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    rs.enable()

    #publish
	#pub = rospy.Publisher('gripper_test_both/request', gripperTestRequest, latch = True)
	#msg = gripperTestRequest()
	#pub.publish(msg)
    
    rospy.Subscriber("/robot/sonar/head_sonar/state", PointCloud, sonarCb)
    rospy.sleep(1)
    r_axis=[0,-1,0]
    r_angle=pi
    
    [x,y,z,w]=get_quaternion(r_axis, r_angle)
    print closest_pos[0]
    print closest_pos
    theta=atan2(closest_pos[0],closest_pos[1])
    x_cart = 0.4*sin(theta)
    y_cart = 0.7*cos(theta)
    print "X", x_cart
    print "Y", y_cart
    move_arm('left', ik_test("left",x_cart,y_cart, 0.5,0,0,-1,0), 800)
    rospy.spin()
    return 0

def sonarCb(data):
    #~ print len(data.points)
    #~ print data.points
    global closest_pos
    
    min_distance = 9999
    pos = -1
    for i in range(len(data.points)):
        if (data.points[i]!=[0,0,0]):
            if (point_in_poly(data.points[i].x,data.points[i].y,[(0.8,1),(2,1),(2,-1),(0.8,-1)])):
                distance = sqrt((data.points[i].x)*(data.points[i].x)+(data.points[i].y)*(data.points[i].y)+(data.points[i].z)*(data.points[i].z))
                if distance < min_distance:
                    min_distance = distance
                    pos = i
    if pos == -1:
        return
    closest_pos[0] = data.points[pos].x
    closest_pos[1] = data.points[pos].y
    closest_pos[2] = data.points[pos].z
        

if __name__ == '__main__':
    sys.exit(main())
