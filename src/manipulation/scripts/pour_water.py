#!/usr/bin/env python
##to do adjust for limb right
import numpy as np
import argparse
import struct
import sys
import rospy
import baxter_interface
from math import *
from baxter_interface import CHECK_VERSION
import os
import numpy as np
from copy import copy
import actionlib

from vision.srv import *
from common_functions import *
from Constants import *
from rotate_object import *

from utilities.msg import *

Z_START=0.4
Z_PICK = TableZ + 0.075

def main():
    #initiate ros, robot, assign variables...
    rospy.init_node("pour_water")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    rs.enable()
    
    move_keep_orientation("left",0,-0.4,0,3)
    
    #~ rospy.sleep(50000)
    gripper('both','close')
    rospy.spin()

    ##move to fixed position
    theta = radians(-90)
    alpha = radians(90)
    x=0.4
    y=0.02
    [r_x,r_y,r_z,r_w, offset_x, offset_y] = find_gesture_cylinder(limb, theta,alpha, GripperYoffset, GripperZoffset)
    jointPosition_list= [ik_position_list(limb,x+offset_x, y+offset_y,Z_PICK +0.2,r_x,r_y,r_z,r_w) ]
    duration=[4]
    [r_x,r_y,r_z,r_w, offset_x, offset_y] = find_gesture_cylinder(limb, theta,alpha, GripperYoffset, GripperZoffset)
    jointPosition_list.append (ik_position_list(limb,x+offset_x, y+offset_y,Z_PICK,r_x,r_y,r_z,r_w) )
    duration.append (duration[-1]+2)
    moveTrajectory(limb,jointPosition_list,duration)
      
    os.system("rostopic pub /gripper_test_both/request utilities/gripperTestRequest -1 '[0, now,base_link]' 0 3")
    rospy.sleep(30)
    
    ### move up
    theta = radians(-90)
    alpha = radians(95)
    x=0.55
    y=0.3
    [r_x,r_y,r_z,r_w, offset_x, offset_y] = find_gesture_cylinder(limb, theta,alpha, GripperYoffset, GripperZoffset)
    jointPosition_list = [ ik_position_list(limb,x+offset_x, y+offset_y,Z_PICK+0.15,r_x,r_y,r_z,r_w) ]
    duration = [7]
    
    ### pour
    gamma = radians(-100)
    rospy.sleep(2)
    [r_x,r_y,r_z,r_w, offset_x, offset_y] = find_gesture_cylinder_pour(limb, theta,alpha, gamma, GripperYoffset, GripperZoffset)
    jointPosition_list.append ( ik_position_list(limb,x+offset_x, y+offset_y,Z_PICK+0.15,r_x,r_y,r_z,r_w) )
    duration.append (duration[-1]+10)
    
    ###pause
    jointPosition_list.append ( ik_position_list(limb,x+offset_x, y+offset_y,Z_PICK+0.15,r_x,r_y,r_z,r_w) )
    duration.append (duration[-1]+10)
    
    ### unpour
    [r_x,r_y,r_z,r_w, offset_x, offset_y] = find_gesture_cylinder(limb, theta,alpha, GripperYoffset, GripperZoffset)
    jointPosition_list.append (ik_position_list(limb,x+offset_x, y+offset_y,Z_PICK+0.15,r_x,r_y,r_z,r_w) )
    duration.append (duration[-1]+3)
    
    ###place
    x=0.45
    y=0.35
    [r_x,r_y,r_z,r_w, offset_x, offset_y] = find_gesture_cylinder(limb, theta,alpha, GripperYoffset, GripperZoffset)
    jointPosition_list.append (ik_position_list(limb,x+offset_x, y+offset_y,Z_PICK,r_x,r_y,r_z,r_w) )
    duration.append (duration[-1]+3)
    
    for i in range(len(jointPosition_list)):
		traj.add_point(jointPosition_list[i], duration[i])
    traj.start()
    traj.wait(30.0)
    traj.clear(limb)
    
    os.system("rostopic pub /gripper_test_both/request utilities/gripperTestRequest -1 '[0, now,base_link]' 0 3")
    

    return 0
    
if __name__ == '__main__':
    sys.exit(main())
