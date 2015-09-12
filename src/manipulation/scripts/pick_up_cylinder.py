#!/usr/bin/env python
import numpy as np
import argparse
import struct
import sys
import rospy
import baxter_interface
from math import *
from baxter_interface import CHECK_VERSION
import os
from vision.srv import *
from common_functions import *
from Constants import *

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from std_msgs.msg import Header

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
    
def main():
      
    #initiate ros, robot, assign variables...
    rospy.init_node("rsdk_set_position")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    rs.enable()
    
    single_arm_pick("left","CoffeeCup")
    

    return 0
    
## remember to import TableZ, GripperLength before calling this function
# alpha in degree, lim= "left" or "right"
def single_arm_pick(limb,object):
    
    ##constants
    Z_PICK = TableZ + 0.065
    theta = radians(-90)
    #object radius adjustment
    r=0
    offset = 0.1
    X_START=0.4
    Y_START=0.5
    Z_START=0.3
    R_X=0.603489643517
    R_Y=0.634442665138
    R_Z=-0.335770984639
    R_W=0.347189574577
    traj = Trajectory(limb)
    rospy.on_shutdown(traj.stop)
    
    #start position, no block of camera
    traj.add_point(ik_position_list(limb,X_START,Y_START,Z_START,R_X,R_Y,R_Z,R_W), 5)
    traj.start()
    traj.wait(10.0)
    traj.clear(limb)

    os.system("rostopic pub /gripper_test_both/request utilities/gripperTestRequest -1 '[0, now,base_link]' 0 3")
    
    #get plate position
    [x,y]=find_centre(get_object_position(object))
    print "centre position: %s\n"%[x,y]
    
    ##motion planning
    step=2.5
    times=1
    alternating = -1
    foundAngle = 0
    jointPosition_list = []
    duration = []
    while abs(degrees(theta) -(- 90))<30:
		#away and higher from object
		[r_x,r_y,r_z,r_w, offset_x, offset_y] = find_gesture_cylinder(limb, theta,1.1*GripperYoffset, 2*GripperZoffset)
		jointPosition_list = [ ik_position_list(limb,x+offset_x, y+offset_y,Z_PICK+0.13,r_x,r_y,r_z,r_w) ]
		duration = [4]
		#lower
		jointPosition_list.append(ik_position_list(limb,x+offset_x, y+offset_y,Z_PICK,r_x,r_y,r_z,r_w))
		duration.append (duration[-1]+2)
		#closer in z1 direnction
		[r_x,r_y,r_z,r_w, offset_x, offset_y] = find_gesture_cylinder(limb, theta,1.1*GripperYoffset, GripperZoffset)
		jointPosition_list.append(ik_position_list(limb,x+offset_x, y+offset_y,Z_PICK,r_x,r_y,r_z,r_w))
		duration.append (duration[-1]+2)
		#tighter in y1 direction
		[r_x,r_y,r_z,r_w, offset_x, offset_y] = find_gesture_cylinder(limb, theta,GripperYoffset, GripperZoffset)
		jointPosition_list.append(ik_position_list(limb,x+offset_x, y+offset_y,Z_PICK,r_x,r_y,r_z,r_w))
		duration.append (duration[-1]+1)
		#tighter in z1 direction
		[r_x,r_y,r_z,r_w, offset_x, offset_y] = find_gesture_cylinder(limb, theta,GripperYoffset, 0.95*GripperZoffset)
		jointPosition_list.append(ik_position_list(limb,x+offset_x, y+offset_y,Z_PICK,r_x,r_y,r_z,r_w))
		duration.append (duration[-1]+1)
		
		#check if found solution
		foundAngle = 1
		for i in jointPosition_list:
			if i == 2:
				foundAngle = 0
		if foundAngle ==1:
			break
		else:
			theta = -90 - step*times*alternating
			times = times+1
			alternating = -alternating
			
	if foundAngle =0
		print "\n!!!No solution found!!! Quit\n"
		return 1
		
    #~ #away and higher from object
    #~ [r_x,r_y,r_z,r_w, offset_x, offset_y] = find_gesture_cylinder(limb, theta,1.1*GripperYoffset, 2*GripperZoffset)
    #~ traj.add_point(ik_position_list(limb,x+offset_x, y+offset_y,Z_PICK+0.13,r_x,r_y,r_z,r_w), 4.0)
    #~ #lower
    #~ traj.add_point(ik_position_list(limb,x+offset_x, y+offset_y,Z_PICK,r_x,r_y,r_z,r_w), 6.0)
    #~ 
    #~ #closer
    #~ [r_x,r_y,r_z,r_w, offset_x, offset_y] = find_gesture_cylinder(limb, theta,1.1*GripperYoffset, GripperZoffset)
    #~ traj.add_point(ik_position_list(limb,x+offset_x, y+offset_y,Z_PICK,r_x,r_y,r_z,r_w), 8.0)
    #~ 
    #~ #tighter in y1 direction
    #~ [r_x,r_y,r_z,r_w, offset_x, offset_y] = find_gesture_cylinder(limb, theta,GripperYoffset, GripperZoffset)
    #~ traj.add_point(ik_position_list(limb,x+offset_x, y+offset_y,Z_PICK,r_x,r_y,r_z,r_w), 9.0)
    #~ #tighter in z1 direction
    #~ [r_x,r_y,r_z,r_w, offset_x, offset_y] = find_gesture_cylinder(limb, theta,GripperYoffset, 0.95*GripperZoffset)
    #~ traj.add_point(ik_position_list(limb,x+offset_x, y+offset_y,Z_PICK,r_x,r_y,r_z,r_w), 10.0)
    
    print "\n picking up using theta = ",theta, "\n"
    
    for i in range(len(jointPosition_list)):
		traj.add(jointPosition_list[i], duration[i])
    traj.start()
    traj.wait(10.0)
    traj.clear(limb)
    
    os.system("rostopic pub /gripper_test_both/request utilities/gripperTestRequest -1 '[0, now,base_link]' 1 3")
    
    return 0
    

if __name__ == '__main__':
    sys.exit(main())
