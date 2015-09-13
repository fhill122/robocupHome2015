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

X_START=0.4
Y_START=0.63
Z_START=0.4
Z_PICK = TableZ + 0.075

def main():
      
    #initiate ros, robot, assign variables...
    rospy.init_node("rsdk_set_position")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    rs.enable()
    traj = Trajectory("left")
    rospy.on_shutdown(traj.stop)
    limb = "left"
    
    ##test rotate bottle
    rotateBottle(limb,"CoffeeCup")
    rospy.sleep(3000)
    
    ##pick up
    endPosition = single_arm_pick(limb,"CoffeeCup")
    
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

def rotateBottle(limb,object):
    #~ bottlePosition = get_object_position(object)
    y_angle = radians(179)
    Ry = np.mat([ [cos(y_angle), 0, sin(y_angle)],[0, 1, 0],[-sin(y_angle), 0, cos(y_angle)] ])
    theta = radians(90)
    Rz = np.mat([ [cos(theta), -sin(theta), 0],[sin(theta), cos(theta), 0],[0, 0, 1] ])
    
    R= Ry*Rz
    
    w =1.*sqrt(1+R.item(0,0)+R.item(1,1)+R.item(2,2)) /2
    x =1.*(R.item(2,1)-R.item(1,2)) / (4*w)
    y =1.*(R.item(0,2)-R.item(2,0)) / (4*w)
    z =1.*(R.item(1,0)-R.item(0,1)) / (4*w)
    
    position_list = [ik_position_list(limb,X_START,Y_START,Z_START,x,y,z,w)]
    moveTrajectory(limb,position_list,[5])

## remember to import TableZ, GripperLength before calling this function
# alpha in degree, lim= "left" or "right"
def single_arm_pick(limb,object):
    
    traj = Trajectory(limb)
    ##constants
    theta = radians(-90)
    alpha = radians(95)
    #object radius adjustment
    r=0
    offset = 0.1
    X_START=0.4
    Y_START=0.63
    Z_START=0.4
    R_X=0.603489643517
    R_Y=0.634442665138
    R_Z=-0.335770984639
    R_W=0.347189574577

    
    #start position, no block of camera
    traj.add_point(ik_position_list(limb,X_START,Y_START,Z_START,R_X,R_Y,R_Z,R_W), 6)
    traj.start()
    traj.wait(10.0)
    traj.clear(limb)

    #~ os.system("rostopic pub /gripper_test_both/request utilities/gripperTestRequest -1 '[0, now,base_link]' 0 3")
    
    #get plate position
    [x,y]=find_centre(get_object_position(object))
    print "centre position: %s\n"%[x,y]
    
    ##motion planning
    step= 5
    times=1
    alternating = -1
    foundAngle = 0
    jointPosition_list = []
    duration = []
    while abs(degrees(theta) -(- 90))<90:
        print "testing theta = ",degrees(theta), "\n"
		#away and higher from object
        [r_x,r_y,r_z,r_w, offset_x, offset_y] = find_gesture_cylinder(limb, theta,alpha, 1.5*GripperYoffset, 2*GripperZoffset)
        jointPosition_list = [ ik_position_list(limb,x+offset_x, y+offset_y,Z_PICK+0.13,r_x,r_y,r_z,r_w) ]
        duration = [4]
        #lower
        jointPosition_list.append(ik_position_list(limb,x+offset_x, y+offset_y,Z_PICK,r_x,r_y,r_z,r_w))
        duration.append (duration[-1]+2)
        #pause
        jointPosition_list.append(ik_position_list(limb,x+offset_x, y+offset_y,Z_PICK,r_x,r_y,r_z,r_w))
        duration.append (duration[-1]+1)
        #closer in z1 direnction
        [r_x,r_y,r_z,r_w, offset_x, offset_y] = find_gesture_cylinder(limb, theta,alpha,  1.5*GripperYoffset, 1.2*GripperZoffset)
        jointPosition_list.append(ik_position_list(limb,x+offset_x, y+offset_y,Z_PICK,r_x,r_y,r_z,r_w))
        duration.append (duration[-1]+4)
        #pause
        jointPosition_list.append(ik_position_list(limb,x+offset_x, y+offset_y,Z_PICK,r_x,r_y,r_z,r_w))
        duration.append (duration[-1]+1)
        #tighter in y1 direction
        [r_x,r_y,r_z,r_w, offset_x, offset_y] = find_gesture_cylinder(limb, theta,alpha,  0.75*GripperYoffset, 0.9*GripperZoffset)
        jointPosition_list.append(ik_position_list(limb,x+offset_x, y+offset_y,Z_PICK,r_x,r_y,r_z,r_w))
        duration.append (duration[-1]+2)
        #~ #tighter in z1 direction
        #~ [r_x,r_y,r_z,r_w, offset_x, offset_y] = find_gesture_cylinder(limb, theta,alpha,  0*GripperYoffset, 0.5*GripperZoffset)
        #~ jointPosition_list.append(ik_position_list(limb,x+offset_x, y+offset_y,Z_PICK,r_x,r_y,r_z,r_w))
        #~ duration.append (duration[-1]+2)
        
        #check if found solution
        foundAngle = 1
        for i in jointPosition_list:
            if i == 2:
                foundAngle = 0
        
        #break if found a solution
        #~ print "foundAngle = %s\n"%foundAngle
        if foundAngle ==1:
            break
        else:
            theta = radians(-90 - step*times*alternating)
            times = times+1
            alternating = -alternating
            #~ print "testing theta = ",degrees(theta), "\n"
			
    if foundAngle == 0:
        print "\n!!!No solution found!!! Quit\n"
        return 1
		
    
    print "\n picking up using theta = ",degrees(theta), "\n"
    
    for i in range(len(jointPosition_list)):
		traj.add_point(jointPosition_list[i], duration[i])
    traj.start()
    traj.wait(30.0)
    traj.clear(limb)
    
    os.system("rostopic pub /gripper_test_both/request utilities/gripperTestRequest -1 '[0, now,base_link]' 1 3")
    
    ##up
    [r_x,r_y,r_z,r_w, offset_x, offset_y] = find_gesture_cylinder(limb, theta,90,  0.75*GripperYoffset, 0.9*GripperZoffset)
    jointPosition_list.append(ik_position_list(limb,x+offset_x, y+offset_y,Z_PICK+0.2,r_x,r_y,r_z,r_w))
    
    
    return 0
    

if __name__ == '__main__':
    sys.exit(main())
