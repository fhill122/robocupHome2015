#!/usr/bin/env python
##to do adjust for limb right
import numpy as np
import argparse
import struct
import sys
import rospy
from math import *
import baxter_interface
from baxter_interface import CHECK_VERSION
import os
import numpy as np
from copy import copy
import actionlib

from vision.srv import *
from common_functions import *
from Constants import *
import rotate_object
from utilities.srv import *
from utilities.msg import *
import pick_up_cylinder

Z_START=0.4
Z_PICK = TableZ + 0.075

def main():
    #initiate ros, robot, assign variables...
    rospy.init_node("pour_water_demo")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    rs.enable()
    gripper('both','open')

    # limb = "left"
    # angles = dict(zip(baxter_interface.limb.Limb(limb).joint_names(),[-1.0787719878479005, 1.604927397491455, 1.0204807179748536, -0.23469906027832033, 0.2304806131164551, 1.5243934062194826, 1.9527575407470705]))
    # baxter_interface.limb.Limb(limb).move_to_joint_positions(angles)

    #move to natural position
    # baxter_interface.limb.Limb("left").move_to_neutral()
    # baxter_interface.limb.Limb("right").move_to_neutral()
    pick_up_cylinder.move_to_start("left")
    pick_up_cylinder.move_to_start("right")
    print "ready for operation"

    '''step 1: detect cup put on coaster'''
    coaster_position = None
    while( True):
        coaster_position =get_object_position("Coaster")
        if (coaster_position == False):
            break
        last_position = coaster_position
        rospy.sleep(0.01)
    coaster_position = last_position
    print ("detect cup on coaster!"), coaster_position

    '''step 2: check with user'''
    try:
        srv_h=rospy.ServiceProxy("/socket_android/android_interact",android_interact)
        resp =srv_h("Hello,    do you want chocolate m & m's or peanut m & m's ?", "I repeat, which m & m's do you want? chocolate or peanut", "OK", ["chocolate","peanut"])
    except rospy.ServiceException, e:
        print "service dall failed: %s"%e
    if resp.index == 1:
        object = "CoffeeCup"
    else:
        object = "Chocolate"
    print "picked: "+object


    # '''step 3:grab cup on coaster'''
    limb = "right"
    pick_up_cylinder.grip_cylinder(limb,coaster_position)
    move_keep_orientation(limb,0,0,0.15,2)

    '''STEP 4:grab container'''
    # version 1
    limb = "left"
    pick_up_cylinder.find_grip_cylinder(limb,object)
    move_keep_orientation(limb,0,0,0.15,1.5)
    #move closer to baxter
    [r_x,r_y,r_z,r_w, offset_x, offset_y] = find_gesture_cylinder(limb, radians(-90),radians(92), GripperYoffset,GripperZoffset)
    moveTrajectory(limb,[ ik_position_list(limb,0.5+offset_x, 0.3+offset_y,Z_PICK+0.2,r_x,r_y,r_z,r_w) ],[2])
    move_keep_orientation(limb,0,0,-0.2,2.0)
    gripper(limb,'open')
    #move away
    move_keep_orientation(limb,0,0,-0.01,1)
    rospy.sleep(0.5)
    move_keep_orientation(limb,-0.025,0,0,1)
    rospy.sleep(0.5)
    move_keep_orientation(limb,0,0.1,0,1)
    move_keep_orientation(limb,0,0,0.3,1.5)

    # version 2
    # limb = "left"
    # rotate_object.graspTop(limb,find_centre(get_object_position(object)))
    # gripper(limb,"close")
    # move_keep_orientation(limb,0,0,0.2,2)
    # #move to start position
    # [x,y]=(0.5,0.2)
    # [rx,ry,rz,rw,offset_x,offset_y] = rotate_object.graspTopGesture(limb)
    # position_list = [ik_position_list(limb,x+offset_x,y+offset_y,Z_PICK+0.3,rx,ry,rz,rw)]
    # moveTrajectory(limb,position_list,[5])
    # gripper(limb,"open")
    # rospy.sleep(5000)

    '''STEP 5: ROTATE then grab it again'''
    limb = "left"
    rotate_object.rotate_to_original(limb,object)
    move_keep_orientation(limb,0,0,0.1,1.5)
    move_keep_orientation(limb,0,0.2,0.15,3.5)
    # pick up
    pick_up_cylinder.move_to_start(limb)
    pick_up_cylinder.find_grip_cylinder(limb,object)
    # pick_up_cylinder.find_grip_cylinder(limb,object)
    move_keep_orientation(limb,0,0,0.3,1.5)

    '''STEP 6: PRE POURING POSE'''
    moveTrajectory('right',[ik_position_list('right',0.649274286384,-0.124420979527, 0.0556079809065,-0.601030426405,0.447287450997,0.363027540124,0.553992209176)], [5])
    moveTrajectory('left',[ik_position_list('left',0.513975738964,0.121790468159, 0.110592316607,-0.383043274145,-0.597082624267,0.638377581709,-0.298737766446)], [2])
    moveTrajectory('left',[ik_position_list('left',0.543975738964,0.121790468159, 0.110592316607,-0.383043274145,-0.597082624267,0.638377581709,-0.298737766446)], [2])
    moveTrajectory('left',[ik_position_list('left',0.569404851914,0.110451328135, 0.175854846917,-0.00833304041973,-0.69074518654,0.719911802142,0.0672952067089)], [4])
    #pause
    moveTrajectory('left',[ik_position_list('left',0.569404851914,0.110451328135, 0.175854846917,-0.00833304041973,-0.69074518654,0.719911802142,0.0672952067089)], [10])
    moveTrajectory('left',[ik_position_list('left',0.510906641503,0.134156407589, 0.179486134566,0.568424137363,0.504650233009,-0.494327516422,0.421737416997)], [3])

    traj_left = Trajectory("left")
    traj_right = Trajectory("right")

    traj_left.start()
    traj_right.start()
    traj_left.wait(20)
    traj_right.wait(20)
    traj_right.clear("right")
    traj_left.clear("left")

    '''step 8 Give user'''
    limb = "right"
    [r_x,r_y,r_z,r_w, offset_x, offset_y] = find_gesture_cylinder(limb, radians(-60),radians(92), GripperYoffset,GripperZoffset)
    moveTrajectory(limb,[ ik_position_list(limb,0.8+offset_x, -0.1+offset_y,Z_PICK+0.4,r_x,r_y,r_z,r_w) ],[4])
    try:
        srv_h=rospy.ServiceProxy("/socket_android/android_interact",android_interact)
        resp =srv_h("Are you ready to take it?", "Are you ready to take it now? Yes or no", "Here you go", ["yes"])
    except rospy.ServiceException, e:
        print "service dall failed: %s"%e
    gripper(limb,"open")

    '''step 10: put back container'''
    # put
    #peanut
    if object == "CoffeeCup":
        FINAL_X=0.75
        FINAL_Y=0.20
    #chocolate
    else:
        FINAL_X=0.75
        FINAL_Y=0.45

    limb = "left"
    [r_x,r_y,r_z,r_w, offset_x, offset_y] = find_gesture_cylinder(limb, radians(-50),radians(92), GripperYoffset,GripperZoffset)
    moveTrajectory(limb,[ ik_position_list(limb,FINAL_X+offset_x, FINAL_Y+offset_y,Z_PICK+0.2,r_x,r_y,r_z,r_w) ],[2])
    move_keep_orientation(limb,0,0,-0.2,2.0)
    gripper(limb,"open")

    #move away
    # STAGE: y1
    [r_x,r_y,r_z,r_w, offset_x, offset_y] = find_gesture_cylinder(limb, radians(-50), radians(92), 1.03*GripperYoffset, GripperZoffset)
    jointPosition_list=[(ik_position_list(limb,FINAL_X+offset_x, FINAL_Y+offset_y,Z_PICK,r_x,r_y,r_z,r_w))]
    duration=[2]
    # STAGE: z1
    [r_x,r_y,r_z,r_w, offset_x, offset_y] = find_gesture_cylinder(limb, radians(-50), radians(92),  1.03*GripperYoffset, 2.5*GripperZoffset)
    jointPosition_list.append(ik_position_list(limb,FINAL_X+offset_x, FINAL_Y+offset_y,Z_PICK,r_x,r_y,r_z,r_w))
    duration.append (duration[-1]+3)
    moveTrajectory(limb,jointPosition_list,duration)
    move_keep_orientation(limb,0,0,0.2,2)


    return 0



if __name__ == '__main__':
    sys.exit(main())
