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
from move_plate_to_start_position import *

import Coordinated_Motion


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
    rospy.init_node("dual_arm_pick")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    rs.enable()
    gripper('both','open')
    
    #pickup
    alpha = 35
    #VARIABLES
    RIGHT = 0
    LEFT = 1
    DUAL = 3
    
    #~ ##If incorrect remove from here to the end of the if statements and replace "single_arm_pick("right",alpha)"
    plate_position=get_object_position("Plate")
    [x,y] = find_centre(plate_position)

    if (point_in_poly(x,y,poly_dual)==True):
        pickup_state = DUAL
        print "Straight to Dual Arm"
    elif (point_in_poly(x,y,poly_left)==True):
        pickup_state = LEFT
        print "Left Arm Pick-Up"
        single_arm_pick("left",alpha)    
    else:
        pickup_state = RIGHT
        print "RIGHT Arm Pick-Up"
        single_arm_pick("right",alpha)
        
           
    #single_arm_pick("left",alpha)    
    Z_PICK = TableZ + sin(radians(alpha)) * GripperLength +0.008
    ## start position
    X_START=0.621888692162
    Y_START= - 0.46333973238
    Z_START=0.193484766029
    R_X=-0.832475437793
    R_Y=0.00871415462847
    R_Z=0.0230735969917
    R_W=0.553512708167
    
    #moveTrajectory("right", [ik_position_list("right",X_START,Y_START,Z_START,R_X,R_Y,R_Z,R_W)] ,[3.5])
    rospy.sleep(2)
    plate_position=get_object_position("Plate")
    print plate_position
    if (pickup_state == DUAL):
        ## move
        limb = "left"
        traj = Trajectory(limb)
        #~ #move to plate
        [[p1x,p1y],[p4x,p4y]]=find_edge(limb,plate_position)
        [r_x,r_y,r_z,r_w, offset_x, offset_y] = find_gesture(limb,[[p1x,p1y],[p4x,p4y]],alpha, GripperLength*cos(radians(alpha)) )
        traj.add_point(ik_position_list(limb,(p1x+p4x)/2+1.2*offset_x,(p1y+p4y)/2+1.2*offset_y,Z_PICK+0.06,r_x,r_y,r_z,r_w), 6.0)
        #lower z position
        traj.add_point(ik_position_list(limb,(p1x+p4x)/2+1.2*offset_x,(p1y+p4y)/2+1.2*offset_y,Z_PICK,r_x,r_y,r_z,r_w), 7.0)
        #closer
        traj.add_point(ik_position_list(limb,(p1x+p4x)/2+0.85*offset_x,(p1y+p4y)/2+0.85*offset_y,Z_PICK,r_x,r_y,r_z,r_w), 8.0)
        
        limb = "right"
        traj2 = Trajectory(limb)
        #~ #move to plate
        [[p2x,p2y],[p3x,p3y]]=find_edge(limb,plate_position)
        [r_x,r_y,r_z,r_w, offset_x, offset_y] = find_gesture(limb,[[p2x,p2y],[p3x,p3y]],alpha, GripperLength*cos(radians(alpha)) )
        traj2.add_point(ik_position_list(limb,(p2x+p3x)/2+1.2*offset_x,(p2y+p3y)/2+1.2*offset_y,Z_PICK+0.06,r_x,r_y,r_z,r_w), 6.0)
        #lower z position
        traj2.add_point(ik_position_list(limb,(p2x+p3x)/2+1.2*offset_x,(p2y+p3y)/2+1.2*offset_y,Z_PICK,r_x,r_y,r_z,r_w), 7.0)
        #closer
        traj2.add_point(ik_position_list(limb,(p2x+p3x)/2+0.85*offset_x,(p2y+p3y)/2+0.85*offset_y,Z_PICK,r_x,r_y,r_z,r_w), 8.0)
        
        traj.start()
        traj2.start()
        traj.wait(20)
        traj2.wait(20)
        traj2.clear("right")
        traj.clear("left")
        os.system("rostopic pub /gripper_test_both/request utilities/gripperTestRequest -1 '[0, now,base_link]' 1 5")
        
    elif (pickup_state == LEFT):
        ## move
        limb = "left"
        traj = Trajectory(limb)
        #move to plate
        [[p1x,p1y],[p4x,p4y]]=find_edge(limb,plate_position)
        #~ [r_x,r_y,r_z,r_w, offset_x, offset_y] = find_gesture(limb,[[p1x,p1y],[p4x,p4y]],alpha, GripperLength*cos(radians(alpha)) )
        #~ traj.add_point(ik_position_list(limb,(p1x+p4x)/2+1.2*offset_x,(p1y+p4y)/2+1.2*offset_y,Z_PICK+0.06,r_x,r_y,r_z,r_w), 6.0)
        #~ #lower z position
        #~ traj.add_point(ik_position_list(limb,(p1x+p4x)/2+1.2*offset_x,(p1y+p4y)/2+1.2*offset_y,Z_PICK,r_x,r_y,r_z,r_w), 7.0)
        #~ #closer
        #~ traj.add_point(ik_position_list(limb,(p1x+p4x)/2+0.85*offset_x,(p1y+p4y)/2+0.85*offset_y,Z_PICK,r_x,r_y,r_z,r_w), 8.0)
        
        limb = "right"
        traj2 = Trajectory(limb)
        #~ #move to plate
        [[p2x,p2y],[p3x,p3y]]=find_edge(limb,plate_position)
        [r_x,r_y,r_z,r_w, offset_x, offset_y] = find_gesture(limb,[[p2x,p2y],[p3x,p3y]],alpha, GripperLength*cos(radians(alpha)) )
        traj2.add_point(ik_position_list(limb,(p2x+p3x)/2+1.2*offset_x,(p2y+p3y)/2+1.2*offset_y,Z_PICK+0.06,r_x,r_y,r_z,r_w), 6.0)
        #lower z position
        traj2.add_point(ik_position_list(limb,(p2x+p3x)/2+1.2*offset_x,(p2y+p3y)/2+1.2*offset_y,Z_PICK,r_x,r_y,r_z,r_w), 7.0)
        #closer
        traj2.add_point(ik_position_list(limb,(p2x+p3x)/2+0.85*offset_x,(p2y+p3y)/2+0.85*offset_y,Z_PICK,r_x,r_y,r_z,r_w), 8.0)
        
        #traj.start()
        traj2.start()
        #traj.wait(20)
        traj2.wait(20)
        traj2.clear("right")
        traj.clear("left")
        os.system("rostopic pub /gripper_test_both/request utilities/gripperTestRequest -1 '[0, now,base_link]' 1 5")
        
    elif (pickup_state == RIGHT):
        ## move
        limb = "left"
        traj = Trajectory(limb)
        #~ #move to plate
        [[p1x,p1y],[p4x,p4y]]=find_edge(limb,plate_position)
        [r_x,r_y,r_z,r_w, offset_x, offset_y] = find_gesture(limb,[[p1x,p1y],[p4x,p4y]],alpha, GripperLength*cos(radians(alpha)) )
        traj.add_point(ik_position_list(limb,(p1x+p4x)/2+1.2*offset_x,(p1y+p4y)/2+1.2*offset_y,Z_PICK+0.06,r_x,r_y,r_z,r_w), 6.0)
        #lower z position
        traj.add_point(ik_position_list(limb,(p1x+p4x)/2+1.2*offset_x,(p1y+p4y)/2+1.2*offset_y,Z_PICK,r_x,r_y,r_z,r_w), 7.0)
        #closer
        traj.add_point(ik_position_list(limb,(p1x+p4x)/2+0.85*offset_x,(p1y+p4y)/2+0.85*offset_y,Z_PICK,r_x,r_y,r_z,r_w), 8.0)
        
        limb = "right"
        traj2 = Trajectory(limb)
        #move to plate
        [[p2x,p2y],[p3x,p3y]]=find_edge(limb,plate_position)
        #~ [r_x,r_y,r_z,r_w, offset_x, offset_y] = find_gesture(limb,[[p2x,p2y],[p3x,p3y]],alpha, GripperLength*cos(radians(alpha)) )
        #~ traj2.add_point(ik_position_list(limb,(p2x+p3x)/2+1.2*offset_x,(p2y+p3y)/2+1.2*offset_y,Z_PICK+0.06,r_x,r_y,r_z,r_w), 6.0)
        #~ #lower z position
        #~ traj2.add_point(ik_position_list(limb,(p2x+p3x)/2+1.2*offset_x,(p2y+p3y)/2+1.2*offset_y,Z_PICK,r_x,r_y,r_z,r_w), 7.0)
        #~ #closer
        #~ traj2.add_point(ik_position_list(limb,(p2x+p3x)/2+0.85*offset_x,(p2y+p3y)/2+0.85*offset_y,Z_PICK,r_x,r_y,r_z,r_w), 8.0)
        
        traj.start()
        #traj2.start()
        traj.wait(20)
        #traj2.wait(20)
        traj2.clear("right")
        traj.clear("left")
        os.system("rostopic pub /gripper_test_both/request utilities/gripperTestRequest -1 '[0, now,base_link]' 1 5")
    
    # up
    alpha=0
    [r_x,r_y,r_z,r_w, offset_x, offset_y] = find_gesture("right",[[p2x,p2y],[p3x,p3y]],alpha, GripperLength*cos(radians(alpha)) )
    traj2.add_point(ik_position_list("right",(p2x+p3x)/2+0.7*offset_x +0.1,(p2y+p3y)/2+0.7*offset_y +0.05,Z_PICK+0.2,r_x,r_y,r_z,r_w), 4.0)
    
    hand_r = [(p2x+p3x)/2+0.7*offset_x +0.1, (p2y+p3y)/2+0.7*offset_y +0.05, Z_PICK+0.2]
    
    [[p1x,p1y],[p4x,p4y]]=find_edge("left",plate_position)
    [r_x,r_y,r_z,r_w, offset_x, offset_y] = find_gesture("left",[[p1x,p1y],[p4x,p4y]],alpha, GripperLength*cos(radians(alpha)) )
    traj.add_point(ik_position_list("left",(p1x+p4x)/2+0.7*offset_x +0.1,(p1y+p4y)/2+1.0*offset_y +0.05,Z_PICK+0.2,r_x,r_y,r_z,r_w), 4.0)
    traj.start()
    traj2.start()
    
    hand_l = [(p1x+p4x)/2+0.7*offset_x +0.1,(p1y+p4y)/2+1.0*offset_y +0.05,Z_PICK+0.2]
    
    centre = [(hand_l[0]+hand_r[0])/2 , (hand_l[1]+hand_r[1])/2 , (hand_l[2]+hand_r[2])/2, 0.0]
    
    traj.wait(10)
    traj2.wait(10)
    traj2.clear("right")
    traj.clear("left")
    
    os.system("rosnode kill /rsdk_position_joint_trajectory_action_server")
    rospy.sleep(2)
    angle_spin = 15
    pos = Coordinated_Motion.move_co(centre, [0.6, 0.0, 0.2, 0.0])
    pos = Coordinated_Motion.move_co(pos, [0.6, 0.0, 0.2, -angle_spin])    
    pos = Coordinated_Motion.move_co(pos, [0.6, 0.0, 0.2, angle_spin])
    pos = Coordinated_Motion.move_co(pos, [0.6, 0.0, 0.2, 0.0])  
    pos = Coordinated_Motion.move_co(pos, [0.6, -0.17, 0.2, 0.0])
    pos = Coordinated_Motion.move_co(pos, [0.6, 0.0,0.5,0.0] )
    pos = Coordinated_Motion.move_co(pos, [0.6, 0.0, 0.3, 0.0])
    pos = Coordinated_Motion.move_co(pos,centre)
    rospy.sleep(2)
    
    position_right=[]
    position_left=[]
    position_right_seg=[]
    position_left_seg=[]
    position=[]
    
    steps_drop = 16
    limb = "right"
    for i in range(steps_drop):
        angle_inc = 35.0/(steps_drop-1)
        Z_inc = (0.2-0.18)/(steps_drop-1)
        alpha=i*angle_inc
        [r_x,r_y,r_z,r_w, offset_x, offset_y] = find_gesture("right",[[p2x,p2y],[p3x,p3y]],alpha, GripperLength*cos(radians(alpha)))
        position_right.append([(p2x+p3x)/2+0.7*offset_x +0.1,(p2y+p3y)/2+0.7*offset_y +0.05,Z_PICK+0.2-Z_inc*i,r_x,r_y,r_z,r_w])
    
    [r_x,r_y,r_z,r_w, offset_x, offset_y] = find_gesture(limb,[[p2x,p2y],[p3x,p3y]],alpha, GripperLength*cos(radians(alpha)) )
    position_right.append([(p2x+p3x)/2+0.85*offset_x,(p2y+p3y)/2+0.85*offset_y,Z_PICK,r_x,r_y,r_z,r_w])
    position_right.append([(p2x+p3x)/2+0.85*offset_x,(p2y+p3y)/2+1.0*offset_y,Z_PICK-0.008,r_x,r_y,r_z,r_w])
    position_right.append([(p2x+p3x)/2+1.2*offset_x,(p2y+p3y)/2+1.2*offset_y,Z_PICK-0.008,r_x,r_y,r_z,r_w])
    position_right.append([(p2x+p3x)/2+1.2*offset_x,(p2y+p3y)/2+1.2*offset_y,Z_PICK+0.06,r_x,r_y,r_z,r_w])
    
    limb = "left"
    for i in range(steps_drop):
        angle_inc = 35.0/(steps_drop-1)
        Z_inc = (0.2-0.18)/(steps_drop-1)
        alpha=i*angle_inc
        [r_x,r_y,r_z,r_w, offset_x, offset_y] = find_gesture("left",[[p1x,p1y],[p4x,p4y]],alpha, GripperLength*cos(radians(alpha)) )
        position_left.append([(p1x+p4x)/2+0.7*offset_x +0.1,(p1y+p4y)/2+1.0*offset_y +0.05,Z_PICK+0.2-Z_inc*i,r_x,r_y,r_z,r_w])

    [r_x,r_y,r_z,r_w, offset_x, offset_y] = find_gesture(limb,[[p1x,p1y],[p4x,p4y]],alpha, GripperLength*cos(radians(alpha)) )
    position_left.append([(p1x+p4x)/2+0.85*offset_x,(p1y+p4y)/2+0.85*offset_y,Z_PICK,r_x,r_y,r_z,r_w])
    position_left.append([(p1x+p4x)/2+0.85*offset_x,(p1y+p4y)/2+1.0*offset_y,Z_PICK-0.008,r_x,r_y,r_z,r_w])
    position_left.append([(p1x+p4x)/2+1.2*offset_x,(p1y+p4y)/2+1.2*offset_y,Z_PICK-0.008,r_x,r_y,r_z,r_w])
    position_left.append([(p1x+p4x)/2+1.2*offset_x,(p1y+p4y)/2+1.2*offset_y,Z_PICK+0.06,r_x,r_y,r_z,r_w])
    
    
    for i in range(len(position_left)-1):
        position_right_seg.append(splitter_single(position_right[i], position_right[i+1]))
        position_left_seg.append(splitter_single(position_left[i], position_left[i+1]))
    
    arm_l = baxter_interface.Limb('left')
    arm_r = baxter_interface.Limb('right')
    grip=True
    for j in range(min(len(position_left_seg),len(position_right_seg))):
        if j>=steps_drop:
            rospy.sleep(0.1)
        for k in range(min(len(position_left_seg[j]),len(position_right_seg[j]))):
            print "step no.", j
            print "segment no.", k
            position = ik_test(position_left_seg[j][k],position_right_seg[j][k])
            i=1
            if grip==True and j>steps_drop:
                grip=False
                os.system("rostopic pub /gripper_test_both/request utilities/gripperTestRequest -1 '[0, now,base_link]' 0 5")
            while not ((i>1.0)or(rospy.is_shutdown())):
                arm_l.set_joint_positions(position[0])
                arm_r.set_joint_positions(position[1])
                rospy.sleep(.01)
                i=i+1
    os.system("rostopic pub /gripper_test_both/request utilities/gripperTestRequest -1 '[0, now,base_link]' 1 5")
    print "\n*********\nRERUN Trajectory Server\n*********\nRERUN Trajectory Server\n*********\nRERUN Trajectory Server\n*********\n"
    return 0

if __name__ == '__main__':
    sys.exit(main())
