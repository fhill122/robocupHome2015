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
    alpha=35
    single_arm_pick("right",alpha)
    
    Z_PICK = TableZ + sin(radians(alpha)) * GripperLength +0.008
    ## start position
    X_START=0.621888692162
    Y_START= - 0.46333973238
    Z_START=0.193484766029
    R_X=-0.832475437793
    R_Y=0.00871415462847
    R_Z=0.0230735969917
    R_W=0.553512708167
    moveTrajectory("right", [ik_position_list("right",X_START,Y_START,Z_START,R_X,R_Y,R_Z,R_W)] ,[3.5])
    
    plate_position=get_object_position("Plate")
    
    ## move
    os.system("rostopic pub /gripper_test_both/request utilities/gripperTestRequest -1 '[0, now,base_link]' 0 5")
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
    [[p1x,p1y],[p4x,p4y]]=find_edge(limb,plate_position)
    [r_x,r_y,r_z,r_w, offset_x, offset_y] = find_gesture(limb,[[p1x,p1y],[p4x,p4y]],alpha, GripperLength*cos(radians(alpha)) )
    traj2.add_point(ik_position_list(limb,(p1x+p4x)/2+1.2*offset_x,(p1y+p4y)/2+1.2*offset_y,Z_PICK+0.06,r_x,r_y,r_z,r_w), 6.0)
    #lower z position
    traj2.add_point(ik_position_list(limb,(p1x+p4x)/2+1.2*offset_x,(p1y+p4y)/2+1.2*offset_y,Z_PICK,r_x,r_y,r_z,r_w), 7.0)
    #closer
    traj2.add_point(ik_position_list(limb,(p1x+p4x)/2+0.85*offset_x,(p1y+p4y)/2+0.85*offset_y,Z_PICK,r_x,r_y,r_z,r_w), 8.0)
    
    traj.start()
    traj2.start()
    traj.wait(20)
    traj2.wait(20)
    traj2.clear("right")
    traj.clear("left")
    os.system("rostopic pub /gripper_test_both/request utilities/gripperTestRequest -1 '[0, now,base_link]' 1 5")
    
    # up
    alpha=0
    [r_x,r_y,r_z,r_w, offset_x, offset_y] = find_gesture("right",[[p1x,p1y],[p4x,p4y]],alpha, GripperLength*cos(radians(alpha)) )
    traj2.add_point(ik_position_list("right",(p1x+p4x)/2+0.7*offset_x +0.1,(p1y+p4y)/2+0.7*offset_y +0.05,Z_PICK+0.2,r_x,r_y,r_z,r_w), 4.0)
    
    hand_r = [(p1x+p4x)/2+0.7*offset_x +0.1, (p1y+p4y)/2+0.7*offset_y +0.05, Z_PICK+0.2]
    
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
    
    #~ rospy.sleep(10)
    #~ 
    #~ traj2.clear("right")
    #~ traj.clear("left")
    #~ Coordinated_Motion.move_co(centre)
    #~ os.system("/home/robocuphome/ros_ws/src/baxter_examples/scripts/Coordinated_Motion.py"+str(centre[0])+str(centre[1])+str(centre[2]))
    
    os.system("rosnode kill /rsdk_position_joint_trajectory_action_server")
    rospy.sleep(2)
    Coordinated_Motion.move_co(centre)
    os.system(" rosrun baxter_interface joint_trajectory_action_server.py ")
    return 0

if __name__ == '__main__':
    sys.exit(main())
