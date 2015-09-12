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
    
    single_arm_pick("left",35)
    

    return 0
    
## remember to import TableZ, GripperLength before calling this function
# alpha in degree, lim= "left" or "right"
def single_arm_pick(limb,alpha):
    
    Z_PICK = TableZ + sin(radians(alpha)) * GripperLength +0.013
    
    X_START=0.621888692162
    Y_START=0.46333973238
    Z_START=0.193484766029
    R_X=0.832475437793
    R_Y=-0.00871415462847
    R_Z=-0.0230735969917
    R_W=0.553512708167
    
    #plate final position
    p1x_final = 0.6491340398788452
    p1y_final = 0.18462590873241425
    p4x_final = 0.3690818250179291
    p4y_final = 0.18592007458209991
    
    traj = Trajectory(limb)
    rospy.on_shutdown(traj.stop)
    
    #get plate position
    plate_position=get_object_position("Plate")
    print "plate position %s"%(plate_position)
    
    ## move
    #start position
    traj.add_point(ik_position_list(limb,X_START,Y_START,Z_START,R_X,R_Y,R_Z,R_W), 3.5)
    #~ #move to plate
    [[p1x,p1y],[p4x,p4y]]=find_edge(limb,plate_position)
    [r_x,r_y,r_z,r_w, offset_x, offset_y] = find_gesture(limb,[[p1x,p1y],[p4x,p4y]],alpha, GripperLength*cos(radians(alpha)) )
    traj.add_point(ik_position_list(limb,(p1x+p4x)/2+1.2*offset_x,(p1y+p4y)/2+1.2*offset_y,Z_PICK+0.06,r_x,r_y,r_z,r_w), 6.0)
    #lower z position
    traj.add_point(ik_position_list(limb,(p1x+p4x)/2+1.2*offset_x,(p1y+p4y)/2+1.2*offset_y,Z_PICK,r_x,r_y,r_z,r_w), 7.0)
    #closer
    traj.add_point(ik_position_list(limb,(p1x+p4x)/2+0.7*offset_x,(p1y+p4y)/2+0.7*offset_y,Z_PICK,r_x,r_y,r_z,r_w), 8.0)
    traj.start()
    traj.wait(10.0)
    traj.clear(limb)
    
    os.system("rostopic pub /gripper_test_both/request utilities/gripperTestRequest -1 '[0, now,base_link]' 1 3")
    
    #move to the final position
    [r_x,r_y,r_z,r_w, offset_x, offset_y] = find_gesture(limb,[[p1x_final,p1y_final],[p4x_final,p4y_final]],alpha,GripperLength*cos(radians(alpha)))
    traj.add_point(ik_position_list(limb,(p1x_final+p4x_final)/2+0.7*offset_x,(p1y_final+p4y_final)/2+0.7*offset_y,Z_PICK,r_x,r_y,r_z,r_w), 3.0)
    traj.start()
    traj.wait()
    traj.clear(limb)
    
    os.system("rostopic pub /gripper_test_both/request utilities/gripperTestRequest -1 '[0, now,base_link]' 0 3")
    
    #move away
    traj.add_point(ik_position_list(limb,(p1x_final+p4x_final)/2+0.7*offset_x,(p1y_final+p4y_final)/2+0.7*offset_y,Z_PICK-0.015,r_x,r_y,r_z,r_w), 1.0)
    traj.add_point(ik_position_list(limb,(p1x_final+p4x_final)/2+1.5*offset_x,(p1y_final+p4y_final)/2+1.5*offset_y,Z_PICK-0.015,r_x,r_y,r_z,r_w), 2.0)
    traj.add_point(ik_position_list(limb,(p1x_final+p4x_final)/2+1.5*offset_x,(p1y_final+p4y_final)/2+1.5*offset_y,Z_PICK+0.2,r_x,r_y,r_z,r_w), 3.5)
    traj.add_point(ik_position_list(limb,X_START,Y_START,Z_START,R_X,R_Y,R_Z,R_W), 6.5)
    traj.start()
    traj.wait()
    
    return 0
    

if __name__ == '__main__':
    sys.exit(main())
