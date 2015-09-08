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
from ManipulationConstants import *

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


##constant
X_START=0.621888692162
Y_START=0.46333973238
Z_START=0.193484766029
R_X=0.832475437793
R_Y=-0.00871415462847
R_Z=-0.0230735969917
R_W=0.553512708167

#~ X_GOOD = 0.518952228181
#~ Y_GOOD = 0.346920753837
#~ Z_GOOD =-0.0654064032341+0.2
Z_PICK = -0.06


p1x_final = 0.6491340398788452
p1y_final = 0.18462590873241425
p4x_final = 0.3690818250179291
p4y_final = 0.18592007458209991
    
def main():

    #degree
    alpha = 45
    Z_PICK = TableZ + sin(radians(alpha)) * GripperLength +0.015
    
    #initiate ros, robot, assign variables...
    rospy.init_node("rsdk_set_position")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    rs.enable()
    #rospy.sleep(1000)
    
    plate_position=get_plate_position()
    print "plate position %s"%(plate_position)
    
    #move to start position
    #~ os.system("rostopic pub /gripper_test_both/request utilities/gripperTestRequest -1 '[0, now,base_link]' 0 3")
    move_arm('left',ik_test("left",X_START,Y_START,Z_START,R_X,R_Y,R_Z,R_W),500,)
    
    #move to the plate 
    [[p1x,p1y],[p4x,p4y]]=find_edge('left',plate_position)
    [r_x,r_y,r_z,r_w, offset_x, offset_y] = find_gesture('left',[[p1x,p1y],[p4x,p4y]],alpha, GripperLength*cos(radians(alpha)) )
    ##check if position if right
    move_arm('left',ik_test("left",(p1x+p4x)/2+offset_x,(p1y+p4y)/2+offset_y,Z_PICK+0.06,r_x,r_y,r_z,r_w),400)
    #~ rospy.sleep(10);
    
    # grip the plate
    ## move away offset position 
    ### this is the most difficult position, if no solution found, should use the other arm
    move_arm('left',ik_test("left",(p1x+p4x)/2+1.2*offset_x, (p1y+p4y)/2+1.3*offset_y, Z_PICK+0.06,r_x,r_y,r_z,r_w),100)
    ##lower z position
    move_arm('left',ik_test("left",(p1x+p4x)/2+1.2*offset_x, (p1y+p4y)/2+1.3*offset_y, Z_PICK,r_x,r_y,r_z,r_w),100)
    #move CLOSER
    move_arm('left',ik_test("left",(p1x+p4x)/2+0.9*offset_x, (p1y+p4y)/2+0.9*offset_y, Z_PICK,r_x,r_y,r_z,r_w),50)
    move_arm('left',ik_test("left",(p1x+p4x)/2+0.7*offset_x, (p1y+p4y)/2+0.7*offset_y, Z_PICK,r_x,r_y,r_z,r_w),50)
    os.system("rostopic pub /gripper_test_both/request utilities/gripperTestRequest -1 '[0, now,base_link]' 1 3")
    
    #move to the final position
    [r_x,r_y,r_z,r_w, offset_x, offset_y] = find_gesture('left',[[p1x_final,p1y_final],[p4x_final,p4y_final]],alpha,GripperLength*cos(radians(alpha)))
    move_arm('left',ik_test("left",(p1x_final+p4x_final)/2+0.7*offset_x, (p1y_final+p4y_final)/2+0.7*offset_y, Z_PICK,r_x,r_y,r_z,r_w),300)
    os.system("rostopic pub /gripper_test_both/request utilities/gripperTestRequest -1 '[0, now,base_link]' 0 3")
    #move away
    move_arm('left',ik_test("left",(p1x_final+p4x_final)/2+0.8*offset_x, (p1y_final+p4y_final)/2+0.8*offset_y, Z_PICK-0.015,r_x,r_y,r_z,r_w),20)
    move_arm('left',ik_test("left",(p1x_final+p4x_final)/2+1.3*offset_x, (p1y_final+p4y_final)/2+1.3*offset_y, Z_PICK-0.015,r_x,r_y,r_z,r_w),50)
    #move to start position
    move_arm('left',ik_test("left",X_START,Y_START,Z_START,R_X,R_Y,R_Z,R_W),300,)
    
    return 0
    

if __name__ == '__main__':
    sys.exit(main())
