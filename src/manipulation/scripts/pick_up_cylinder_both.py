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

from pick_up_cylinder import *

    


def main():
      
    #initiate ros, robot, assign variables...
    rospy.init_node("pickup_cylinder")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    rs.enable()

    limb = "right"
    
    ##start position
    #for left
    X_START=0.4
    Y_START=0.63
    Z_START=0.4
    R_X=0.603489643517
    R_Y=0.634442665138
    R_Z=-0.335770984639
    R_W=0.347189574577
    if limb == "right":
        Y_START= - Y_START
        R_X=-R_X
        R_Z=-R_Z
    print X_START,Y_START,Z_START, R_X, R_Y, R_Z 
    #~ moveTrajectory(limb,[ik_position_list(limb,X_START,Y_START,Z_START,R_X,R_Y,R_Z,R_W)], [6])
    
    ##grip
    find_grip_cylinder(limb,"WaterBottle")
    move_keep_orientation(limb,0,0,0.2,2)
    
    #release
    gripper(limb,'open')

    return 0
    

if __name__ == '__main__':
    sys.exit(main())
