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
    rospy.init_node("sonar_detect")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    rs.enable()

	pub = rospy.Publisher('gripper_test_both/request', gripperTestRequest, latch = True)
	msg = gripperTestRequest()
	pub.publish(msg)
	
    return 0

if __name__ == '__main__':
    sys.exit(main())
