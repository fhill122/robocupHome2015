#!/usr/bin/env python
import numpy as np
import argparse
import struct
import sys
import rospy
from math import *
from baxter_interface import CHECK_VERSION
import os


from sensor_msgs.msg import PointCloud


from std_msgs.msg import Header

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
def main():
    rospy.init_node("sonar_detect")

    #publish
	#pub = rospy.Publisher('gripper_test_both/request', gripperTestRequest, latch = True)
	#msg = gripperTestRequest()
	#pub.publish(msg)
    
    rospy.Subscriber("/robot/sonar/head_sonar/state", PointCloud, sonarCb)
    
    rospy.spin()
    return 0

def sonarCb(data):
    print len(data.points)

if __name__ == '__main__':
    sys.exit(main())
