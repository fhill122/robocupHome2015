#!/usr/bin/env python

import cv2
import cv2.cv as cv
import numpy as np


import argparse
import struct
import sys

import rospy

import baxter_interface

from math import sin
from math import cos
from math import ceil
from math import sqrt
import math
from baxter_interface import CHECK_VERSION

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
   
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.add_argument(
        '-l', '--limb', choices=['left', 'right'], required=True,
        help="the limb to test"
    )
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("rsdk_set_position")

    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    def clean_shutdown():
        print("\nExiting example...")
        if not init_state:
            print("Disabling robot...")
            rs.disable()
    rospy.on_shutdown(clean_shutdown)

    print("Enabling robot... ")
    rs.enable()
    
    
    a=True
    
    
    while (a):       
       
  
        
        
        #generate pose sequence for pickup
        i=0        
        c=0
        h=0
        k=0
        hello=True
        while hello==True:
            c=0
            k=0
            
            ik_time_sum=0
            
            while c<len(10):
                stage=c
               
                        

                arm_l=baxter_interface.Limb('left')
                arm_r=baxter_interface.Limb('right')
                
                i=1
                if k==0:
                    k=1
                    while not ((i>500)or(rospy.is_shutdown())):
                        arm_l.set_joint_positions(position[0])
                        arm_r.set_joint_positions(position[1])
                        rospy.sleep(.01)
                        i=i+1
                    
                while not ((i>1.0)or(rospy.is_shutdown())):
                   
                  
                    print( "left pose %s\n" % arm_l.endpoint_pose())
                    print( "right pose %s\n" % arm_r.endpoint_pose())
                    print( "left effort %s\n" % arm_l.endpoint_effort())
                    print( "right effort %s\n" % arm_r.endpoint_effort())
                    rospy.sleep(.1)
                    i=i+1
                    
                c=c+1
                
            
            
           
    quit()

if __name__ == '__main__':
    sys.exit(main())
