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
import numpy as np
from vision.srv import *
from common_functions import *

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
X_START=0.5
Y_START=   0.0100350771099329
Z_START= 0.23229863286018372

def ik_test(limb, p_x,p_y,p_z,r_x,r_y,r_z,r_w):
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    poses = {
        'left': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=p_x,
                    y=p_y,
                    z=p_z,
                ),
                orientation=Quaternion(
                    x= r_x,
                    y=r_y,
                    z=r_z,
                    w=r_w,
                ),
            ),
        ),
        'right': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=p_x,
                    y=p_y,
                    z=p_z,
                ),
                orientation=Quaternion(
                    x= r_x,
                    y=r_y,
                    z=r_z,
                    w=r_w,
                ),
            ),
        ),
    }

    ikreq.pose_stamp.append(poses[limb])
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1

    # Check if result valid, and type of seed ultimately used to get solution
    #convert rospy's string representation of uint8[]'s to int's
    resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                               resp.result_type)
    if (resp_seeds[0] != resp.RESULT_INVALID):
        seed_str = {
                    ikreq.SEED_USER: 'User Provided Seed',
                    ikreq.SEED_CURRENT: 'Current Joint Angles',
                    ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                   }.get(resp_seeds[0], 'None')
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")
    #~ limb_joints['left_s1']=0
    #~ limb_joints['left_s0']=0
    #~ limb_joints['left_w0']=0
    #~ limb_joints['left_w1']=0
    #~ limb_joints['left_w2']=0
    #~ limb_joints['left_e0']=0
    #~ limb_joints['left_e1']=0
    print limb_joints
    return (limb_joints)
    
def move_arm(lim,position,i_max,):
    arm=baxter_interface.Limb(lim)
    i=1
    while not ((i>i_max)or(rospy.is_shutdown())):
        arm.set_joint_positions(position)
        rospy.sleep(.01)
        i=i+1
    print "moved"
    return 0

def get_quaternion(r_axis, r_angle):
    x =r_axis[0] * sin(r_angle/2)
    y =r_axis[1] * sin(r_angle/2)
    z =r_axis[2] * sin(r_angle/2)
    w =cos(r_angle/2)
    return [x,y,z,w]

def main():
    #initiate ros, robot, assign variables...
    rospy.init_node("rsdk_set_position")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    rs.enable()
    
    r_axis=[1,0,0]
    r_angle=pi/2
    [x,y,z,w]=get_quaternion(r_axis, r_angle)
    
    [px,py]=find_centre(get_object_position("Coaster"))
    print "object location: x: ",px," y: ",py
    
    #~ move_arm('left',ik_test("left",0.4605,0.1805,Z_START-0.3,x,y,z,w),4000)
    move_arm('left',ik_test("left",px,py,Z_START-0.3,x,y,z,w),800)
    move_arm('left',ik_test("left",px,py,Z_START-0.3,x,y,z,w),400)
    
    r_axis=[1,0,0]
    r_angle=pi
    [x,y,z,w]=get_quaternion(r_axis, r_angle)
    move_arm('left',ik_test("left",px,py,Z_START,x,y,z,w),800)
    move_arm('left',ik_test("left",px,py,Z_START-0.2,x,y,z,w),800)

    return 0

if __name__ == '__main__':
    sys.exit(main())
