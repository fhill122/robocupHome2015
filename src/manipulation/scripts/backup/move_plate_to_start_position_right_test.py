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

X_GOOD = 0.518952228181
Y_GOOD = 0.346920753837
Z_GOOD =-0.0654064032341
Z_PICK = -0.0788354381932

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
    

def get_plate_position():
    rospy.wait_for_service('/vision/get_plate_position')
    try:
        #create service handler
        srv_h=rospy.ServiceProxy('/vision/get_plate_position',platePosition)
        resp = srv_h()
        return [ [resp.p1[0],resp.p1[1]], [resp.p2[0],resp.p2[1]], [resp.p3[0],resp.p3[1]],[resp.p4[0],resp.p4[1]] ]
    except rospy.ServiceException, e:
        print "service dall failed: %s"%e
        
def find_edge(position,plate_position):
    #p1p4 p2p3 are the short edge
    
    p1p4_y = plate_position[0][1] + plate_position[3][1]
    p2p3_y = plate_position[1][1] + plate_position[2][1]
    
    if p1p4_y > p2p3_y:
        left_points = [plate_position[0],plate_position[3]]
        right_points = [plate_position[2],plate_position[1]]
    else:
        left_points = [plate_position[2],plate_position[1]]
        right_points = [plate_position[0],plate_position[3]]
    
    if position == 'right':
        return right_points
    else:
        return left_points

def find_gesture( limb, edge,theta1,offset_distance):
    #theta 1 unit is angle, distance: m
    p1x=edge[0][0]
    p1y=edge[0][1]
    
    p4x=edge[1][0]
    p4y=edge[1][1]
    alpha = atan((p1y-p4y)/(p1x-p4x))
    theta = 1.*(theta1+90)/180*pi
    #~ alpha = 30./180*pi
    #~ theta = 10./180*pi + pi/2
    print "alpha(degree): %s\n"%(alpha/pi*180)
    print "theta(degree): %s\n"%(theta/pi*180)
    
    Rz = np.mat([ [cos(alpha), -sin(alpha), 0],[sin(alpha), cos(alpha), 0],[0, 0, 1] ])
    if limb == 'left':
        Rx = np.mat([ [1, 0, 0],[0 ,cos(theta), -sin(theta)],[0, sin(theta), cos(theta)] ])
        R= Rz*Rx
        print "rotation matrix: %s\n"%R
        
        w =1.*sqrt(1+R.item(0,0)+R.item(1,1)+R.item(2,2)) /2
        x =1.*(R.item(2,1)-R.item(1,2)) / (4*w)
        y =1.*(R.item(0,2)-R.item(2,0)) / (4*w)
        z =1.*(R.item(1,0)-R.item(0,1)) / (4*w)
        
        offset = Rz*np.mat([ [0],[offset_distance],[0] ])
        print "offset distance: %s\n"%R  
        return [x,y,z,w, offset.item(0), offset.item(1)]
    else:
        #right
        Rx = np.mat([ [1, 0, 0],[0 ,cos(-theta), -sin(-theta)],[0, sin(-theta), cos(-theta)] ])
        R= Rz*Rx
        print "rotation matrix: %s\n"%R
        
        w =1.*sqrt(1+R.item(0,0)+R.item(1,1)+R.item(2,2)) /2
        x =1.*(R.item(2,1)-R.item(1,2)) / (4*w)
        y =1.*(R.item(0,2)-R.item(2,0)) / (4*w)
        z =1.*(R.item(1,0)-R.item(0,1)) / (4*w)
        
        offset = Rz*np.mat([ [0],[-offset_distance],[0] ])
        print "offset distance: %s\n"%R  
        return [x,y,z,w, offset.item(0), offset.item(1)]
    
    
    
def main():

    #initiate ros, robot, assign variables...
    rospy.init_node("rsdk_set_position")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    rs.enable()
    
    plate_position=get_plate_position()
    print "plate position %s"%(plate_position)
    
    #move to start position
    os.system("rostopic pub /gripper_test_both/request utilities/gripperTestRequest -1 '[0, now,base_link]' 0 4")
    move_arm('right',ik_test("right",X_START,-Y_START,Z_START,-R_X,-R_Y,-R_Z,R_W),500,)
        
    #move to the plate 
    [[p1x,p1y],[p4x,p4y]]=find_edge('right',plate_position)
    [r_x,r_y,r_z,r_w, offset_x, offset_y] = find_gesture('right',[[p1x,p1y],[p4x,p4y]],7,0.2)
    move_arm('right',ik_test("right",(p1x+p4x)/2+offset_x,(p1y+p4y)/2+offset_y,Z_PICK+0.06,r_x,r_y,r_z,r_w),400)
    ##lower z position
    move_arm('right',ik_test("right",(p1x+p4x)/2+offset_x,(p1y+p4y)/2+offset_y,Z_PICK,r_x,r_y,r_z,r_w),100)
    #move CLOSER
    move_arm('right',ik_test("right",(p1x+p4x)/2+offset_x/4,(p1y+p4y)/2+offset_y/4,Z_PICK-0.01,r_x,r_y,r_z,r_w),100)
    os.system("rostopic pub /gripper_test_both/request utilities/gripperTestRequest -1 '[0, now,base_link]' 1 4")
    
    #move to the good position
    move_arm('right',ik_test("right",X_GOOD,Y_GOOD,Z_GOOD,R_X,R_Y,R_Z,R_W),200)
    os.system("rostopic pub /gripper_test_both/request utilities/gripperTestRequest -1 '[0, now,base_link]' 0 4")
    #move farther
    move_arm('right',ik_test("right",X_GOOD,Y_GOOD+0.08,Z_GOOD,R_X,R_Y,R_Z,R_W),200)
    #move to start position
    move_arm('right',ik_test("right",X_START,Y_START,Z_START,R_X,R_Y,R_Z,R_W),300,)
    
    return 0

if __name__ == '__main__':
    sys.exit(main())
