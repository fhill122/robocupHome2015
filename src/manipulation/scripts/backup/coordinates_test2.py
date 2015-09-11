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
                    x=0.656982770038,
                    y=-0.852598021641,
                    z=0.0388609422173,
                ),
                orientation=Quaternion(
                    x=0.367048116303,
                    y=0.885911751787,
                    z=-0.108908281936,
                    w=0.261868353356,
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
    
def main():
    #initiate ros, robot, assign variables...
    rospy.init_node("rsdk_set_position")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    rs.enable()
    
    plate_position=get_plate_position()
    print "plate position %s"%(plate_position)
    
    #move to start position
    #os.system("rostopic pub /gripper_test/request utilities/gripperTestRequest -1 '[0, now,base_link]' 0 3")
    #move_arm('left',ik_test("left",X_START,Y_START,Z_START),500,)
        
    #move to the plate 
    #find the left edge
    [[p1x,p1y],[p4x,p4y]]=find_edge('left',plate_position)
    
    #~ #construct unit vector n
    #~ theta=20.0/180*pi
    #~ x=1
    #~ y=(p1x-p4x)/(p4y-p1y)
    #~ z=-sin(theta)*sqrt(1.+ pow( (p1x-p4x)/(p4y-p1y) ,2))
    #~ n = np.array([x,y,z])
    #~ mag= sqrt(np.dot(n,n))
    #~ n = [n[0]/mag, n[1]/mag, n[2]/mag]
    #~ print n
    #~ 
    #~ #find rotation axis, angle
    #~ Z_0 =np.array([0.,0.,1.]) #z inertial axis
    #~ #rotation axis
    #~ r_axis = np.cross(n,Z_0)
    #~ mag= sqrt(np.dot(r_axis,r_axis))
    #~ r_axis = [r_axis[0]/mag, r_axis[1]/mag, r_axis[2]/mag]
    #~ print "angle %s\n"%(r_axis)
    #~ r_angle = acos(np.dot(Z_0,n))
    #~ print "angle %s\n"%(r_angle)

    alpha = atan((p1y-p4y)/(p1x-p4x))
    theta = 1.*(15+90)/180*pi
    #~ alpha = 30./180*pi
    #~ theta = 10./180*pi + pi/2
    print "alpha(degree): %s\n"%(alpha/pi*180)
    print "theta(degree): %s\n"%(theta/pi*180)
    Rz = np.mat([ [cos(alpha), -sin(alpha), 0],[sin(alpha), cos(alpha), 0],[0, 0, 1] ])
    Rx = np.mat([ [1, 0, 0],[0 ,cos(theta), -sin(theta)],[0, sin(theta), cos(theta)] ])
    R= Rz*Rx
    print R
    
    w =1.*sqrt(1+R.item(0,0)+R.item(1,1)+R.item(2,2)) /2
    x =1.*(R.item(2,1)-R.item(1,2)) / (4*w)
    y =1.*(R.item(0,2)-R.item(2,0)) / (4*w)
    z =1.*(R.item(1,0)-R.item(0,1)) / (4*w)
    
    print "x %s\ny %s\nz %s\nw %s\n"%(x,y,z,w)
    move_arm('left',ik_test("left",X_START,Y_START,Z_START+0.2,x,y,z,w),800)
    #~ move_arm('left',ik_test("left",(p1x+p4x)/2,(p1y+p4y)/2+0.15,Z_PICK+0.2),800)
    

    return 0

if __name__ == '__main__':
    sys.exit(main())
