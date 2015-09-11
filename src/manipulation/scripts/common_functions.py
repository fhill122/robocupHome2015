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
from copy import copy
import actionlib

from vision.srv import *

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

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

# get the positions of 4 rectangle corners via service calling, return [ [p1x,p1y],[p2x,p2y],[p3x,p3y],[p4x,p4y] ]
def get_plate_position():
    rospy.wait_for_service('/vision/get_plate_position')
    try:
        #create service handler
        srv_h=rospy.ServiceProxy('/vision/get_plate_position',platePosition)
        resp = srv_h()
        return [ [resp.p1[0],resp.p1[1]], [resp.p2[0],resp.p2[1]], [resp.p3[0],resp.p3[1]],[resp.p4[0],resp.p4[1]] ]
    except rospy.ServiceException, e:
        print "service dall failed: %s"%e

# return the right or left short edge of an rectangle,
# position = "left" or "right"
# rectangle = [ [p1x,p1y],[p2x,p2y],[p3x,p3y],[p4x,p4y] ], and p1p4 p2p3 are the two short edges, this
# can be obtained by calling get_plate_position
def find_edge(position,rectangle):
    
    p1p4_y = rectangle[0][1] + rectangle[3][1]
    p2p3_y = rectangle[1][1] + rectangle[2][1]
    
    if p1p4_y > p2p3_y:
        left_points = [rectangle[0],rectangle[3]]
        right_points = [rectangle[2],rectangle[1]]
    else:
        left_points = [rectangle[2],rectangle[1]]
        right_points = [rectangle[0],rectangle[3]]
    
    if position == 'right':
        return right_points #return [p3,p2] or [p1,p4]
    else:
        return left_points #return [p1,p4] or [p3,p2]


# get quaternion from rotation angle and axis,
# r_axis = [x,y,z] ,  r_angle is in radius
def get_quaternion(r_axis, r_angle):
    x =r_axis[0] * sin(r_angle/2)
    y =r_axis[1] * sin(r_angle/2)
    z =r_axis[2] * sin(r_angle/2)
    w =cos(r_angle/2)
    return [x,y,z,w]

#~ to do, simplify find_gesture function, get quaternion from rotation matrix def get_quaternion_R ()

# find the rotation and offset distance to grip an edge
# limb = "left" or "right", use which to pick up
# edge = [p1,p4] or [p3,p2], can be found by calling find_edge
# theta1 = the angle (degree) between the arm and horizontal surface (around 10 for plate, can be 0 for other object ) 
# offset_distance = distance to move away from the objectin (unit m, scalar) 
# return [x,y,z,w, offset_x, offset_y]
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
    
    #rotation matrix aruond z
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

# move single arm, lim = "left" or "right"
def move_arm(lim,position,i_max,):
    arm=baxter_interface.Limb(lim)
    i=1
    while not ((i>i_max)or(rospy.is_shutdown())):
        arm.set_joint_positions(position)
        rospy.sleep(.01)
        i=i+1
    print "moved"
    return 0

# move two arms simutanuously
def move_both_arms(position_l,position_r,i_max,):
    arm_l=baxter_interface.Limb('left')
    arm_r=baxter_interface.Limb('right')
    i=1
    while not ((i>i_max)or(rospy.is_shutdown())):
        arm_r.set_joint_positions(position_r)
        arm_l.set_joint_positions(position_l)
        rospy.sleep(.01)
        i=i+1
    print "moved"
    return 0

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
        print("*******************\n")
        print(resp.joints[0].position)
        print("*******************\n")
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")

    return (limb_joints)
    
    
# return a list of joint position instead, used for trajectory
def ik_position_list(limb, p_x,p_y,p_z,r_x,r_y,r_z,r_w):
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
        limb_joints = resp.joints[0].position
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")

    return limb_joints

# adapted from /baxter_examples/scripts/joint_trajectory_client
class Trajectory(object):
    def __init__(self, limb):
        ns = 'robot/limb/' + limb + '/'
        self._client = actionlib.SimpleActionClient(
            ns + "follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        self._goal = FollowJointTrajectoryGoal()
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear(limb)

    def add_point(self, positions, time):
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time.now()
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def clear(self, limb):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.trajectory.joint_names = [limb + '_' + joint for joint in \
            ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']]
    


