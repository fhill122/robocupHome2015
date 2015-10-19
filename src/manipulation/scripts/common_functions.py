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
from utilities.msg import *

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

from baxter_core_msgs.msg import (
    EndpointState
)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

# todo: use loop to accept any number of points instead of 4 points.
# find the centre position of an object
# points = [ [p1x,p1y],[p2x,p2y],[p3x,p3y],[p4x,p4y] ]
# return [x,y]
def find_centre(points):
    x= 1.*(points[0][0] + points[1][0]+ points[2][0]+ points[3][0])/4
    y= 1.*(points[0][1] + points[1][1]+ points[2][1]+ points[3][1])/4
    return [x,y]
    
# get the positions of 4 rectangle corners via service calling, return [ [p1x,p1y],[p2x,p2y],[p3x,p3y],[p4x,p4y] ]
def get_object_position(object):
    print "check image\n"
    rospy.wait_for_service('/vision/get_object_position')
    try:
        #create service handler
        srv_h=rospy.ServiceProxy('/vision/get_object_position',platePosition)
        resp = srv_h(object)
        if ([ [resp.p1[0],resp.p1[1]], [resp.p2[0],resp.p2[1]], [resp.p3[0],resp.p3[1]],[resp.p4[0],resp.p4[1]] ] ==[ [0,0],[0,0],[0,0],[0,0] ]):
            return False
        return [ [resp.p1[0],resp.p1[1]], [resp.p2[0],resp.p2[1]], [resp.p3[0],resp.p3[1]],[resp.p4[0],resp.p4[1]] ]
    except rospy.ServiceException, e:
        print "service dall failed: %s"%e

# return the right or left short edge of an rectangle,
# position = "left" or "right"
# rectangle = [ [p1x,p1y],[p2x,p2y],[p3x,p3y],[p4x,p4y] ], and p1p4 p2p3 are the two short edges, this
# can be obtained by calling get_object_position
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
    
#get quaternion from rotation matrix
def R_to_quaternion(R):
    w =1.*sqrt(1+R.item(0,0)+R.item(1,1)+R.item(2,2)) /2
    x =1.*(R.item(2,1)-R.item(1,2)) / (4*w)
    y =1.*(R.item(0,2)-R.item(2,0)) / (4*w)
    z =1.*(R.item(1,0)-R.item(0,1)) / (4*w)
    return [x,y,z,w]

#get rotation matrix
# axis ='x' or 'y' or 'z'
# theta in radians
# return Rx or Ry or Rz
def get_R(axis,theta):
    if(axis =='x'):
        R=np.mat([ [1, 0, 0],[0 ,cos(theta), -sin(theta)],[0, sin(theta), cos(theta)] ])
    elif(axis =='y'):
        R=np.mat([ [cos(theta), 0, sin(theta)],[0, 1, 0],[-sin(theta), 0, cos(theta)] ])
    elif(axis =='z'):
        R=np.mat([ [cos(theta), -sin(theta), 0],[sin(theta), cos(theta), 0],[0, 0, 1] ])
        
    return R

#~ to do, simplify find_gesture function, get quaternion from rotation matrix def get_quaternion_R ()

# to do write right limb
# find the rotation and offset distance to grip an cylinder
# limb = "left" or "right", use which to pick up
# centre = [x,y]
# theta = the angle to rotate around z , rand
# alpha = the angle to rotate around y , rand, around radians(90) when pick up, radians(180) for final step of pouring, or use another function
# offset = distance to move away from the centre (unit m, scalar) , in gripper frame
# return [x,y,z,w, offset_x, offset_y] ,offset in global frame
def find_gesture_cylinder( limb, theta, alpha, offset_y_gripper, offset_z_gripper):
    #for left and right hands:
    #rotation change Rz Rx
    #offset change y 
    
    if limb == 'right':
        theta = -theta
    Rz = np.mat([ [cos(theta), -sin(theta), 0],[sin(theta), cos(theta), 0],[0, 0, 1] ])
    y_angle = alpha
    Ry = np.mat([ [cos(y_angle), 0, sin(y_angle)],[0, 1, 0],[-sin(y_angle), 0, cos(y_angle)] ])
    R= Rz*Ry
    
    w =1.*sqrt(1+R.item(0,0)+R.item(1,1)+R.item(2,2)) /2
    x =1.*(R.item(2,1)-R.item(1,2)) / (4*w)
    y =1.*(R.item(0,2)-R.item(2,0)) / (4*w)
    z =1.*(R.item(1,0)-R.item(0,1)) / (4*w)
    
    offset = R*np.mat([ [0],[-offset_y_gripper],[-offset_z_gripper] ])
    if limb == 'right':
        offset = R*np.mat([ [0],[offset_y_gripper],[-offset_z_gripper] ])
    return [x,y,z,w, offset.item(0), offset.item(1)]
        
# to do write right limb
# find the rotation and offset distance to pour an cylinder
# limb = "left" or "right", use which to pick up
# centre = [x,y]
# theta = the angle to rotate around z , rand
# alpha = the angle to rotate around y , rand, around radians(90) when pick up
# gama = the angle to rotate around z again , rand, around radians(+-90) when pour
# offset = distance to move away from the centre (unit m, scalar) , in gripper frame
# return [x,y,z,w, offset_x, offset_y] ,offset in global frame
def find_gesture_cylinder_pour( limb, theta, alpha,gamma, offset_y_gripper, offset_z_gripper):
    if limb == 'left':
        Rz = np.mat([ [cos(theta), -sin(theta), 0],[sin(theta), cos(theta), 0],[0, 0, 1] ])
        y_angle = alpha
        Ry = np.mat([ [cos(y_angle), 0, sin(y_angle)],[0, 1, 0],[-sin(y_angle), 0, cos(y_angle)] ])
        Rz2 = np.mat([ [cos(gamma), -sin(gamma), 0],[sin(gamma), cos(gamma), 0],[0, 0, 1] ])
        R= Rz*Ry*Rz2
        
        w =1.*sqrt(1+R.item(0,0)+R.item(1,1)+R.item(2,2)) /2
        x =1.*(R.item(2,1)-R.item(1,2)) / (4*w)
        y =1.*(R.item(0,2)-R.item(2,0)) / (4*w)
        z =1.*(R.item(1,0)-R.item(0,1)) / (4*w)
        
        offset = R*np.mat([ [0],[-offset_y_gripper],[-offset_z_gripper] ])
        return [x,y,z,w, offset.item(0), offset.item(1)]
    

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
        print "offset distance: %s\n"%offset  
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


def get_current_pose(limb):
    msg = rospy.wait_for_message("robot/limb/"+limb+"/endpoint_state", EndpointState)
    currentPose = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,\
        msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w]
    return currentPose


# move while keep the endpoint orientation
def move_keep_orientation(limb,delt_x,delt_y,delt_z,time):
    [x,y,z,rx,ry,rz,rw] = get_current_pose(limb)
    moveTrajectory(limb,[ik_position_list(limb, x+delt_x, y+delt_y,z+delt_z,rx,ry,rz,rw)],[time])
    
    return
    

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
        rospy.logwarn("INVALID POSE - No Valid Joint Solution Found.")

    return (limb_joints)
    
    
# return a list of joint position instead, used for trajectory
def ik_position_list(limb, p_x,p_y,p_z,r_x,r_y,r_z,r_w):
    print "trying to move to",p_x,p_y,p_z
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
        rospy.logwarn("INVALID POSE - No Valid Joint Solution Found.\n")
        return 2
    print("IK solution Found.\n")
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

def moveTrajectory(limb, jointPosition_list, duration):
    traj = Trajectory(limb)     
    for i in range(len(jointPosition_list)):
        traj.add_point(jointPosition_list[i], duration[i])
    traj.start()
    #the total time it will wait, then exit no matter finish or no
    traj.wait(duration[i]+5)
    traj.clear(limb)

#useg: gripper('both','open')
def gripper(limb,operation,sleepTime = 1):
    
    pub = rospy.Publisher('gripper_test_both/request', gripperTestRequest, latch = True)
    msg = gripperTestRequest()
    
    if limb == 'left':
        limb = 3
    elif limb == 'right':
        limb=4
    elif limb == 'both':
        limb=5
    else:
        rospy.logerr( "gripper limb input error")
        return
    
    if operation == 'open':
        cmd = 0
    elif operation == 'close':
        cmd=1
    else:
        rospy.logerr("gripper command input error")
        return
        
    msg.limb = limb
    msg.cmd = cmd
    
        
    pub.publish(msg)
    rospy.sleep(sleepTime)

#finds whether or not a point is in a polygon    
def point_in_poly(x,y,poly):
#polygons are defined with vertexes in a clockwise direction ie. poly=[(0,0), (0,1), (1,1), (1,0)]
    n = len(poly)
    inside = False

    p1x,p1y = poly[0]
    for i in range(n+1):
        p2x,p2y = poly[i % n]
        if y > min(p1y,p2y):
            if y <= max(p1y,p2y):
                if x <= max(p1x,p2x):
                    if p1y != p2y:
                        xints = (y-p1y)*(p2x-p1x)/(p2y-p1y)+p1x
                    if p1x == p2x or x <= xints:
                        inside = not inside
        p1x,p1y = p2x,p2y


    return inside

def ik_test(pick_left,pick_right):
    
    
    
    #rospy.init_node("rsdk_ik_service_client")
    ns = "ExternalTools/" + "left" + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    poses = {
        'left': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                
                    x=pick_left[0],
                    y=pick_left[1],
                    z=pick_left[2],
                ),
                orientation=Quaternion(
                    x=pick_left[3],
                    y=pick_left[4],
                    z=pick_left[5],
                    w=pick_left[6],
                ),
            ),
        ),
        'right': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=.5,
                    y=-.7,
                    z=.4,
                ),
                orientation=Quaternion(
                    x=1,
                    y=0,
                    z=0,
                    w=0,
                ),
            ),
        ),
    }
    
    ikreq.pose_stamp.append(poses["left"])
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1

    # Check if result valid, and type of seed ultimately used to get solution
    # convert rospy's string representation of uint8[]'s to int's
    resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                               resp.result_type)
    if (resp_seeds[0] != resp.RESULT_INVALID):
        seed_str = {
                    ikreq.SEED_USER: 'User Provided Seed',
                    ikreq.SEED_CURRENT: 'Current Joint Angles',
                    ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                   }.get(resp_seeds[0], 'None')
        print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
              (seed_str,))
        # Format solution into Limb API-compatible dictionary
        limb_joints_l = dict(zip(resp.joints[0].name, resp.joints[0].position))
        
        
        
        
        
        print "\nIK Joint Solution:\n", limb_joints_l
        print "------------------"
        print "Response Message:\n", resp
        
        
        
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")
        
    ns = "ExternalTools/" + "right" + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    poses = {
        'left': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=.4,
                    y=.4,
                    z=0.2,
                ),
                orientation=Quaternion(
                    x=1,
                    y=0,
                    z=0,
                    w=0,
                ),
            ),
        ),
        'right': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                
                    x=pick_right[0],
                    y=pick_right[1],
                    z=pick_right[2],
                ),
                orientation=Quaternion(
                    x=pick_right[3],
                    y=pick_right[4],
                    z=pick_right[5],
                    w=pick_right[6],
                ),
            ),
        ),
    }

    ikreq.pose_stamp.append(poses["right"])
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1

    # Check if result valid, and type of seed ultimately used to get solution
    # convert rospy's string representation of uint8[]'s to int's
    resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                               resp.result_type)
    if (resp_seeds[0] != resp.RESULT_INVALID):
        seed_str = {
                    ikreq.SEED_USER: 'User Provided Seed',
                    ikreq.SEED_CURRENT: 'Current Joint Angles',
                    ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                   }.get(resp_seeds[0], 'None')
        print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
              (seed_str,))
        # Format solution into Limb API-compatible dictionary
        limb_joints_r = dict(zip(resp.joints[0].name, resp.joints[0].position))
    
        
        print "\nIK Joint Solution:\n", limb_joints_r
        print "------------------"
        print "Response Message:\n", resp
        
        
        
        
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")
        
    return (limb_joints_l,limb_joints_r)

    return 0    

def splitter_single(start, end):
    #Give in the the start(x1,y1,z1, xr1,yr1,zr1, wr1) and end(x2,y2,z2,xr2,yr2,zr2, wr2) and divides it up based on the step size
    x1 = start[0]
    y1 = start[1]
    z1 = start[2]
    xr1 = start[3]
    yr1 = start[4]
    zr1 = start[5]
    wr1 = start[6]
    x2 = end[0]
    y2 = end[1]
    z2 = end[2]

    dist = sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1)+(z2-z1)*(z2-z1))
    step_size = 0.005
    no_step = int(ceil(dist/step_size))+1
    path =[]
    x_del = x2-x1
    y_del = y2-y1
    z_del = z2-z1
    seq = 1.0/no_step


    for i in range(0,no_step):
        path.append([x1+x_del*i*seq, y1+y_del*i*seq, z1+z_del*i*seq, xr1, yr1, zr1, wr1])
    path.append(end)
    return path
