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

def tray_orientation(tray_angle):
    #Give in angle of tray (in radians) and returns the quaternion orientation in format (qr,ql) in order x,y,z,w

    pi = 3.14158
    hand_orientation = 90.0*math.pi/180

    if ( tray_angle > 0) : #positive tray angle means that from baxter view, the right arm will be further away from the base than the left arm
        #LEFT
        Ry_l = np.matrix([ [cos(pi/2), 0, sin(pi/2)] , [0, 1, 0] , [-sin(pi/2), 0, cos(pi/2) ] ])
        Rz_l = np.matrix([ [1, 0, 0] , [0, cos(pi/2-tray_angle), -sin(pi/2-tray_angle)] , [0, sin(pi/2-tray_angle), cos(pi/2-tray_angle)] ])
        Rx_l = np.matrix([[cos(hand_orientation), -sin(hand_orientation), 0],[sin(hand_orientation), cos(hand_orientation), 0],[0, 0, 1]])
		#RIGHT
        Ry_r = np.matrix([[cos(pi/2), 0, sin(pi/2)],[0, 1, 0],[-sin(pi/2), 0, cos(pi/2)]])
        Rz_r = np.matrix([[1, 0, 0],[0, cos(-pi/2-tray_angle), -sin(-pi/2-tray_angle)],[0, sin(-pi/2-tray_angle), cos(-pi/2-tray_angle)]])
        Rx_r = np.matrix([[cos(-hand_orientation), -sin(-hand_orientation), 0],[sin(-hand_orientation), cos(-hand_orientation), 0],[0, 0, 1]])
    else:
        tray_angle=-tray_angle #make angle positive for calculation purposes
		#LEFT
        Ry_l = np.matrix([[cos(pi/2), 0, sin(pi/2)],[0, 1, 0],[-sin(pi/2), 0, cos(pi/2)]])
        Rz_l = np.matrix([[1, 0, 0],[0, cos(pi/2+tray_angle), -sin(pi/2+tray_angle)],[0, sin(pi/2+tray_angle), cos(pi/2+tray_angle)]])
        Rx_l = np.matrix([[cos(hand_orientation), -sin(hand_orientation), 0],[sin(hand_orientation), cos(hand_orientation), 0],[0, 0, 1]])
		#RIGHT
        Ry_r = np.matrix([[cos(pi/2), 0, sin(pi/2)],[0, 1, 0],[-sin(pi/2), 0, cos(pi/2)]])
        Rz_r = np.matrix([[1, 0, 0],[0, cos(-(pi/2-tray_angle)), -sin(-(pi/2-tray_angle))],[0, sin(-(pi/2-tray_angle)), cos(-(pi/2-tray_angle))]])
        Rx_r = np.matrix([[cos(-hand_orientation), -sin(-hand_orientation), 0],[sin(-hand_orientation), cos(-hand_orientation), 0],[0, 0, 1]])
    R_r = Ry_r*Rz_r*Rx_r #Rotation Matrix for Right end effector
    R_l = Ry_l*Rz_l*Rx_l #Rotation Matrix for Left End Effector
    
    #Convert Rotation Matrices to quaternions
    qr = quaternion_from_matrix( R_r )
    ql = quaternion_from_matrix( R_l )

    return (qr, ql)

    
def tray_to_end_effector(positions):
    #Give in position (x,y,z) and orientation (thera in degrees) of the tray. Returns pose of the two end effectors
    X_L = []
    X_R = []
    #Hard coded values for orientation ("the normal pose")
    w_rot = 0.5
    x_rot = 0.5
    y_rot = 0.5
    z_rot = 0.5 
    #tray dimensions
    tray_l = ((36.4/100)/2)+6.0/100 #tray width in metres
    tray_h = (28.3/100)/2 #tray length in metres
    
    for i in range(len(positions)):
        x = positions[i][0]
        y = positions[i][1]
        z = positions[i][2]
        theta = positions[i][3]
        phi = 3.14158*theta/180 #convert to radians       
        #if theta==0.0:
            #Faster to just hard code when we want the "normal pose"
          #  ql = [0.707, 0.0, 0.0, 0.707]
          #  qr = [0.0, 0.707, 0.707, 0.0]
        #else:
            #if there is an angle need to find out the orientation of the end effectors 
        q_both = tray_orientation(phi)
        qr = q_both[0]
        ql = q_both[1]
        print 'qr',qr
        if theta >= 0:
            X_L.append([x - tray_l*sin(phi), y + tray_l*cos(phi), z, ql[0], ql[1], ql[2], ql[3]])
            X_R.append([x + tray_l*sin(phi), y - tray_l*cos(phi), z, qr[0], qr[1], qr[2], qr[3]])
        elif theta < 0:
            phi=abs(phi)
            X_L.append([x + tray_l*sin(phi), y + tray_l*cos(phi), z, ql[0], ql[1], ql[2], ql[3]])
            X_R.append([x - tray_l*sin(phi), y - tray_l*cos(phi), z, qr[0], qr[1], qr[2], qr[3]])
        
    return (X_R,X_L)

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

def splitter(start, end):
    #Give in the the start(x1,y1,z1) and end(x2,y2,z2) and divides it up based on the step size
    x1 = start[0]
    y1 = start[1]
    z1 = start[2]
    x2 = end[0]
    y2 = end[1]
    z2 = end[2]
    
    theta1=start[3]
    theta2=end[3]
    
    
    
    dist = sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1)+(z2-z1)*(z2-z1))
    step_size = 0.01
    no_step = int(ceil(dist/step_size))+1
    path =[]
    theta_del = theta2-theta1
    if no_step<=1:
        step_size = 1.0 #degree
        no_step = int(ceil(sqrt((theta2-theta1)*(theta2-theta1))/step_size))+1
        seq = 1.0/no_step
        for i in range(0,no_step):
            path.append([x1, y1, z1, theta1+theta_del*i*seq])
    else:
        x_del = x2-x1
        y_del = y2-y1
        z_del = z2-z1


        seq = 1.0/no_step


        for i in range(0,no_step):
            path.append([x1+x_del*i*seq, y1+y_del*i*seq, z1+z_del*i*seq, theta1+theta_del*i*seq])

    return path
        
def quaternion_from_matrix(R):
    #Input a 3x3 Rot Matrix and returns the quaternion representation(x,y,z,w)
    Rxx = R.item((0,0))
    Rxy = R.item((0,1))
    Rxz = R.item((0,2))
    Ryx = R.item((1,0))    
    Ryy = R.item((1,1))
    Ryz = R.item((1,2))
    Rzx = R.item((2,0))
    Rzy = R.item((2,1))
    Rzz = R.item((2,2))

    w = sqrt(Rxx*Ryy*Rzz+1.0)/2.0

    if isinstance(w,complex):
        w = 0

    x = sqrt( 1 + Rxx - Ryy - Rzz ) / 2.0
    y = sqrt( 1 + Ryy - Rxx - Rzz ) / 2.0
    z = sqrt( 1 + Rzz - Ryy - Rxx ) / 2.0
    q = []
    q = [w,x,y,z] 
  
    if (max(q) == w):
        x = (Rzy - Ryz)/(4*w)
        y = (Rxz - Rzx)/(4*w)
        z = (Ryx - Rxy)/(4*w)
    elif (max(q) == x):
        w = (Rzy - Ryz)/(4*x)
        y = (Rxy + Ryx)/(4*x)
        z = (Rzx + Rxz)/(4*x)
    elif (max(q) == y):
        w = (Rxz - Rzx)/(4*y)
        x = (Rxy + Ryx)/(4*y)
        z = (Ryz + Rzy)/(4*y)
    elif (max(q) == z):
        w = (Ryx - Rxy)/(4*z)
        x = (Rzx + Rxz)/(4*z)      
        y = (Ryz + Rzy)/(4*z)

    q = [ x, y, z, w]

    return q        

def main():
    """RSDK Inverse Kinematics Example

    A simple example of using the Rethink Inverse Kinematics
    Service which returns the joint angles and validity for
    a requested Cartesian Pose.

    Run this example, passing the *limb* to test, and the
    example will call the Service with a sample Cartesian
    Pose, pre-defined in the example code, printing the
    response of whether a valid joint solution was found,
    and if so, the corresponding joint angles.
    """
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
    
    f = open('bimanual_testing_x_plane_rectangle', 'w')
    f.write('Test 2 - bimanual_testing_x_plane_rectangle\n')
    a=True
    
    left_pose = []
    right_pose = []
    position = []
    tray_length = (36.4 + 12)/100
    
    for i in range(30):
        right_pose.append([0.6, -tray_length/2, 0.3, -0.7071, 0.0, 0.0, 0.7071])
        left_pose.append([0.6, tray_length/2+i*0.01, 0.3,0.7071, 0.0, 0.0, 0.7071])
    os.system("rostopic pub /gripper_test_both/request utilities/gripperTestRequest -1 '[0, now,base_link]' 0 3")    
    while (a):       
       
        #print("Left Pose: \n\n")
        #position=ik_test('left')
    
        #arm=baxter_interface.Limb('left')  
        
        
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
            
            while c<len(left_pose):
                stage=c
                start=rospy.get_time()
                pick_left=left_pose[c]
                pick_right=right_pose[c]
                position=ik_test(pick_left,pick_right)
                end=rospy.get_time()
                ik_time=end-start
                
                print "ik_time: ", ik_time
                ik_time_sum=ik_time_sum+ik_time                             

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
                    os.system("rostopic pub /gripper_test_both/request utilities/gripperTestRequest -1 '[0, now,base_link]' 1 3")
                    f.write("%f\n" % rospy.get_time())
                    f.write( "left pose %s\n" % arm_l.endpoint_pose())
                    f.write( "right pose %s\n" % arm_r.endpoint_pose())
                    f.write( "left effort %s\n" % arm_l.endpoint_effort())
                    f.write( "right effort %s\n" % arm_r.endpoint_effort())
                    arm_l.set_joint_positions(position[0])
                    arm_r.set_joint_positions(position[1])
                    rospy.sleep(.1)
                    i=i+1
                    
                c=c+1
                
            av_time=ik_time_sum/c
            
            print "average time: ", av_time
    quit()

if __name__ == '__main__':
    sys.exit(main())
