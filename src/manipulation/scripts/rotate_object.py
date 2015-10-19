#!/usr/bin/env python
##to do adjust for limb right
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
from common_functions import *
from Constants import *
from pick_up_cylinder import *



def main():
    #initiate ros, robot, assign variables...
    rospy.init_node("rsdk_set_position")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    rs.enable()

    limb = "left"
    
    #test rotate bottle to original angle
    rotate_to_original(limb,"CoffeeCup")

    return 0

# find an object and its angle rotated, then move to it, rotate it to original angle
def rotate_to_original(limb,object):
    ##find object
    object_position = get_object_position(object)
    #get angle to rotate
    angle = find_object_angle(object_position)

    rotateBottle(limb,object_position,angle)


def graspTopGesture(limb):

    #suppose after Ry, z1,-Z0 allign
    Ry = get_R('y',radians(179))
    Rz = get_R('z',0)
    Rx = get_R('x',radians(10))
    R= Ry*Rz*Rx
    [rx,ry,rz,rw] = R_to_quaternion(R)
    #offset in y(gripper) direction
    offset = R*np.mat([ [0],[-0.025],[0] ]) #origin -0.03

    return [rx,ry,rz,rw,offset.item(0),offset.item(1)]

def graspTop(limb,centre):
    Z_PICK = TableZ + 0.552
    #locate the centre
    [x,y]=centre

    [rx,ry,rz,rw,offset_x,offset_y] = graspTopGesture(limb)

    position_list = [ik_position_list(limb,x+offset_x,y+offset_y,Z_PICK-0.2,rx,ry,rz,rw)]
    moveTrajectory(limb,position_list,[5])


    #down
    position_list = [ik_position_list(limb,x+offset_x,y+offset_y,Z_PICK-0.29,rx,ry,rz,rw)]
    moveTrajectory(limb,position_list,[3])

    return



# given object position, rotate a certain angle, then move up, can be used to unscrew cap
#input: object position; angle to rotate(z)
def rotateBottle(limb,object_position,angle):
    Z_PICK = TableZ + 0.552
    [x,y]=find_centre(object_position)
    graspTop(limb,[x,y])
    gripper(limb,"close")

    #suppose after Ry, z1,-Z0 allign
    Ry = get_R('y',radians(179))
    Rx = get_R('x',radians(10))


    #rotate bottle
    Rz = get_R('z', angle)
    R= Ry*Rz*Rx
    [rx,ry,rz,rw] = R_to_quaternion(R)
    #offset in y(gripper) direction
    offset = R*np.mat([ [0],[-0.03],[0] ])
    
    position_list = [ik_position_list(limb,x+offset.item(0),y+offset.item(1),Z_PICK-0.29,rx,ry,rz,rw)]
    # position_list.append(ik_position_list(limb,x+offset.item(0),y+offset.item(1),Z_PICK-0.29,rx,ry,rz,rw) )
    # position_list.append(ik_position_list(limb,x+offset.item(0),y+offset.item(1),Z_PICK-0.293,rx,ry,rz,rw) )
    moveTrajectory(limb,position_list,[3])
    rospy.sleep(0.1)
    move_keep_orientation(limb,0,0,-0.004,2)
    
    #release
    rospy.sleep(0.5)
    gripper(limb,"open")
    # os.system("rostopic pub /gripper_test_both/request utilities/gripperTestRequest -1 '[0, now,base_link]' 0 3")
    
    #move away,up
    position_list = [ik_position_list(limb,x+2*offset.item(0),y+2*offset.item(1),Z_PICK-0.294,rx,ry,rz,rw)]
    position_list.append( ik_position_list(limb,x+2*offset.item(0),y+2*offset.item(1),Z_PICK-0.294,rx,ry,rz,rw) )
    moveTrajectory(limb,position_list,[3,6])


#return the angle that the object has rotated in z axis
def find_object_angle(object_position):
    
    #detect bottle
    [ [x1,y1], [x2,y2], [x3,y3], [x4,y4] ] = object_position
    #~ print x1, y1, x2, y2, x3, y3, x4, y4 
    theta1 = atan((x2-x1)/(y1-y2))
    theta2 = atan((y2-y3)/(x2-x3))
    theta3 = atan((x3-x4)/(y4-y3))
    theta4 = atan((y1-y4)/(x1-x4))
    print degrees(theta1), degrees(theta2), degrees(theta3), degrees(theta4)
    avg_angle = getAverage([theta1,theta2,theta3,theta4])
    print "average :", degrees(avg_angle)
    rospy.logwarn("object angle: "+str(degrees(avg_angle)))


    ''' check whether it is in range -90 to 90 '''

    #check cos
    if (abs(avg_angle)<pi/4):
        if (x1+x2-x3-x4 +y1+y4-y2-y3)>0 :
            print "1"
            return avg_angle
        else:
            print "2"
            return avg_angle +pi
    #check sin
    else:
        if(avg_angle>=0):
            if (x2-x1+y2-y3+x3-x4+y1-y4) >=0:
                print "3"
                return avg_angle
            else:
                print "4"
                return avg_angle+pi
        else:
            if (x2-x1+y2-y3+x3-x4+y1-y4) <0:
                print "5"
                return avg_angle
            else:
                print "6"
                rospy.logwarn("final object angle: "+str(degrees(avg_angle)))
                return avg_angle+pi


    return avg_angle


def getAverage(values):
    total = 0
    diff_max = 0
    toRemove = 0
    
    for i in values:
        total = total + i
    
    avg1 = total/len(values)
    
    #find farest value
    for i in range(len(values)):
        if(abs(values[i]-avg1)>diff_max):
            diff_max=abs(values[i]-avg1)
            toRemove = values[i]
            
    #~ print "remove: ", toRemove,"\n"
    total = total - toRemove
            
    return total/(len(values)-1)
    
if __name__ == '__main__':
    sys.exit(main())
