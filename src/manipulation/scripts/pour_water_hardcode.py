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
from rotate_object import *

from utilities.msg import *

Z_START=0.4
Z_PICK = TableZ + 0.075

def main():
    #initiate ros, robot, assign variables...
    rospy.init_node("pour_water")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    rs.enable()
    
    gripper('both','open')
    
    limb = "left"
    ##move coffee cup
    #for left
    X_START=0.4
    Y_START=0.63
    Z_START=0.4
    R_X=0.603489643517
    R_Y=0.634442665138
    R_Z=-0.335770984639-0.67066286733
    R_W=0.347189574577
    if limb == "right":
        Y_START= - Y_START
        R_X=-R_X
        R_Z=-R_Z
    print X_START,Y_START,Z_START, R_X, R_Y, R_Z 
    #~ moveTrajectory(limb,[ik_position_list(limb,X_START,Y_START,Z_START,R_X,R_Y,R_Z,R_W)], [6])
    #grip
    endPosition = find_grip_cylinder(limb,"CoffeeCup")
    moveTrajectory(limb,[ik_position_list(limb,0.501479366625,0.143842483985,0.0082061447491,0.53110348361,0.52654829117,-0.418296260338,0.515465057352)], [6])
    moveTrajectory(limb,[ik_position_list(limb,0.501479366625,0.143842483985,-0.0652061447491,0.53110348361,0.52654829117,-0.418296260338,0.515465057352)], [2])
    
    #release
    gripper(limb,'open')
    moveTrajectory(limb,[ik_position_list(limb,0.461479366625,0.243842483985,-0.0652061447491,0.53110348361,0.52654829117,-0.418296260338,0.515465057352)], [2])
    moveTrajectory(limb,[ik_position_list(limb,0.461479366625,0.243842483985,0.0392061447491,0.53110348361,0.52654829117,-0.418296260338,0.515465057352)], [2])
    
    #rotate
    rotate_to_original(limb,"CoffeeCup")

    #grip
    moveTrajectory(limb,[ik_position_list(limb,0.461051938177,0.336637498465, 0.157353047402,0.542076050473,0.490276403055,-0.484119467499,0.481051915389)], [5])
    moveTrajectory(limb,[ik_position_list(limb,0.513129100527,0.182330035623,-0.0593816840898,0.53911306625,0.53482609242,-0.507202099158,0.407509733964)], [6])
    moveTrajectory(limb,[ik_position_list(limb,0.526129100527,0.160330035623,-0.0593816840898,0.53911306625,0.53482609242,-0.507202099158,0.407509733964)], [2])
    gripper(limb,'close')
    #up
    moveTrajectory(limb,[ik_position_list(limb,0.526129100527,0.142330035623, 0.0813816840898,0.53911306625,0.53482609242,-0.507202099158,0.407509733964)], [2])
    
    ##right arm
    moveTrajectory('right',[ik_position_list('right',0.57128668875,-0.273064797288, -0.0271532247459,-0.277201633439,0.732120900759,0.264310846073,0.563292124692)], [6])
    moveTrajectory('right',[ik_position_list('right',0.643512580099,-0.284414558727, -0.0854377842576,-0.313298187611,0.684702156086,0.262606674398,0.603377939315)], [3])
    gripper('right','close')
    moveTrajectory('right',[ik_position_list('right',0.649274286384,-0.114420979527, 0.0556079809065,-0.601030426405,0.447287450997,0.363027540124,0.553992209176)], [6])
    
    ##pour
    moveTrajectory('left',[ik_position_list('left',0.513975738964,0.121790468159, 0.110592316607,-0.383043274145,-0.597082624267,0.638377581709,-0.298737766446)], [2])
    moveTrajectory('left',[ik_position_list('left',0.543975738964,0.121790468159, 0.110592316607,-0.383043274145,-0.597082624267,0.638377581709,-0.298737766446)], [2])
    #~ moveTrajectory('left',[ik_position_list('left',0.5413092634,0.104715207057, 0.13043238814,-0.28143884878,-0.642473630115,0.688838090586,-0.183089852156)], [2])
    moveTrajectory('left',[ik_position_list('left',0.569404851914,0.110451328135, 0.175854846917,-0.00833304041973,-0.69074518654,0.719911802142,0.0672952067089)], [4])
    #pause
    moveTrajectory('left',[ik_position_list('left',0.569404851914,0.110451328135, 0.175854846917,-0.00833304041973,-0.69074518654,0.719911802142,0.0672952067089)], [10])
    moveTrajectory('left',[ik_position_list('left',0.510906641503,0.134156407589, 0.179486134566,0.568424137363,0.504650233009,-0.494327516422,0.421737416997)], [3])

    
    traj_left = Trajectory("left")
    traj_right = Trajectory("right")
    traj_left.add_point(ik_position_list("left",0.695833814153,0.377953578602,-0.0839780496639,0.357330727941,0.644491915913,-0.378578353503,0.560020849121), 7.0)
    traj_right.add_point(ik_position_list("right",0.563311060052,-0.116402212415,-0.0831584221165,0.564970941747,-0.47776060133,-0.53066049692,-0.413463516901), 7.0)
    traj_left.start()
    traj_right.start()
    traj_left.wait(20)
    traj_right.wait(20)
    traj_right.clear("right")
    traj_left.clear("left")
    
    gripper('both','open')
    
    return 0
    
if __name__ == '__main__':
    sys.exit(main())
