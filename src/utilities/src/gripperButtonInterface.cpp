/**
@file gripperButtonInterface.cpp
@brief Open and close gripper from pushing button on Baxter's arm
@date June 2014
*/

/*
Node to enable buttons on the robot's arm to interact with the gripper.
*/

#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include <baxter_core_msgs/ITBState.h>
#include "utilities/gripperTestRequest.h"
#include "std_msgs/String.h"

#define OPEN 0
#define CLOSED 1

int leftState = OPEN;
int rightState = OPEN;

ros::Publisher gripperTestPub, gripperTestPub2;

void
leftButtonCB(const baxter_core_msgs::ITBState button) {

  utilities::gripperTestRequest req;
	if (button.buttons[1] == true) {
    req.cmd = req.CMD_OPEN; 
    gripperTestPub.publish(req);
    
	} else if (button.buttons[2] == true) {
		req.cmd = req.CMD_CLOSE; 
		gripperTestPub.publish(req);
	}

  sleep(1);

}

void
rightButtonCB(const baxter_core_msgs::ITBState button) {

	utilities::gripperTestRequest req;
	if (button.buttons[1] == true) {
    req.cmd = req.CMD_OPEN; 
    gripperTestPub2.publish(req);
    
	} else if (button.buttons[2] == true) {
		req.cmd = req.CMD_CLOSE; 
		gripperTestPub2.publish(req);
	}

  sleep(1);



}

int 
main(int argc, char** argv){

  ros::init(argc, argv, "gripper_button_interface");
  ros::NodeHandle nh;

  ros::Subscriber subButtonLeft = nh.subscribe("/robot/itb/left_itb/state", 1, leftButtonCB);
  ros::Subscriber subButtonRight = nh.subscribe("/robot/itb/right_itb/state", 1, rightButtonCB);
	
	// publishers
	gripperTestPub = nh.advertise<utilities::gripperTestRequest>("gripper_test/request", 1);
  gripperTestPub2 = nh.advertise<utilities::gripperTestRequest>("gripper_test2/request", 1);

  ros::spin();

  return 0;
}
