/**
@file core.cpp
@brief Main module that interfaces vision and manipulation
@date Sept 2015

This source file contains all callback functions from other modules. 
The sequence of execution is based on a state machine.

Otherhelper function definitions in coreFunctions.cpp
class and structure definitions in core.h

*/

#include "core.h"

/** 
callback function when command received from hmi module
*/
void
coreModule::hmiCB (const hmi::hmiRequest msg) {

  if (msg.cmd == msg.CMD_GO_TO_NEUTRAL) {

    //Send go to neutral to the manipulation module.
    manipulation::manipulationRequest req;
    req.cmd = req.CMD_GO_TO_NEUTRAL;

    manipulationRequestPub.publish(req);
    
  } else if (msg.cmd == msg.CMD_ENABLE) {

    manipulation::manipulationRequest req;
    req.cmd = req.CMD_ENABLE;

    manipulationRequestPub.publish(req);

  } else if (msg.cmd == msg.CMD_DISABLE) {

    manipulation::manipulationRequest req;
    req.cmd = req.CMD_DISABLE;

    manipulationRequestPub.publish(req);

  } else if (msg.cmd == msg.CMD_INTRO) {
  	hmi::introRequest req;
  	req.enable = true;
  	introRequestPub.publish(req);
  	
  }
	
}


/**
Main program that interfaces with vision, manipulation and HMI
*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "core");
	coreModule core; // create instance of core class module
	
  ros::Rate loop_rate(10);
  ros::spin();

  return 0;
}



