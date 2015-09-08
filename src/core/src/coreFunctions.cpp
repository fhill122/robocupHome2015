/**
@file coreFunctions.cpp
@brief Containes function definitions of helper functions used in core
@date Sept 2015

Contains function definitions for all functions except call back functions, which are defined in core.cpp
*/

#include "core.h"

// constructor
coreModule::coreModule() {

	// PUBLISHERS:
	// publish object detection request to vision module
  visionRequestPub = n.advertise<vision::visionRequest>("core/vision_request", 1);
  // publish manipulation request to manipulation module
  manipulationRequestPub = n.advertise<manipulation::manipulationRequest>("core/manipulation_request", 1);
  // grippers
  gripperTestPub = n.advertise<utilities::gripperTestRequest>("gripper_test/request", 1); //TEST
  gripperTestPub2 = n.advertise<utilities::gripperTestRequest>("gripper_test2/request", 1); //TEST
  // introduction
  introRequestPub = n.advertise<hmi::introRequest>("hmi/intro/request", 1);

	// SUBSCRIBERS:
  // subscribe to hmi module to receive commands
  hmiSub = n.subscribe("hmi/request", 1, &coreModule::hmiCB, this);
  // subscribe to vision module
  featureDetectionStatusSub = n.subscribe("vision_module/feature_detection_status", 1, &coreModule::featureDetectionStatusCB, this);
  objectSegmentationStatusSub = n.subscribe("vision_module/object_segmentation_status", 1, &coreModule::objectSegmentationStatusCB, this); 
  objectClassificationStatusSub = n.subscribe("vision_module/object_classification_status", 1, &coreModule::objectClassificationStatusCB, this);
  // subscribe to manipulation module status which notifies when desired position reached
  manipulationStatusSub = n.subscribe("manipulation_module/status", 1, &coreModule::manipulationStatusCB, this);
  gripperTestStatusSub = n.subscribe("gripper_test/status", 1, 
    &coreModule::gripperTestStatusCB, this);
  gripperTestStatusSub2 = n.subscribe("gripper_test2/status", 1, 
    &coreModule::gripperTestStatusCB2, this);
  
  // service to call colour histogram comparison server
  colourHistogram_srv = n.serviceClient<vision::colourHistogramRequest>("colour_histogram_comparison/compare");
  // service call to get transformation matrix
  tf_srv = n.serviceClient<utilities::tfRequest>("tf_server");


  // Override the default ros sigint handler.
  // This must be set after the first NodeHandle is created.
  // - To delete the camera_used parameter when demo exits
  signal(SIGINT, mySigintHandler);
  
  // SET default values
  initEndPose();
  command = INITIAL;

}



// grip object
void
coreModule::gripObject() {
  //TODO: grip object here
  cerr<<"GRIP"<<endl;
  
  // TEST code
  utilities::gripperTestRequest req;
  req.cmd = req.CMD_CLOSE;
  req.limb = req.BOTH;
  gripperTestPub.publish(req);
  
}
// ungrip object
void
coreModule::unGripObject() {
  //TODO: grip object here
  cerr<<"RELEASE"<<endl;
  
  // TEST code
  utilities::gripperTestRequest req;
  req.cmd = req.CMD_OPEN;
  req.limb = req.BOTH;
  gripperTestPub.publish(req);
  
}

// manipulate to drop off location
void
coreModule::moveToDropoff() {


}

// *** Helper functions:
//function that runs when ctrl+c pressed or node is closed
void 
mySigintHandler(int sig)
{
  // Do some custom action.
  // For example, publish a stop message to some other nodes.
  //delete parameter value
  ros::param::del("/camera_used");
  
  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}

