/**
@file core.h
@brief Contains definitions of class, structures and constants used in core
@date June 2014

Main class used in this code is coreModule.
*/

#include "ros/ros.h"
#include "vision/visionRequest.h"
#include "vision/featureDetectionStatus.h"
#include "vision/objectSegmentationStatus.h"
#include "vision/objectClassificationStatus.h"
#include "manipulation/manipulationRequest.h"
#include "manipulation/manipulationStatus.h"
#include "hmi/hmiRequest.h"
#include "hmi/introRequest.h"
#include <std_msgs/String.h>
#include "vision/colourHistogramRequest.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <cstdlib>
#include <stdlib.h>
#include <signal.h>
#include <math.h>
#include "utilities/tfRequest.h"
#include "utilities/gripperTestRequest.h"
#include "utilities/gripperTestStatus.h"

#include "Eigen/Core"
#include <sensor_msgs/PointCloud2.h>

#define DEBUG_MODE true // flag to display debugging messages

#define ENDPOINTQX_RIGHT 0.432291189296
#define ENDPOINTQY_RIGHT 0.540187058723
#define ENDPOINTQZ_RIGHT -0.402050426973
#define ENDPOINTQW_RIGHT 0.599731376047

using namespace std;


// utility functions:
bool withinBounds(geometry_msgs::Point center, geometry_msgs::Point min, geometry_msgs::Point max);
geometry_msgs::Quaternion rpyToQuaternion(Eigen::Matrix3d R);
void mySigintHandler(int sig);

// data structure for objects and planes
struct Object{
  vector<string> detected_obj;
  vector<double> probability;
  vector<double> updatedProb;
  geometry_msgs::Point center;
  geometry_msgs::Point dimensions; // currently unused
  vector<geometry_msgs::Point> corners;
  sensor_msgs::PointCloud2 ROScloud;
};
struct Plane{
	vector<Object> objects;
};

// current command (STATE) of program
enum command_t {
  INITIAL, // idle state
  OBJECT_SEGMENT_AND_REQUEST, // segments if its the first time, and then points
  OBJECT_REQUEST, // points to object
  OBJECT_SEGMENTATION, // object segmentation only - does not point to any object
  OBJECT_GRIP,
  OBJECT_PLACE_STEP1, // move a certain z offset above object
  OBJECT_PLACE_STEP2, // place object at desired location
  OBJECT_RELEASE,
  SEGMENT_CLEAN, // segments and then clean objects on plane without identifying
  CLEAN // clean objects on plane without identifying
};

class coreModule {

		ros::NodeHandle n;
		// publishers for vision and manipulation request
		ros::Publisher visionRequestPub;
		ros::Publisher manipulationRequestPub;
		ros::Publisher introRequestPub;
		ros::Publisher gripperTestPub; //TEST
		ros::Publisher gripperTestPub2; //TEST
		// service clients
		ros::ServiceClient colourHistogram_srv;
		ros::ServiceClient tf_srv;
		// subscribers
		ros::Subscriber hmiSub;
		ros::Subscriber featureDetectionStatusSub;
		ros::Subscriber objectSegmentationStatusSub;
		ros::Subscriber objectClassificationStatusSub;
		ros::Subscriber manipulationStatusSub;
		ros::Subscriber gripperTestStatusSub; //TEST
		ros::Subscriber gripperTestStatusSub2; //TEST
		
		// variable for program states:
		string objectRequested;
		command_t command;
		bool segmentationPerformed;
		geometry_msgs::Pose pose; // current pose of EE
		string limb; // current limb in use
  	double theta; // angle from fixed joint of arm (rad)
  	
  	// variables for CLEAN task
  	geometry_msgs::Point clean_min, clean_max;
  	bool clean_mode; //to continue to pick up multiple objects
		
		//CONSTANTS
		geometry_msgs::Pose END_POSE_LEFT;
  	geometry_msgs::Pose END_POSE_RIGHT; 
  	
  	// structure for objects detected
  	vector<Plane> planes; // data struct of all planes and objects (only cleared when object segmentation called explicitly)
  	int maxObjPlaneIndex, maxObjIndex;
  	vector<Plane> planesToClean; // data struct of all plane and objects remaining to be cleaned (for CLEAN command)
		
	public:
		coreModule(); // constructor
    // from HMI module
		void hmiCB (const hmi::hmiRequest msg);
		// from vision module
		void featureDetectionStatusCB (vision::featureDetectionStatus msg);
		void objectSegmentationStatusCB(vision::objectSegmentationStatus msg);
		void objectClassificationStatusCB(vision::objectClassificationStatus status);
		// from manipulation module
		void manipulationStatusCB(manipulation::manipulationStatus status);	
		//TEST
		void gripperTestStatusCB(utilities::gripperTestStatus status);
		void gripperTestStatusCB2(utilities::gripperTestStatus status);
		
		// functions:
		void moveToDropoff();	
		void gripObject();
		void unGripObject();		
};




