/**
 * @file object_detector.cpp
 * @author Lingfeng Bian
 * @brief subscribe image from image topic, display it on request (service)
 */
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include <iostream>
#include <stdio.h>

#include "vision/platePosition.h"


using namespace std;
using namespace cv;

/** Global variables */
string window_name = "Good Matches & Object detection";

#define DATA_FOLDER "/home/ivan/catkin_ws/src/vision/data/"

Mat frame;
/** Function Headers */

/**
 * @function main
 */
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	frame=cv_bridge::toCvCopy(msg, "bgr8")->image;
}

bool get_plate_position(vision::platePosition::Request &req, vision::platePosition::Response &res){
	//startWindowThread();
	//namedWindow("test");
	imshow("test",frame);
	waitKey(0);
	//destroyWindow("test");
	//waitKey(1);
	ROS_INFO("match displayed");
	return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  //~ cv::namedWindow("view");
  //~ cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("usb_cam/image_raw", 1, imageCallback);
  
  ros::ServiceServer service = nh.advertiseService("get_plate_position",get_plate_position);
  ROS_INFO("ready to detect the plate!");
  ros::spin();
}
