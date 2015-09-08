/**
 * @file object_detector.cpp
 * @author Lingfeng Bian
 * @brief recognise object with SIFT
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
#include <fstream>
#include <string>

#include "vision/platePosition.h"
#include "Constants.h"

#define DATA_FOLDER "/home/robocuphome/robocuphome2015/src/vision/data/"
#define IMAGE_NAME "Plate.jpg" //"id.jpg" "Plate.jpg"
#define IMAGE_TOPIC "/camera/rgb/image_raw" //webcam: "usb_cam/image_raw", kinect:"/camera/rgb/image_color"

using namespace std;
using namespace cv;

/** Global variables */
string window_name = "Good Matches & Object detection";
Mat img_frame, img_object, descriptors_object;
Mat H;//map image position to global position
std::vector<KeyPoint> keypoints_object, keypoints_scene;
//position_reference real
Point2f pr_r1,pr_r2,pr_r3,pr_r4,pr_r5,pr_r6;
//position_reference pixel//set to plate
Point2f pr_p1(247.478,181.249),pr_p2(420.907,181.566),pr_p3(429.926,313.306),pr_p4(230.793,306.769);


/** Function Headers */
int detectAndDisplay( Mat frame, Mat obj, vector<KeyPoint> keypoints_object, Mat descriptors_object,vision::platePosition::Response &res);
void imageCallback(const sensor_msgs::ImageConstPtr& msg);//copy image from topic to global variable frame
bool get_plate_position(vision::platePosition::Request &req, vision::platePosition::Response &res); //service function
bool displayFrame(vision::platePosition::Request &req, vision::platePosition::Response &res);

template <typename T> string tostr(const T& t) { 
   ostringstream os; 
   os<<t; 
   return os.str(); 
} 

Mat readCalibration(ifstream &file){
    std::vector<Point2f> ref_pixel_position;
    std::vector<Point2f> ref_real_position;
    string line;

    getline(file,line);//skip two lines
    getline(file,line);
    printf("calibration info:\n");
    
    //fill ref_pixel_position
    printf("image:\n");
    for (int i=0; i<6; i=i+1){
        //get x
        getline(file,line);
        const char* number =line.substr(line.find("=")+1).c_str();
        float x=atof(number);
        //get y
        getline(file,line);
        float y=atof(line.substr(line.find("=")+1).c_str());
        //push back
        ref_pixel_position.push_back(Point2f(x,y));
        printf("Point %d: x=  %f, y=  %f\n",i,ref_pixel_position[i].x,ref_pixel_position[i].y);
    }
    
    //fill global position
    printf("global:\n");
    getline(file,line);
    for (int i=0; i<6; i=i+1){
        //get x
        getline(file,line);
        const char* number =line.substr(line.find("=")+1).c_str();
        float x=atof(number);
        //get y
        getline(file,line);
        float y=atof(line.substr(line.find("=")+1).c_str());
        //push back
        ref_real_position.push_back(Point2f(x,y));
        printf("Point %d: x=  %f, y=  %f\n",i,ref_real_position[i].x,ref_real_position[i].y);
    }
    return findHomography( ref_pixel_position, ref_real_position);
}

/**
 * @function main
 */
int main( int argc, char** argv )
{
	ros::init(argc, argv, "object_detector");
	ros::NodeHandle nh;
	
	///subscribe to camera image topic
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe((string)IMAGE_TOPIC, 1, imageCallback);
	
    ///read calibration data
    ifstream file (CALIBRATION_FILE);
    if (!file.is_open()){
        printf("ERROR: Unable to open calibration file\n");
        return 2;
    }
    H=readCalibration(file);


    
	//feature calculation of objct image
	img_object = imread( (string)DATA_FOLDER+(string)IMAGE_NAME, CV_LOAD_IMAGE_GRAYSCALE );
	//-- Step 1: Detect the keypoints using SURF Detector
	SiftFeatureDetector detector;
	detector.detect( img_object, keypoints_object );;
	//-- Step 2: Calculate descriptors (feature vectors)
	SiftDescriptorExtractor extractor;
	extractor.compute( img_object, keypoints_object, descriptors_object );
    
	
    //run service
	ros::ServiceServer service = nh.advertiseService("vision/get_plate_position", get_plate_position);
	ros::ServiceServer service1 = nh.advertiseService("vision/displayFrame",displayFrame);
	ROS_INFO("ready to detect the plate");
        
    ros::spin();
	return 0;
}

/**
 * @function detectAndDisplay return 0 if find the object,1 if not
 */
int detectAndDisplay( Mat img_frame, Mat img_object, vector<KeyPoint> keypoints_object, Mat descriptors_object,vision::platePosition::Response &res, Mat H2)
{
    int minHessian = 400;
	SiftFeatureDetector detector;
	SiftDescriptorExtractor extractor;
	std::vector<KeyPoint> keypoints_frame;
	Mat descriptors_frame;
	
	//-- Step 1: Detect the keypoints
	detector.detect( img_frame, keypoints_frame );
	
	//-- Step 2: Calculate descriptors (feature vectors)
	extractor.compute( img_frame, keypoints_frame, descriptors_frame );
	
	//-- Step 3: Matching descriptor vectors using FLANN matcher
	FlannBasedMatcher matcher;
	std::vector< DMatch > matches;
	printf("size: descriptor_object rows: %d\t descriptors_frame rows: %d\n",descriptors_object.rows,descriptors_frame.rows);
	if(!descriptors_frame.rows){
		printf("!!null scene descriptor\n");
		return 1;
	}
	matcher.match( descriptors_object, descriptors_frame, matches );
	//printf("matches size: %d\n",matches.size());
	

	//-- Quick calculation of max and min distances between keypoints
	double max_dist = 0; double min_dist = 1000;
	for( int i = 0; i < descriptors_object.rows; i++ )
	{ double dist = matches[i].distance;
		if( dist < min_dist ) min_dist = dist;
		if( dist > max_dist ) max_dist = dist;
		//printf("i:%d\t",i);
	}
	printf("-- Max dist : %f \n", max_dist );
    printf("-- Min dist : %f \n", min_dist );

	
	//-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
	std::vector< DMatch > good_matches;
	for( int i = 0; i < descriptors_object.rows; i++ )
	{ 
		//~ if( matches[i].distance < 3*max(0.02,min_dist) )
		//~ { good_matches.push_back( matches[i]); }
		if( matches[i].distance < 250)
		{ good_matches.push_back( matches[i]); }
	}
	printf("good matches size %d\n",(int)good_matches.size());
	Mat img_matches;
	drawMatches( img_object, keypoints_object, img_frame, keypoints_frame,
			   good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
			   vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
			  
	//-- Localize the object from img_1 in img_2
	std::vector<Point2f> obj;
	std::vector<Point2f> scene;

	for( size_t i = 0; i < good_matches.size(); i++ )
	{
		//-- Get the keypoints from the good matches
		obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
		scene.push_back( keypoints_frame[ good_matches[i].trainIdx ].pt );
	}
	if (good_matches.size()<=9){
	  printf("insufficient good matches\n");
	  imshow( window_name, img_matches );
	  waitKey(0);
	  
      return 1;
	}
	else{
		Mat H = findHomography( obj, scene, CV_RANSAC );
		printf("lala\n");
		
		//-- Get the corners from the image_1 ( the object to be "detected" )
		std::vector<Point2f> obj_corners(4);
		obj_corners[0] = Point(0,0); obj_corners[1] = Point( img_object.cols, 0 );
		obj_corners[2] = Point( img_object.cols, img_object.rows ); obj_corners[3] = Point( 0, img_object.rows );
		std::vector<Point2f> scene_corners(4);
		
		perspectiveTransform( obj_corners, scene_corners, H);
		
		
		//-- Draw lines between the corners (the mapped object in the scene - image_2 )
		Point2f offset( (float)img_object.cols, 0);
		printf("image object size: row %d, col %d\n",img_object.rows,img_object.cols);
        printf("image scene size: row %d, col %d\n",img_frame.rows,img_frame.cols);
		line( img_matches, scene_corners[0] + offset, scene_corners[1] + offset, Scalar(0, 255, 0), 4 );
		line( img_matches, scene_corners[1] + offset, scene_corners[2] + offset, Scalar( 0, 255, 0), 4 );
		line( img_matches, scene_corners[2] + offset, scene_corners[3] + offset, Scalar( 0, 255, 0), 4 );
		line( img_matches, scene_corners[3] + offset, scene_corners[0] + offset, Scalar( 0, 255, 0), 4 );
        
        string point1 = "p1: "+tostr(scene_corners[0].x)+" "+tostr(scene_corners[0].y);
        string point2 = "p2: "+tostr(scene_corners[1].x)+" "+tostr(scene_corners[1].y);
        string point3 = "p3: "+tostr(scene_corners[2].x)+" "+tostr(scene_corners[2].y);
        string point4 = "p4: "+tostr(scene_corners[3].x)+" "+tostr(scene_corners[3].y);
        putText(img_matches,point1,scene_corners[0] + offset, FONT_HERSHEY_SCRIPT_SIMPLEX, 0.5, Scalar(255,0,255),2);
        putText(img_matches,point2,scene_corners[1] + offset, FONT_HERSHEY_SCRIPT_SIMPLEX, 0.5, Scalar(255,0,255),2);
		putText(img_matches,point3,scene_corners[2] + offset, FONT_HERSHEY_SCRIPT_SIMPLEX, 0.5, Scalar(255,0,255),2);
        putText(img_matches,point4,scene_corners[3] + offset, FONT_HERSHEY_SCRIPT_SIMPLEX, 0.5, Scalar(255,0,255),2);
        
        printf("point:\n%s\n%s\n%s\n%s\n",point1.c_str(),point2.c_str(),point3.c_str(),point4.c_str());
        
		imshow( window_name, img_matches );
        waitKey(0);
        
        
        //to find the plate in real position
        //to calibrate change pr_r* and Point2f pr_p*
        //position_reference real
        Point2f pr_r1(0.852991670452,0.166859215521),pr_r2(0.870399996545, -0.185887708082),pr_r3( 0.588867961436,-0.170358598533),pr_r4(0.597280405865,0.178723491525);
        //position_reference pixel//set to plate
        Point2f pr_p1(247.478,181.249),pr_p2(420.907,181.566),pr_p3(429.926,313.306),pr_p4(230.793,306.769);
    
        
        std::vector<Point2f> ref_pixel_position;//known
        std::vector<Point2f> ref_real_position;//to measure by moving baxter hand to the poit
        //std::vector<Point2f> plate_pixel_position;//=scene corners known
        std::vector<Point2f> plate_real_position;//to find
        
        ref_real_position.push_back(pr_r1);
        ref_real_position.push_back(pr_r2);
        ref_real_position.push_back(pr_r3);
        ref_real_position.push_back(pr_r4);
        
        ref_pixel_position.push_back(pr_p1);
        ref_pixel_position.push_back(pr_p2);
        ref_pixel_position.push_back(pr_p3);
        ref_pixel_position.push_back(pr_p4);
        
        
        Mat H2 = findHomography( ref_pixel_position, ref_real_position);
        
        perspectiveTransform( scene_corners, plate_real_position, H2);
        
        //print out
        string plate_point1 = "p1: "+tostr(plate_real_position[0].x)+" "+tostr(plate_real_position[0].y);
            string plate_point2 = "p2: "+tostr(plate_real_position[1].x)+" "+tostr(plate_real_position[1].y);
            string plate_point3 = "p3: "+tostr(plate_real_position[2].x)+" "+tostr(plate_real_position[2].y);
            string plate_point4 = "p4: "+tostr(plate_real_position[3].x)+" "+tostr(plate_real_position[3].y);
            printf("plate real postion:\n%s\n%s\n%s\n%s\n",plate_point1.c_str(),plate_point2.c_str(),plate_point3.c_str(),plate_point4.c_str());
        
        
        //write to service response
        res.p1[0] = plate_real_position[0].x;
        res.p1[1] = plate_real_position[0].y;
        res.p2[0] = plate_real_position[1].x;
        res.p2[1] = plate_real_position[1].y;
        res.p3[0] = plate_real_position[2].x;
        res.p3[1] = plate_real_position[2].y;
        res.p4[0] = plate_real_position[3].x;
        res.p4[1] = plate_real_position[3].y;
        return 0;
    }
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	img_frame=cv_bridge::toCvCopy(msg, "bgr8")->image;
}

bool get_plate_position(vision::platePosition::Request &req, vision::platePosition::Response &res){
	int detect_error = detectAndDisplay(img_frame, img_object,keypoints_object,descriptors_object,res, H);
	if (detect_error == 0)
		ROS_INFO("match displayed");
	else if (detect_error ==1)
		ROS_INFO("not there");
		
	return true;
}

bool displayFrame(vision::platePosition::Request &req, vision::platePosition::Response &res){
	//startWindowThread();
	//namedWindow("test");
	imshow("test",img_frame);
	waitKey(0);
	//destroyWindow("test");
	//waitKey(1);
	return true;
}

