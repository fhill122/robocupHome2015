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
#include "../../../path.h"

#include "object_detector.h"


using namespace std;
using namespace cv;


/** Global variables */
string calibration_file_string((string)PROJECT_PATH+(string)CALIBRATION_FILE);
string objects_file_string((string)PROJECT_PATH+(string)OBJECTS_FILE);
vector <DesktopObject> objects;

string window_name = "Good Matches & Object detection";
Mat img_frame, img_object;
Mat H;//map image position to global position

/**
 * @function main
 */
int main( int argc, char** argv ){
	
	ros::init(argc, argv, "object_detector");
	ros::NodeHandle nh;
	
	///subscribe to camera image topic
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe((string)IMAGE_TOPIC, 1, imageCallback);
	
    ///read calibration data
    printf("openning calibration file: %s\n",calibration_file_string.c_str());
    ifstream calibration_file(calibration_file_string.c_str());
    if (!calibration_file.is_open()){
        printf("ERROR: Unable to open calibration file\n");
        return 2;
    }
    H=readCalibration(calibration_file);
    calibration_file.close();
    
    ///read object database
    printf("openning image database file: %s\n",objects_file_string.c_str());
    ifstream objects_file(objects_file_string.c_str());
    if (!objects_file.is_open()){
        printf("ERROR: Unable to open objects file\n");
        return 2;
    }
    readObjectsDataFile(objects_file,objects);
    objects_file.close();
	
    ///run service
	ros::ServiceServer service = nh.advertiseService("vision/get_object_position", get_object_position);
	ros::ServiceServer service1 = nh.advertiseService("vision/displayFrame",displayFrame);
	ROS_INFO("ready to detect the plate");
        
    ros::spin();
	return 0;
}

/**
 * @function detectAndDisplay return 0 if find the object,1 if not
 */
int detectAndDisplay( Mat img_frame, Mat img_object, float matchDistance, int matchNumber, vector<KeyPoint> keypoints_object, Mat descriptors_object,vision::platePosition::Response &res, Mat H2, double h)
{

	std::vector<KeyPoint> keypoints_frame;
	Mat descriptors_frame;
    SIFTfeatureCalculate(img_frame, keypoints_frame,descriptors_frame);
	

	///Matching descriptor vectors using FLANN matcher
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
	for( int i = 0; i < descriptors_object.rows; i++ ){
		if( matches[i].distance < matchDistance)
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
	if (good_matches.size()<matchNumber){
	  printf("insufficient good matches\n");
	  imshow( window_name, img_matches );
	  waitKey(0);
	  
      return 1;
	}
    ///object founded
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
        
        //mapping to global position
        std::vector<Point2f> plate_real_position;//to find
        perspectiveTransform( scene_corners, plate_real_position, H2);
        //shift global position due to object height
        double height = CameraZ- TableZ;
        for (size_t i=0; i<plate_real_position.size(); i++){
            plate_real_position[i].x = plate_real_position[i].x - h/height * (plate_real_position[i].x - CameraX);
            plate_real_position[i].y = plate_real_position[i].y - h/height * plate_real_position[i].y;
        }
        
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

/// convert image topic to opencv image
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	img_frame=cv_bridge::toCvCopy(msg, "bgr8")->image;
}

/// service callback return position
bool get_object_position(vision::platePosition::Request &req, vision::platePosition::Response &res){
    
    ///find if object is in database
    int found = -1;
    
    for (size_t i=0; i<objects.size(); i++){
        if(objects[i].getName() == req.objectName){
            found = (int)i;
            printf("Detecting %s\n",req.objectName.c_str());
            break;
        }
    }
    
    if(found == -1){
        ROS_ERROR("Not found in objects database!");
        return false;
    }

    ///detect object
    int detect_error =1;
    vector<Mat> obj_images = objects[found].getImages();
    //go through each image for that object
    for (size_t i=0; i<obj_images.size(); i++){
        std::vector<KeyPoint> keypoints_object;
        Mat descriptors_object;
        
        SIFTfeatureCalculate(obj_images[i], keypoints_object,descriptors_object);
        
        detect_error = detectAndDisplay(img_frame, obj_images[i], objects[found].getMatchDistance(i),objects[found].getMatchNumber(i),
            keypoints_object,descriptors_object,res, H, objects[found].getHeight() );
        
        //found
        if (detect_error == 0)
			break;
    }
	if (detect_error == 0)
		ROS_INFO("match displayed");
	else if (detect_error ==1)
		ROS_WARN("not there");
		
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

int SIFTfeatureCalculate(Mat &img, vector<KeyPoint> &keypoints,Mat &descriptors ){
    SiftFeatureDetector detector;
    SiftDescriptorExtractor extractor;
    
    detector.detect( img, keypoints );
    extractor.compute( img, keypoints, descriptors );
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
        float x=atof(number);Mat readCalibration(ifstream &file);
        //get y
        getline(file,line);
        float y=atof(line.substr(line.find("=")+1).c_str());
        //push back
        ref_real_position.push_back(Point2f(x,y));
        printf("Point %d: x=  %f, y=  %f\n",i,ref_real_position[i].x,ref_real_position[i].y);
    }
    return findHomography( ref_pixel_position, ref_real_position);
}

//function to used in ini read
string findParameter(string line, string name){
    size_t found;
    found = line.find(name+"=");
    
    if(found == std::string::npos){
        ROS_ERROR("Check Objects.ini format");
    }
    
    string value =line.substr(name.length()+1,string::npos);
    cout <<name<<": "<<value<<"\n";
    return value;
}

//object data ini reader
int readObjectsDataFile(ifstream &file, vector <DesktopObject> &objects){
    string line;
    
    int numberLoaded=0;
    while(true){
        
        getline(file,line);
        //finished
        
        if(line == "[end]")
            break;
        else{
            //name
            string name = line.substr(1,line.length()-2);
            cout <<"\nname: "<<name<<"\n";
            
            //height
            getline(file,line);
            double height = strtod( findParameter(line, "height").c_str(), NULL);
            
            //size
            getline(file,line);
            double size = strtod( findParameter(line, "size").c_str(), NULL);
            
            //number of images
            getline(file,line);
            int imageNo = atof( findParameter(line, "imageNo").c_str());
            
            //add object
            objects.push_back( DesktopObject(name,height,size) );
            
            ////add object images, and match distance and threshold number for each image
            for(int j=0;j< imageNo; j++){
                string img_path = (string)PROJECT_PATH+(string)DATA_FOLDER+"/"+(string)name + tostr(j+1) + ".jpg";
                cout<< img_path<<"\n";
                Mat img = imread( img_path, CV_LOAD_IMAGE_GRAYSCALE );
                imshow(name,img);
                waitKey(0);
                destroyWindow(name);
                
                //match of distance
                getline(file,line);
                float matchDistance = (float)strtod( findParameter(line, "matchDistance").c_str(), NULL);
                getline(file,line);
                int matchNumber = atof( findParameter(line, "matchNumber").c_str());
                objects[numberLoaded].addImages(img, matchDistance, matchNumber);
            }
            
            numberLoaded++;
        }
        
        
    }

    printf("\nTotal %d objects loaded\n",(int)objects.size());
    for(size_t i=0;i< objects.size();i++){
        cout<<objects[i].getName()<<"\n";
    }
    cout<<"object data loading completed..\n\n";
    return 0;
}


