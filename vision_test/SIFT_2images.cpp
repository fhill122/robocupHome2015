/**
 * @file objectDetection.cpp
 * @author A. Huaman ( based in the classic facedetect.cpp in samples/c )
 * @brief A simplified version of facedetect.cpp, show how to load a cascade classifier and how to find objects (Face + eyes) in a video stream
 */
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"

#include <iostream>
#include <stdio.h>

using namespace std;
using namespace cv;

/** Function Headers */
int detectAndDisplay( Mat frame, Mat obj, vector<KeyPoint> keypoints_object, Mat descriptors_object);
void readme();

/** Global variables */
string window_name = "Good Matches & Object detection";

template <typename T> string tostr(const T& t) { 
   ostringstream os; 
   os<<t; 
   return os.str(); 
} 

/**
 * @function main
 */
int main( int argc, char** argv )
{
	//read image file name from arg
	if( argc != 3 )
	{ readme(); return -1; }
	Mat img_object = imread( argv[1], CV_LOAD_IMAGE_GRAYSCALE );
    Mat img_scene = imread(argv[2], CV_LOAD_IMAGE_GRAYSCALE);

	//-- Step 1: Detect the keypoints using SURF Detector
	//int minHessian = 400;
	SiftFeatureDetector detector;
	std::vector<KeyPoint> keypoints_object, keypoints_scene;
	detector.detect( img_object, keypoints_object );
	//detector.detect( img_scene, keypoints_scene );

	//-- Step 2: Calculate descriptors (feature vectors)
	SiftDescriptorExtractor extractor;
	Mat descriptors_object, descriptors_scene;
	extractor.compute( img_object, keypoints_object, descriptors_object );
	//~ extractor.compute( img_scene, keypoints_scene, descriptors_scene );
	
	namedWindow(window_name,WINDOW_NORMAL);
    

    detectAndDisplay(img_scene, img_object,keypoints_object,descriptors_object);
    
    

    
    waitKey(0);
	return 0;
}

/**
 * @function detectAndDisplay return 0 if find the object,1 if not
 */
int detectAndDisplay( Mat img_frame, Mat img_object, vector<KeyPoint> keypoints_object, Mat descriptors_object )
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
	if(!descriptors_frame.rows)
		return 1;
	matcher.match( descriptors_object, descriptors_frame, matches );
	//printf("matches size: %d\n",matches.size());
	  

	//-- Quick calculation of max and min distances between keypoints
	double max_dist = 0; double min_dist = 100;
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
		if( matches[i].distance < 170)
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
	if (good_matches.size()<=12){
	  //imshow( "Good Matches & Object detection", img_matches );
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
        //sleep(10);
        
        
        
        //to find the plate in real position
        //to calibrate change pr_r* and Point2f pr_p*
        //position_reference real
        Point2f pr_r1(0.321,0.494),pr_r2(0.298,-0.534),pr_r3(0.321+0.75,0.494),pr_r4(0.298+0.75,-0.534);
        //position_reference pixel
        Point2f pr_p1(0,438),pr_p2(631,425),pr_p3(74,70),pr_p4(570,72);
    
        
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
        }
        return 0;

}


/**
 * @function readme
 */
void readme()
{ std::cout << " Usage: ./SIFT_2images <img_object> <img_scene>" << std::endl; }
