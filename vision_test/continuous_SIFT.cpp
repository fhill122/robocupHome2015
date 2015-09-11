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
void detectAndDisplay( Mat frame, Mat obj, vector<KeyPoint> keypoints_object, Mat descriptors_object);
void readme();
double testfps();


class FPS{
	time_t start;
	time_t end;
	time_t last;
	int counter;
	
	public:
		FPS(){counter=0;}
		double getAverage();
		double getCurrent();
		void showAverage(){printf("%.0f fps\n", getAverage());};
		void showCurrent(){printf("%.0f fps\n", getCurrent());};	
		//todo function to set counter to 0
};

/** Global variables */
string window_name = "Good Matches & Object detection";

/**
 * @function main
 */
int main( int argc, char** argv )
{
	//read image file name from arg
	if( argc != 2 )
	{ readme(); return -1; }
	Mat img_object = imread( argv[1], CV_LOAD_IMAGE_GRAYSCALE );

	VideoCapture capture;
	//no use, to fix, possible driver problem
	capture.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
    //capture.set(CV_CAP_PROP_CONVERT_RGB , false);
    
	Mat frame;
	FPS fps;

	//-- Step 1: Detect the keypoints using SURF Detector
	int minHessian = 400;
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
	//resizeWindow(window_name,1920,1080);
	//Read the video stream
	capture.open( 0 );
	if( capture.isOpened() )
	{
		for(;;)
		{
			capture >> frame;
		
			if( !frame.empty() )
				{ detectAndDisplay( frame, img_object,keypoints_object,descriptors_object ); }
			else
				{ printf(" --(!) No captured frame -- Break!"); break; }
		
		
		
			fps.showAverage();

			int c = waitKey(10);
			if( (char)c == 'c' ) { break; }
		  
		  //std::cout << "width: " << capture.get(CV_CAP_PROP_FRAME_WIDTH) << std::endl;		
		}
	}
	return 0;
}

/**
 * @function detectAndDisplay
 */
void detectAndDisplay( Mat img_frame, Mat img_object, vector<KeyPoint> keypoints_object, Mat descriptors_object )
{
    int minHessian = 400;
	SiftFeatureDetector detector;
	SiftDescriptorExtractor extractor;
	std::vector<KeyPoint> keypoints_frame;
	Mat descriptors_frame;
	
	//-- Step 1: Detect the keypoints
	detector.detect( img_frame, keypoints_frame );
	
	//-- Draw keypoints
	//~ Mat img_keypoints_frame;
	//~ 
	//~ drawKeypoints( img_frame, keypoints_1, img_keypoints_frame, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
	//~ 
	//~ imshow( window_name, img_keypoints_frame );
	
	//-- Step 2: Calculate descriptors (feature vectors)
	extractor.compute( img_frame, keypoints_frame, descriptors_frame );
	
	//-- Step 3: Matching descriptor vectors using FLANN matcher
	FlannBasedMatcher matcher;
	std::vector< DMatch > matches;
	printf("size: descriptor_object rows: %d\t descriptors_frame rows: %d\n",descriptors_object.rows,descriptors_frame.rows);
	if(!descriptors_frame.rows)
		return;
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
		if( matches[i].distance < 150)
		{ good_matches.push_back( matches[i]); }
	}
	printf("good matches size %d\n",good_matches.size());
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
	if (good_matches.size()<=20){
	  imshow( "Good Matches & Object detection", img_matches );
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
		line( img_matches, scene_corners[0] + offset, scene_corners[1] + offset, Scalar(0, 255, 0), 4 );
		line( img_matches, scene_corners[1] + offset, scene_corners[2] + offset, Scalar( 0, 255, 0), 4 );
		line( img_matches, scene_corners[2] + offset, scene_corners[3] + offset, Scalar( 0, 255, 0), 4 );
		line( img_matches, scene_corners[3] + offset, scene_corners[0] + offset, Scalar( 0, 255, 0), 4 );
	
		
		imshow( window_name, img_matches );
	}

}

double FPS::getAverage(){
	if (counter == 0){
		time(&start);
		counter++;
		return -1;
	}
    else{
		time(&end);
        double sec = difftime(end, start);
        double fps_value = counter/sec;
        counter++;
        // overflow protection
        if (counter == (INT_MAX - 1000))
            counter = 0;
		return fps_value;
	}
}

//to do, error reason, minimum time unit is 1 second
double FPS::getCurrent(){
	time_t current=time(NULL);
	double sec = difftime(current, last);
	printf("%.4f\n",sec);
	double fps_value = 1/sec;
	last=current;
	return fps_value;
}



/**
 * @function readme
 */
void readme()
{ std::cout << " Usage: ./SURF_Homography <img>" << std::endl; }
