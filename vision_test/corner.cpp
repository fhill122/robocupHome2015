#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>

using namespace cv;
using namespace std;

#define CALIBRATION_FILE "/home/robocuphome/robocuphome2015/src/vision/data/Calibration.ini"

/// Global variables
Mat src, src_gray;

int maxCorners = 90;
int maxTrackbar = 150;
vector<Point2f> corners;
vector<Point2f> corners_selected;

RNG rng(12345);
String source_window = "Image";

/// Function header
void goodFeaturesToTrack_Demo( int, void* );
void mouseClick(int event, int x, int y, int flags, void* userdata);

/**
 * @function main
 */
int main( int argc, char** argv ){
    
    if (argc !=2){
        printf("ERROR: Please input image file");
        return 1;
    }
    
    ofstream file (CALIBRATION_FILE);
    if (!file.is_open()){
        printf("ERROR: Unable to open calibration file");
        return 2;
    }
    
    /// Load source image and convert it to gray
    src = imread( argv[1], 1 );
    cvtColor( src, src_gray, CV_BGR2GRAY );
    
    /// Create Window
    namedWindow( source_window, CV_WINDOW_AUTOSIZE );
    
    /// Create Trackbar to set the number of corners
    createTrackbar( "Max  corners:", source_window, &maxCorners, maxTrackbar, goodFeaturesToTrack_Demo );
    
    /// set mouse click behavour
    setMouseCallback(source_window, mouseClick, &src);
    
    imshow( source_window, src );
    
    goodFeaturesToTrack_Demo( 0, 0 );
    
    waitKey(0);
    
    /// Write to calibration file
    if (corners_selected.size() != 6){
        printf("!!Error!! \n!!Please select 6 points!!\n");
    }
    else{
        file << "[image]\n";
        for (int i=0; i<corners_selected.size();i++){
            file << "point"<<(1+i)<<"x="<<corners[i].x<<"\n";
            file << "point"<<(1+i)<<"y="<<corners[i].y<<"\n";
        }
        file << "[global]\n";
        file << "distanceToTable=fill in this\n";
        printf("Calibration completed...\n");
    }
    return(0);
}

/**
 * @function goodFeaturesToTrack_Demo.cpp
 * @brief Apply Shi-Tomasi corner detector
 */
void goodFeaturesToTrack_Demo( int, void* )
{
  if( maxCorners < 1 ) { maxCorners = 1; }

  /// Parameters for Shi-Tomasi algorithm
  double qualityLevel = 0.01;
  double minDistance = 10;
  int blockSize = 3;
  bool useHarrisDetector = false;
  double k = 0.04;

  /// Copy the source image
  Mat copy;
  copy = src.clone();

  /// Apply corner detection
  goodFeaturesToTrack( src_gray,
               corners,
               maxCorners,
               qualityLevel,
               minDistance,
               Mat(),
               blockSize,
               useHarrisDetector,
               k );


  /// Draw corners detected
  //cout<<"** Number of corners detected: "<<corners.size()<<endl;
  int r = 4;
  for( int i = 0; i < corners.size(); i++ )
     { circle( copy, corners[i], r, Scalar(0,255,0), -1, 8, 0 ); }

  /// Show what you got
  namedWindow( source_window, CV_WINDOW_AUTOSIZE );
  imshow( source_window, copy );
}


void mouseClick(int event, int x, int y, int flags, void* userdata){
	Mat* display = (Mat*) userdata;
	double minDist = 150000,dist;
	int closestIndex = 0;
	if  ( event == EVENT_LBUTTONDOWN )
    {
        //~ cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
        for( size_t i = 0; i < corners.size(); i++ ){
			dist = (corners[i].x-x)*(corners[i].x-x) + (corners[i].y-y)*(corners[i].y-y);
			if(dist < minDist){
				minDist = dist;
				closestIndex = (int)i;
			}
        }
        //~ printf("minDist %f, i %d\n",minDist,closestIndex);
        if (minDist<1000){
			Point centre(cvRound(corners[closestIndex].x), cvRound(corners[closestIndex].y));
			circle( *display, centre, 10, Scalar(255,0,0), 3, 8, 0 );
			//~ printf("selected\n");
            corners_selected.push_back(corners[closestIndex]);
			goodFeaturesToTrack_Demo(0,0);
		}
    }
	
}
