/**
 * @file HoughCircle_Demo.cpp
 * @brief Demo code for Hough Transform
 * @author OpenCV team
 */

//#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <math.h>
#include <stdio.h>

using namespace std;
using namespace cv;

namespace
{
    // windows and trackbars name
    const std::string windowName = "Hough Circle Detection Demo";
    const std::string cannyThresholdTrackbarName = "Canny threshold";
    const std::string accumulatorThresholdTrackbarName = "Accumulator Threshold";
    const std::string usage = "Usage : tutorial_HoughCircle_Demo <path_to_input_image>\n";

    // initial and max values of the parameters of interests.
    const int cannyThresholdInitialValue = 200;
    const int accumulatorThresholdInitialValue = 8;
    const int minRadiusInitialValue = 1;
    const int maxRadiusInitialValue = 30;
    const int minDistanceInitialValue = 50;
    const int dpInitialValue = 1;
    
    const int maxAccumulatorThreshold = 200;
    const int maxCannyThreshold = 255;
	const int maxMinRadius = 60;
    const int maxMaxRadius = 60;
	const int maxMinDistance = 150;
	const int maxDp = 4;
	
	std::vector<Vec3f> circles;

    void HoughDetection(const Mat& src_gray, const Mat& src_display, int cannyThreshold, int accumulatorThreshold, int minRadius, int maxRadius, int minDistance, int dp)
    {
        // will hold the results of the detection
        
        // runs the actual detection
        HoughCircles( src_gray, circles, CV_HOUGH_GRADIENT, dp, minDistance, cannyThreshold, accumulatorThreshold, minRadius, maxRadius );

        // clone the colour, input image for displaying purposes
        Mat display = src_display.clone();
        for( size_t i = 0; i < circles.size(); i++ )
        {
            Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);
            // circle center
            circle( display, center, 3, Scalar(0,255,0), -1, 8, 0 );
            // circle outline
            circle( display, center, radius, Scalar(0,0,255), 3, 8, 0 );
        }

        // shows the results
        imshow( windowName, display);
    }
}

void mouseClick(int event, int x, int y, int flags, void* userdata){
	Mat* display = (Mat*) userdata;
	double minDist = 150000,dist;
	int closestIndex = 0;
	if  ( event == EVENT_LBUTTONDOWN )
    {
        cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
        for( size_t i = 0; i < circles.size(); i++ ){
			dist = (circles[i][0]-x)*(circles[i][0]-x) + (circles[i][1]-y)*(circles[i][1]-y);
			if(dist < minDist){
				minDist = dist;
				closestIndex = (int)i;
			}
        }
        
        printf("data points: %d\n",(int)circles.size());
        printf("minDist %f, i %d\n",minDist,closestIndex);
        if (minDist<1500){
			printf("lalala\n");
			printf("minDist %f, i %d\n",minDist,closestIndex);
			
			Point centre(cvRound(circles[closestIndex][0]), cvRound(circles[closestIndex][1]));
			printf("lalala\n");
			printf("selected");
			circle( *display, centre, 10, Scalar(255,0,0), 3, 8, 0 );
			printf("selected");
			
		}
    }
	
}

int main(int argc, char** argv)
{
    Mat src, src_gray;

    if (argc < 2)
    {
        std::cerr<<"No input image specified\n";
        std::cout<<usage;
        return -1;
    }

    // Read the image
    src = imread( argv[1], 1 );

    if( src.empty() )
    {
        std::cerr<<"Invalid input image\n";
        std::cout<<usage;
        return -1;
    }

    // Convert it to gray
    cvtColor( src, src_gray, COLOR_BGR2GRAY );

    // Reduce the noise so we avoid false circle detection
    GaussianBlur( src_gray, src_gray, Size(9, 9), 2, 2 );

    //declare and initialize both parameters that are subjects to change
    int cannyThreshold = cannyThresholdInitialValue;
    int accumulatorThreshold = accumulatorThresholdInitialValue;
    int minRadius = minRadiusInitialValue;
    int maxRadius = maxRadiusInitialValue;
    int minDistance = minDistanceInitialValue;
    int dp = dpInitialValue;

    // create the main window, and attach the trackbars
    namedWindow( windowName, WINDOW_AUTOSIZE );
    createTrackbar(cannyThresholdTrackbarName, windowName, &cannyThreshold,maxCannyThreshold);
    createTrackbar(accumulatorThresholdTrackbarName, windowName, &accumulatorThreshold, maxAccumulatorThreshold);
    createTrackbar("minRadius", windowName, &minRadius, maxMinRadius);
    createTrackbar("maxRadius", windowName, &maxRadius, maxMaxRadius);
    createTrackbar("minDistance", windowName, &minDistance, maxMinDistance);
    createTrackbar("dp", windowName, &dp, maxDp);
	setMouseCallback(windowName, mouseClick, &src);
	
    // infinite loop to display
    // and refresh the content of the output image
    // until the user presses q or Q
    int key = 0;
    while(key != 'q' && key != 'Q')
    {
        // those paramaters cannot be =0
        // so we must check here
        cannyThreshold = max(cannyThreshold, 1);
        accumulatorThreshold = max(accumulatorThreshold, 1);
		dp = max(dp,1);
		minDistance = max(minDistance,1);
        //runs the detection, and update the display
        HoughDetection(src_gray, src, cannyThreshold, accumulatorThreshold, minRadius, maxRadius, minDistance, dp);

		
		
        // get user key
        key = waitKey(10);
    }

    return 0;
}
