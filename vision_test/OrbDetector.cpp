#include <iostream>
#include <opencv2/opencv.hpp>

#include "opencv2/features2d/features2d.hpp"

using namespace cv;

int main(int argc, char **argv){

    Mat img = imread(argv[1]);

    std::vector<KeyPoint> kp;

    // Default parameters of ORB
    int nfeatures=500;
    float scaleFactor=1.2f;
    int nlevels=8;
    int edgeThreshold=15; // Changed default (31);
    int firstLevel=0;
    int WTA_K=2;
    int scoreType=ORB::HARRIS_SCORE;
    int patchSize=31;
    int fastThreshold=20;

    Ptr<ORB> detector = ORB::create(
    nfeatures,
    scaleFactor,
    nlevels,
    edgeThreshold,
    firstLevel,
    WTA_K,
    scoreType,
    patchSize,
    fastThreshold );

    detector->detect(img, kp);
    std::cout << "Found " << kp.size() << " Keypoints " << std::endl;

    Mat out;
    drawKeypoints(img, kp, out, Scalar::all(255));

    imshow("Kpts", out);

    waitKey(0);
    return 0;
}
