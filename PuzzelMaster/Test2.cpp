#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/imgcodecs.hpp"
#include <iostream>
using namespace std;
using namespace cv;

// Read image
int main75(int argc, char* argv[])
{
	Mat im = imread(argv[1], IMREAD_GRAYSCALE);

	cv::SimpleBlobDetector::Params params;
	params.minThreshold = 20;
	params.maxThreshold = 100;
	cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
	std::vector<cv::KeyPoint> keypoints;
	detector->detect(im, keypoints);
	drawKeypoints(im, keypoints, im, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	cv::imshow("crop", im);
	cv::waitKey(0);
	return 0;
}