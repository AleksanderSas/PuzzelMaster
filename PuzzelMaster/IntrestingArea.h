#pragma once

#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <vector>
#include "BackgroundSeparator.h"
#include "PuzzelRectange.h"
#include "DebugFlags.h"

using namespace std;
using namespace cv;

class IntrestingArea
{
public:
	IntrestingArea(Mat areaImage, Mat edgeMap, Rect originRectange, int id, RotatedRect box);
	PuzzelRectange* findPuzzel(BackgroundSeparator* separator, unsigned int& idSequence, int minPuzzelSize);

	Rect OriginRectange;

private:
	std::vector<cv::Point2f> FindIntrestingPointsFromImage(double minDistance);
	vector<Point2f> FindIntrestingPointsFromBackgroundLines(cv::Mat& b, double minDistance);
	PuzzelRectange* FindBestRectange(vector<Point2f>& corners, BackgroundSeparator* separator, unsigned int& idSequence, int minPuzzelSize);

	int id;
	Mat AreaImage;
	Mat EdgeMap;
	RotatedRect box;
};

#define MIN_CROSS_PROD 0.994
