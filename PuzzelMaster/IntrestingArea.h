#pragma once

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <vector>
#include "BackgroundSeparator.h"
#include "PuzzelRectange.h"

using namespace std;
using namespace cv;

class IntrestingArea
{
public:
	
	IntrestingArea(Mat areaImage, Mat edgeMap, vector<vector<cv::Point>*> *contours, Rect originRectange, int id);

	Mat AreaImage;
	Mat EdgeMap;
	vector<vector<cv::Point>*> *contours;
	Rect OriginRectange;
	int id;

	PuzzelRectange* findPuzzel(BackgroundSeparator* separator);

private:
	PuzzelRectange* FindBestRectange(vector<Point2f>& corners, Mat& xDeriv, Mat& yDeriv, BackgroundSeparator* separator);
};

#define MIN_CROSS_PROD 0.997
