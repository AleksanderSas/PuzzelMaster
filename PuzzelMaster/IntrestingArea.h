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
	IntrestingArea(Mat areaImage, Mat edgeMap, Rect originRectange, int id);
	PuzzelRectange* findPuzzel(BackgroundSeparator* separator);

	Rect OriginRectange;

private:
	PuzzelRectange* FindBestRectange(vector<Point2f>& corners, Mat& xDeriv, Mat& yDeriv, BackgroundSeparator* separator);

	int id;
	Mat AreaImage;
	Mat EdgeMap;
};

#define MIN_CROSS_PROD 0.997
#define MIN_RECT_DIAGONAL 50*50
