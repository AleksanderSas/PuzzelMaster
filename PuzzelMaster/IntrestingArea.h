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
	IntrestingArea(Mat areaImage, Mat edgeMap, Rect originRectange, int id, RotatedRect box);
	PuzzelRectange* findPuzzel(BackgroundSeparator* separator);

	Rect OriginRectange;

private:
	PuzzelRectange* FindBestRectange(vector<Point2f>& corners, BackgroundSeparator* separator);

	int id;
	Mat AreaImage;
	Mat EdgeMap;
	RotatedRect box;
};

#define MIN_CROSS_PROD 0.994
#define MIN_RECT_DIAGONAL 50*50
