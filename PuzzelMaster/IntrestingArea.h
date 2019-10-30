#pragma once

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <vector>

using namespace std;
using namespace cv;

class IntrestingArea
{
public:
	
	IntrestingArea(Mat areaImage, Mat edgeMap, vector<vector<cv::Point>*> *contours, Rect originRectange, int id);
	/*IntrestingArea(IntrestingArea&& ia);
	IntrestingArea(IntrestingArea& ia);
	*/
	Mat AreaImage;
	Mat EdgeMap;
	vector<vector<cv::Point>*> *contours;
	Rect OriginRectange;
	int id;
};

