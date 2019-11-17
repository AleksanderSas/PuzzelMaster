#pragma once

#include "opencv2/imgcodecs.hpp"
#include "KMeans.h"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

class PuzzelDetector
{
public: 
	PuzzelDetector(Mat& input);
	vector<PuzzelRectange*> DetectPuzzels();

	void RemoveTooLongLines(cv::Mat& canny_output);

	int cannEdgeThresh = 70;
	int dilation_size = 1;

private: 
	Mat ComputeEdgeMap(vector<vector<Point>>& contours);
	void DrawContours(vector<vector<Point> >& contours, vector<Vec4i> &hierarchy);

	Mat& image;
	Mat image_gray;
	Mat contourDrawing;
	KMeans knn;

#define MIN_LINE_LEN_TO_REMOVE 80
#if 0

#define MIN_SQUARE_DISTANCE  1600
#define INIT_GRID_DENSITY 10

#else

#define MIN_SQUARE_DISTANCE  16000
#define INIT_GRID_DENSITY 10

#endif
};

