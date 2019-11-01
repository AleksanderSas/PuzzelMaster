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

	int cannEdgeThresh = 70;
	int dilation_size = 2;

private: 
	Mat ComputeEdgeMap(vector<vector<Point>>& contours);
	void DrawContours(vector<vector<Point> >& contours, vector<Vec4i> &hierarchy);

	Mat& image;
	Mat image_gray;
	Mat contourDrawing;
	KMeans knn;

#if 0

#define MIN_SQUARE_DISTANCE  1600
#define INIT_GRID_DENSITY 10

#else

#define MIN_SQUARE_DISTANCE  16000
#define INIT_GRID_DENSITY 10

#endif
};

