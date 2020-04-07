#pragma once

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
	static PuzzelDetector* Create(char* input, int expectedPuzzelSize, int cannEdgeThresh = 50);
	PuzzelDetector(Mat& input, int expectedPuzzelSize);
	vector<PuzzelRectange*> DetectPuzzels();

	vector<PuzzelRectange*> AddPuzzelsToList(std::vector<IntrestingArea>& puzzelAreas, BackgroundSeparator*& separator);

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
	int expectedPuzzelSize;

#define MIN_LINE_LEN_TO_REMOVE 80
#define MIN_PUZZEL_SIZE expectedPuzzelSize * 0.75
};

