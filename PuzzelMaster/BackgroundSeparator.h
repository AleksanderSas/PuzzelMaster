#pragma once

#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

using namespace std;
using namespace cv;

class BackgroundSeparator
{
public:
	BackgroundSeparator(Mat& img, vector<Rect> puzzelCandidateAreas);
	float scorePoint(Vec3b* pixel);

private:
	Mat& image;
	void buildHistogram(function<float(Point2i&)> filter);
	void initializeHistogram(vector<Rect>& puzzelCandidateArea);
	void tune();
	void showImg(const char* name = "backgroundMap");
	void normalizeHistograms(float *start, int pointCount);
	void normalizeBackgroundHistograms(int pointCount);
	Mat getMatchMap();

#define QUBE_BIN 4
#define QUBE_SIZE (256 / QUBE_BIN)
#define TOTAL_QUBE_SIZE QUBE_SIZE * QUBE_SIZE * QUBE_SIZE
	float qubeBackgroundHistogram[QUBE_SIZE][QUBE_SIZE][QUBE_SIZE];
	float qubeHistogram[QUBE_SIZE][QUBE_SIZE][QUBE_SIZE];
	double backgroundProbability = 0.0;
	float& getQubeBackgroundBin(Vec3b *pixel);
	float& getQubeColorBin(Vec3b* p);
};

