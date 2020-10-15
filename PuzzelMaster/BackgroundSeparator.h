#pragma once

#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

using namespace std;
using namespace cv;

class BackgroundSeparator
{
public:
	BackgroundSeparator(Mat& img, vector<Rect> puzzelCandidateAreas, Mat& edgeMap);
	float scorePoint(Vec3b* pixel);

	Mat getBackgroundMap(Mat& input);

private:
	Mat& image;
	Mat& edgeMap;
	void buildHistogram(function<float(Point2i&)> filter);
	void initializeHistogram(vector<Rect>& puzzelCandidateArea);
	void tune();
	void showImg(const char* name = "backgroundMap");
	void normalizeHistograms(float *start, int pointCount);
	void normalizeBackgroundHistograms(int pointCount);
	Mat getMatchMap();
	void MatchToEdges(Mat& probablityMap);
	void UpdateMap(bool processX, int startX, int endX, cv::Mat& sourceMap, int i, bool processY, int startY, int endY, int j, cv::Mat& destinationMap);
	int ProcessYPlus(int i, int j, bool& processY);
	int ProcessYMinus(int i, int j, bool& processY);
	int ProcessXPlus(int j, int i, bool& processX);
	int ProcessXMinus(int j, int i, bool& processX);

#define EDGE_MATCH_PADDING 10
#define QUBE_BIN 4
#define QUBE_SIZE (256 / QUBE_BIN)
#define TOTAL_QUBE_SIZE QUBE_SIZE * QUBE_SIZE * QUBE_SIZE
	float qubeBackgroundHistogram[QUBE_SIZE][QUBE_SIZE][QUBE_SIZE];
	float qubeHistogram[QUBE_SIZE][QUBE_SIZE][QUBE_SIZE];
	double backgroundProbability = 0.0;
	float& getQubeBackgroundBin(Vec3b *pixel);
	float& getQubeColorBin(Vec3b* p);
};

void update(unsigned char* p_dest, int x);
