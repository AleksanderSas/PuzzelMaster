#pragma once
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "BackgroundSeparator.h"

using namespace std;
using namespace cv;

struct edgeFeature
{
	int len;
	Vec3i joint;  //x, y, r
	bool isMaleJoint;
	vector<Vec3b> colors1;
	vector<Vec3b> colors2;
	Point2f start, end;
};

class PuzzelRectange
{
public:
	PuzzelRectange(Point2f& left, Point2f& right, Point2f& lower, Point2f& upper, int id, BackgroundSeparator* backgroundSeparator);

	Point2f left, right, lower, upper;
	Mat puzzelArea;
	double score;
	double hitScore;
	double recScore;
	double interestScore;
	double areaScore;
	double noneBackgroundScore;
	vector<vector<Point>*> *contours;
	int id;
	bool isPointInside(int x, int y);
	Mat computeBackgroundColor();
	Mat background;
	float computeBackgroundSimilarity(Vec3f circle, bool isinside);
	float scoreCircle(Vec3f circle);
	float scoreArea(BackgroundSeparator* separator);
	BackgroundSeparator* backgroundSeparator;

	edgeFeature edgeFeatures[4];

	void ComputeEdgeFeatures(string name);
	void FindNeighbour(vector<PuzzelRectange>& puzzels, int edgeNr, string name);
	void ReconstructBorder();
	void FindBestCircleJoin(vector<Vec3f>& circles, Point2f c1, Point2f c2, edgeFeature* e);
	vector<Vec3f> FindJointCandidates(Mat& puzzelArea);

	Vec3f lineParameters_left_upper;
	Vec3f lineParameters_upper_rigth;
	Vec3f lineParameters_right_lower;
	Vec3f lineParameters_lower_left;

	float backgroundProbability;
	#define QUBE_BIN 12
	#define QUBE_SIZE (256 / QUBE_BIN)
	float qubeBackgroundHistogram[QUBE_SIZE][QUBE_SIZE][QUBE_SIZE];
	float qubeHistogram[QUBE_SIZE][QUBE_SIZE][QUBE_SIZE];
	float isBackground(Vec3b pixel);
	float isNotBackground(Vec3b pixel);
	Mat GetContours();

#if 1
	#define MIN_LINE_LEN 18
	#define LINE_TRESHOLD 18
	#define MAX_LINE_GAP 5
#else
	#define MIN_LINE_LEN 15
	#define LINE_TRESHOLD 15
	#define MAX_LINE_GAP 6
#endif

};

