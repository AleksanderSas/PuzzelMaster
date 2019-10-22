#pragma once
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

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
	PuzzelRectange(Point2f& left, Point2f& right, Point2f& lower, Point2f& upper, int id);

	Point2f left, right, lower, upper;
	Mat puzzelArea;
	double score;
	double hitScore;
	double recScore;
	double interestScore;
	double areaScore;
	vector<vector<Point>*> *contours;
	int id;
	bool isPointInside(int x, int y);
	Mat computeBackgroundColor();
	Mat background;
	float computeBackgroundSimilarity(Vec3f circle, bool isinside);
	float scoreCircle(Vec3f circle);

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
};

