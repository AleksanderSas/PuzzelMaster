#pragma once
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "BackgroundSeparator.h"

using namespace std;
using namespace cv;

struct edgeFeature
{
	bool isMaleJoint;
	bool hasJoint = false;
	int len;
	Vec3i joint;  //x, y, r
	vector<Vec3b> colors1;
	vector<Vec3b> colors2;
	Point2f start, end;
};

class PuzzelRectange
{
public:
	static pair<double, int> CompareFeatureVectors(edgeFeature* e1, edgeFeature* e2);
	PuzzelRectange(Point2f& left, Point2f& right, Point2f& lower, Point2f& upper, int id, BackgroundSeparator* backgroundSeparator, RotatedRect box);

	Mat ExtractPuzzelAndRotateEdgeToUp(int edgeIdx, int padding);
	Point2f left, right, lower, upper;
	Mat puzzelArea;
	Mat backgroundEdges;
	Mat edges;
	double score;
	double hitScore;
	double recScore;
	double interestScore;
	double areaScore;
	double noneBackgroundScore;
	int id;
	bool isPointInside(int x, int y);
	Mat background;
	float computeBackgroundSimilarity(Vec3f circle, bool isinside);
	float scoreCircle(Vec3f circle);
	float scoreArea(BackgroundSeparator* separator);
	BackgroundSeparator* backgroundSeparator;
	void PrintScores();

	edgeFeature edgeFeatures[4];

	void ComputeEdgeFeatures();
	void FindNeighbour(vector<PuzzelRectange*>& puzzels, int edgeNr, string name);
	void FindBestCircleJoin(vector<Vec3f>& circles, Point2f c1, Point2f c2, edgeFeature* e);
	vector<Vec3f> FindJointCandidates(Mat& puzzelArea);

	Vec3f lineParameters_left_upper;
	Vec3f lineParameters_upper_rigth;
	Vec3f lineParameters_right_lower;
	Vec3f lineParameters_lower_left;

	float isBackground(Vec3b pixel);
	float isNotBackground(Vec3b pixel);
	Mat GetContours();
	RotatedRect box;

	void MarkEdgesOnOriginImage(Mat& image);
	void MarkJointsOnOriginImage(Mat& image);

#if 1
	#define MIN_LINE_LEN 18
	#define LINE_TRESHOLD 18
	#define MAX_LINE_GAP 5
#else
	#define MIN_LINE_LEN 15
	#define LINE_TRESHOLD 15
	#define MAX_LINE_GAP 6
#endif
#define AREA_PADDING 10
#define MIN_RECT_SIZE 50
#define MIN_COVER_SCORE 0.01
};

