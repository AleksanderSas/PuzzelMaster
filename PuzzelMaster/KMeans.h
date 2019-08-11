#ifndef __KMeans_header__
#define __KMeans_header__

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>

using namespace cv;
using namespace std;

struct Centre
{
	cv::Point Point;
	vector<cv::Point*> BelongingPoints;
	vector<vector<cv::Point>*> CapturedContours;
};

struct PuzzelRectange
{
	Point2f left, right, lower, upper;
	Mat puzzelArea;
	double score;
	double hitScore;
	double recScore;
	double interestScore;
	double areaScore;
};

class KMeans
{
public:
	KMeans(int heigth, int width, int MIN_SQUARE_DISTANCE = 1600, int INIT_GRID_DENSITY = 10);
	int ComputeCentres(vector<vector<cv::Point> > &contours);
	void DrawEigenVectors(Mat& img);
	void DrawBoxes(Mat& img);
	vector<Centre*> centres;
	void drawApprovedContours(Mat& img);
	vector<pair<Mat, Mat>> GetPuzzels(Mat &img, Mat &edgeMap);
	vector<PuzzelRectange>* FindBestRectange(vector<Point2f> &corners, Mat& xDeriv, Mat& yDeriv, Mat& edgeMap);
	

private:
	int Pass(vector<vector<cv::Point> > &contours);
	int heigth;
	int width;

	int MIN_SQUARE_DISTANCE = 1600;
	int INIT_GRID_DENSITY = 10;
};

void ComputeEdgeHits(Mat& edgeMap, PuzzelRectange& puzzel, int& hit, int& miss);

#endif  

