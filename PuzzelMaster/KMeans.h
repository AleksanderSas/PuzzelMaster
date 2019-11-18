#ifndef __KMeans_header__
#define __KMeans_header__

#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include "PuzzelRectange.h"
#include "IntrestingArea.h"

using namespace cv;
using namespace std;

struct Centre
{
	cv::Point Point;
	vector<cv::Point*> BelongingPoints;
	vector<vector<cv::Point>*> CapturedContours;
};

class KMeans
{
public:
	KMeans(int heigth, int width, int MIN_SQUARE_DISTANCE = 1600, int INIT_GRID_DENSITY = 10);
	~KMeans();
	int Pass(vector<vector<cv::Point> > *contours);
	void DrawBoxes(Mat& img);
	vector<Centre*> centres;
	vector<IntrestingArea> GetPuzzels(Mat &img, Mat &edgeMap);	

private:
	void MergeTooCloseCentries();
	int UpdateCentries();
	void MatchContoursToCentries(std::vector<std::vector<cv::Point>>* contours);
	void ResetKMeansCentries();

	int heigth;
	int width;

	int MIN_SQUARE_DISTANCE = 1600;
	int INIT_GRID_DENSITY = 10;
};
#endif  

