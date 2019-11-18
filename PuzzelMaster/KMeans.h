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
	KMeans(int heigth, int width);
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

#if 0

#define MIN_SQUARE_DISTANCE  1600
#define INIT_GRID_DENSITY 10

#else

#define MIN_SQUARE_DISTANCE  16000
#define INIT_GRID_DENSITY 10

#endif

};
#endif  

