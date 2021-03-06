#pragma once

#include "string"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

using namespace std;
using namespace cv;

class Presenter
{
public:
	static void ShowScaledImage(string& name, Mat image, int maxCols = 1200, int maxRows = 700);
	static void ShowScaledImage(const char* name, Mat image, int maxCols = 1200, int maxRows = 700);

private:
	static void PrepareImage(cv::Mat& image, int maxCols, int maxRows);
};

