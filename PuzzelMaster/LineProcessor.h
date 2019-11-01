#pragma once
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

using namespace std;
using namespace cv;

class LineProcessor
{
public:
	static void Process(Point2i p1, Point2i p2, function<bool(int x, int y)> processor);
};

