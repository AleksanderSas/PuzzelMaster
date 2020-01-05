#pragma once
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

using namespace std;
using namespace cv;

// [A, B, C]
Vec3f ParametrizeLine(Point2f& c1, Point2f& c2);
Point2f TrasformPoint(Point2f& p, Mat& transformation);
Point2f LinearComb(Point2f p1, Point2f p2, float p1Weigth);
int squareDist(Vec3f& line, int x, int y);
int squareDist(Point& p1, Point& p2);
