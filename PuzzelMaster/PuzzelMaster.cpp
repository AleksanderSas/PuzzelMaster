#include "windows.h"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <string>
#include "time.h"
#include "PuzzelDetector.h"
#include "KMeans.h"
#include "math.h"

using namespace cv;
using namespace std;

Mat src_gray;
int thresh = 50;
RNG rng(12345);
void thresh_callback(int, void*);

Mat src;
PuzzelDetector* puzzelDetector;


int main(int argc, char** argv)
{
	src = imread(argv[1]);

	if (src.empty())
	{
		cout << "Could not open or find the image!\n" << endl;
		cout << "Usage: " << argv[0] << " <Input image>" << endl;
		return -1;
	}

	puzzelDetector = new PuzzelDetector(src);

	const char* source_window = "Source";
	namedWindow(source_window);
	imshow(source_window, src);
	const int max_thresh = 255;
	createTrackbar("Canny thresh:", source_window, &thresh, max_thresh, thresh_callback);
	thresh_callback(0, 0);
	waitKey();
	return 0;
}
void thresh_callback(int, void*)
{
	puzzelDetector->cannEdgeThresh = thresh;
	auto puzzels = puzzelDetector->DetectPuzzels();
	
	int n = 0;
	Scalar color1(120, 30, 210);
	Scalar color2(220, 20, 10);

	for (auto puzzel = puzzels.begin(); puzzel != puzzels.end(); puzzel++)
	{
		string name("P");
		name += to_string(n++);
		Mat image = puzzel->puzzelArea;
		cout << name
			<< " n=" << n
			<< "  hit score=" << puzzel->hitScore
			<< "  rec score=" << puzzel->recScore
			<< "  int score=" << puzzel->interestScore
			<< "  area score=" << puzzel->areaScore
			<< endl;
		line(image, puzzel->left, puzzel->upper, color2);
		line(image, puzzel->left, puzzel->lower, color2);
		line(image, puzzel->right, puzzel->upper, color2);
		line(image, puzzel->right, puzzel->lower, color2);

		imshow(name, image);
		waitKey(1);
	}
}