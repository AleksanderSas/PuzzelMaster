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

Mat ComposePuzzels(vector<PuzzelRectange> puzzels)
{
	int maxCols = 0;
	int maxRows = 0;

	for (PuzzelRectange& puzzel : puzzels)
	{
		if (maxCols < puzzel.puzzelArea.cols)
		{
			maxCols = puzzel.puzzelArea.cols;
		}
		if (maxRows < puzzel.puzzelArea.rows)
		{
			maxRows = puzzel.puzzelArea.rows;
		}
	}

	int puzzelsPerRow = sqrt(puzzels.size());
	int rowNr = ceil(1.0 * puzzels.size() / puzzelsPerRow);
	Mat mosaic(maxRows * rowNr, maxCols * puzzelsPerRow, puzzels[0].puzzelArea.type());

	int i = 0;
	int row = 0;
	Scalar textColor(0, 0, 0);
	while (i < puzzels.size())
	{
		for (int k = 0; k < puzzelsPerRow && i < puzzels.size(); k++, i++)
		{
			Mat source = puzzels[i].puzzelArea;
			for (int y = 0; y < source.rows; y++)
			{
				for (int x = 0; x < source.cols; x++)
				{
					mosaic.at<Vec3b>(y + maxRows * row, x + maxCols * k) = source.at<Vec3b>(y, x);
				}
			}
			putText(mosaic, to_string(puzzels[i].id), Point(maxCols * k + 10, maxRows * row + 10), HersheyFonts::FONT_HERSHEY_PLAIN, 1.0, textColor);
		}
		row++;
	}
	return mosaic;
}


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
		//puzzel->ReconstructBorder();
		//puzzel->computeBackgroundColor();
		string name("Pxx");
		puzzel->ComputeEdgeFeatures(name + "_R");
		name += to_string(n++);
		Mat image = puzzel->puzzelArea;
		cout << name
			<< " n=" << n
			<< "  hit score=" << puzzel->hitScore
			<< "  rec score=" << puzzel->recScore
			<< "  int score=" << puzzel->interestScore
			<< "  area score=" << puzzel->areaScore
			<< endl;
		/*line(image, puzzel->left, puzzel->upper, color2);
		line(image, puzzel->left, puzzel->lower, color2);
		line(image, puzzel->right, puzzel->upper, color2);
		line(image, puzzel->right, puzzel->lower, color2);*/

		//imshow(name, image);
		
		
	}

	puzzels[3].FindNeighbour(puzzels, 0, "w0");
	puzzels[3].FindNeighbour(puzzels, 1, "w1");
	puzzels[3].FindNeighbour(puzzels, 2, "w2");
	puzzels[3].FindNeighbour(puzzels, 3, "w3");
	//imshow("source winner", puzzels[1].puzzelArea);

	imshow("mosaic", ComposePuzzels(puzzels));
	waitKey(1);
}