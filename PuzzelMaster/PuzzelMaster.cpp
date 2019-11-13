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

Mat src;
PuzzelDetector* puzzelDetector;

// D:\puzzle\2\test_2_2.jpg
// D:\puzzle\test_3.jpg
// D:\puzzle\3\test_3_1.jpg

template<typename T>
Mat ComposePuzzels(vector<PuzzelRectange*>& puzzels, function<Mat(PuzzelRectange*)> selector, Scalar textColor)
{
	int maxCols = 0;
	int maxRows = 0;

	for (PuzzelRectange* puzzel : puzzels)
	{
		Mat img = selector(puzzel);
		if (maxCols < img.cols)
		{
			maxCols = img.cols;
		}
		if (maxRows < img.rows)
		{
			maxRows = img.rows;
		}
	}

	int puzzelsPerRow = ceil(sqrt(puzzels.size()));
	int rowNumber = ceil(1.0 * puzzels.size() / puzzelsPerRow);
	Mat mosaic = Mat::zeros(maxRows * rowNumber, maxCols * puzzelsPerRow, selector(puzzels[0]).type());

	int i = 0;
	int currentRow = 0;
	while (i < puzzels.size())
	{
		for (int k = 0; k < puzzelsPerRow && i < puzzels.size(); k++, i++)
		{
			Mat source = selector(puzzels[i]);
			for (int y = 0; y < source.rows; y++)
			{
				for (int x = 0; x < source.cols; x++)
				{
					mosaic.at<T>(y + maxRows * currentRow, x + maxCols * k) = source.at<T>(y, x);
				}
			}
			putText(mosaic, to_string(puzzels[i]->id), Point(maxCols * k + 10, maxRows * currentRow + 10), HersheyFonts::FONT_HERSHEY_PLAIN, 1.0, textColor);
		}
		currentRow++;
	}
	return mosaic;
}

Mat ComposePuzzels(vector<PuzzelRectange*>& puzzels)
{
	return ComposePuzzels<Vec3b>(puzzels, [](PuzzelRectange* p) {return p->puzzelArea; }, Scalar(0, 0, 0));
}

Mat ComposeBackgroundEdges(vector<PuzzelRectange*>& puzzels)
{
	return ComposePuzzels<unsigned char>(puzzels, [](PuzzelRectange* p) {return p->backgroundEdges; }, Scalar(255));
}

Mat ComposeEdges(vector<PuzzelRectange*>& puzzels)
{
	return ComposePuzzels<unsigned char>(puzzels, [](PuzzelRectange* p) {return p->edges; }, Scalar(255));
}

void run()
{
	puzzelDetector = new PuzzelDetector(src);
	puzzelDetector->cannEdgeThresh = 50;
	auto puzzels = puzzelDetector->DetectPuzzels();

	for(PuzzelRectange* puzzel : puzzels)
	{
		puzzel->ComputeEdgeFeatures();
		puzzel->PrintScores();
		puzzel->MarkEdgesOnOriginImage(src);
		puzzel->MarkJointsOnOriginImage(src);
	}

	imshow("corners", src);
	int puzzelNr = 4;
	puzzels[puzzelNr]->FindNeighbour(puzzels, 0, "w0");
	puzzels[puzzelNr]->FindNeighbour(puzzels, 1, "w1");
	puzzels[puzzelNr]->FindNeighbour(puzzels, 2, "w2");
	puzzels[puzzelNr]->FindNeighbour(puzzels, 3, "w3");

	imshow("mosaic - puzzels", ComposePuzzels(puzzels));
	imshow("mosaic - background edges", ComposeBackgroundEdges(puzzels));
	imshow("mosaic - edges", ComposeEdges(puzzels));
	waitKey(1);
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

	const char* source_window = "Source";
	namedWindow(source_window);
	imshow(source_window, src);
	run();
	waitKey();
	return 0;
}
