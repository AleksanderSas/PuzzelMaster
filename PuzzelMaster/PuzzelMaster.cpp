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

Mat ComposePuzzels(vector<PuzzelRectange*>& puzzels)
{
	int maxCols = 0;
	int maxRows = 0;

	for (PuzzelRectange* puzzel : puzzels)
	{
		if (maxCols < puzzel->puzzelArea.cols)
		{
			maxCols = puzzel->puzzelArea.cols;
		}
		if (maxRows < puzzel->puzzelArea.rows)
		{
			maxRows = puzzel->puzzelArea.rows;
		}
	}

	int puzzelsPerRow = ceil(sqrt(puzzels.size()));
	int rowNumber = ceil(1.0 * puzzels.size() / puzzelsPerRow);
	Mat mosaic = Mat::zeros(maxRows * rowNumber, maxCols * puzzelsPerRow, puzzels[0]->puzzelArea.type());

	int i = 0;
	int currentRow = 0;
	Scalar textColor(0, 0, 0);
	while (i < puzzels.size())
	{
		for (int k = 0; k < puzzelsPerRow && i < puzzels.size(); k++, i++)
		{
			Mat source = puzzels[i]->puzzelArea;
			for (int y = 0; y < source.rows; y++)
			{
				for (int x = 0; x < source.cols; x++)
				{
					mosaic.at<Vec3b>(y + maxRows * currentRow, x + maxCols * k) = source.at<Vec3b>(y, x);
				}
			}
			putText(mosaic, to_string(puzzels[i]->id), Point(maxCols * k + 10, maxRows * currentRow + 10), HersheyFonts::FONT_HERSHEY_PLAIN, 1.0, textColor);
		}
		currentRow++;
	}
	return mosaic;
}

void run()
{
	puzzelDetector = new PuzzelDetector(src);
	puzzelDetector->cannEdgeThresh = 50;
	auto puzzels = puzzelDetector->DetectPuzzels();

	for(PuzzelRectange* puzzel : puzzels)
	{
		//puzzel->ReconstructBorder();
		//puzzel->computeBackgroundColor();
		puzzel->ComputeEdgeFeatures();
		puzzel->PrintScores();
		puzzel->MarkEdgesOnOriginImage(src);
	}
	imshow("gfdgdfgd", src);
	int puzzelNr = 4;
	puzzels[puzzelNr]->FindNeighbour(puzzels, 0, "w0");
	puzzels[puzzelNr]->FindNeighbour(puzzels, 1, "w1");
	puzzels[puzzelNr]->FindNeighbour(puzzels, 2, "w2");
	puzzels[puzzelNr]->FindNeighbour(puzzels, 3, "w3");
	//imshow("source winner", puzzels[1].puzzelArea);

	imshow("mosaic", ComposePuzzels(puzzels));
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
