#include "windows.h"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <string>
#include "time.h"
#include "PuzzelDetector.h"
#include "KMeans.h"
#include "PuzzelSolver.h"
#include "math.h"
#include "Presenter.h"
#include "DebugFlags.h"
#include <time.h>

using namespace cv;
using namespace std;

Mat src;
PuzzelDetector* puzzelDetector;

// D:\puzzle\2\test_2_2.jpg
// D:\puzzle\2\test_2_2_mini.jpg
// D:\puzzle\test_3.jpg
// D:\puzzle\test.jpg
// D:\puzzle\3\test_3_1.jpg
// D:\puzzle\3\test_3_3.jpg
// D:\puzzle\4\test_1.jpg

template<typename T, typename S>
Mat ComposePuzzels(vector<S*>& puzzels, function<Mat(S*)> selector, function<string(S*)> label, Scalar textColor)
{
	return ComposePuzzels<T, S>(puzzels, selector, label, textColor, ceil(sqrt(puzzels.size())));
}

void SetOffsets(int* offsets, int size)
{
	offsets[0] = 0;
	for (int i = 1; i <= size; i++)
	{
		offsets[i] = offsets[i - 1] + offsets[i] + 5;
	}
}

template<typename S>
void SetRowsAndColsSizes(vector<S*>& puzzels, function<Mat(S*)> selector, int* &columnsOffsets, int* &rowOffsets, int rows, int cols)
{
	columnsOffsets = new int[cols + 1];
	memset(columnsOffsets, 0, sizeof(int) * (cols + 1));
	rowOffsets = new int[rows + 1];
	memset(rowOffsets, 0, sizeof(int) * (rows + 1));

	for (int y = 0; y < rows; y++)
	{
		for (int x = 0; x < cols; x++)
		{
			int idx = y * cols + x;
			if (idx >= puzzels.size())
			{
				SetOffsets(columnsOffsets, cols);
				SetOffsets(rowOffsets, rows);
				return;
			}
			Mat img = selector(puzzels[idx]);
			if (columnsOffsets[x + 1] < img.cols)
				columnsOffsets[x + 1] = img.cols;
			if (rowOffsets[y + 1] < img.rows)
				rowOffsets[y + 1] = img.rows;
		}
	}

	SetOffsets(columnsOffsets, cols);
	SetOffsets(rowOffsets, rows);
}

template<typename T, typename S>
Mat ComposePuzzels(vector<S*>& puzzels, function<Mat(S*)> selector, function<string(S*)> label, Scalar textColor, int puzzelsPerRow)
{
	int rowNumber = ceil(1.0 * puzzels.size() / puzzelsPerRow);
	int* columnsOffsets;
	int* rowOffsets;
	
	SetRowsAndColsSizes<S>(puzzels, selector, columnsOffsets, rowOffsets, rowNumber, puzzelsPerRow);
	
	Mat mosaic = Mat::zeros(rowOffsets[rowNumber], columnsOffsets[puzzelsPerRow], selector(puzzels[0]).type());
	
	int i = 0;
	int currentRow = 0;
	while (i < puzzels.size())
	{
		for (int k = 0; k < puzzelsPerRow && i < puzzels.size(); k++, i++)
		{
			Mat source = selector(puzzels[i]);
			int columnOffset = columnsOffsets[k];
			int rowOffset = rowOffsets[currentRow];
			for (int y = 0; y < source.rows; y++)
			{
				for (int x = 0; x < source.cols; x++)
				{
					mosaic.at<T>(y + rowOffset, x + columnOffset) = source.at<T>(y, x);
				}
			}
			putText(mosaic, label(puzzels[i]), Point(columnOffset + 10, rowOffset + 20), HersheyFonts::FONT_HERSHEY_PLAIN, 1.5, textColor);
		}
		currentRow++;
	}
	
	delete[] rowOffsets;
	delete[] columnsOffsets;

	return mosaic;
}

Mat ComposePuzzels(vector<PuzzelRectange*>& puzzels)
{
	return ComposePuzzels<Vec3b, PuzzelRectange>(puzzels, [](PuzzelRectange* p) {return p->puzzelArea; }, [](PuzzelRectange* p) {return to_string(p->id); }, Scalar(0, 0, 255));
}

Mat ComposeBackgroundEdges(vector<PuzzelRectange*>& puzzels)
{
	return ComposePuzzels<unsigned char, PuzzelRectange>(puzzels, [](PuzzelRectange* p) {return p->backgroundEdges; }, [](PuzzelRectange* p) {return to_string(p->id); }, Scalar(255));
}

Mat ComposeEdges(vector<PuzzelRectange*>& puzzels)
{
	return ComposePuzzels<unsigned char, PuzzelRectange>(puzzels, [](PuzzelRectange* p) {return p->edges; }, [](PuzzelRectange* p) {return to_string(p->id); }, Scalar(255));
}

Mat ComposeSelectedEdges(Token* lastToken, int puzzelsPerRow)
{
	char buffer[255];
	sprintf_s(buffer, "score:  %d", lastToken->score);

	vector<Token*> tokens;
	while (lastToken != nullptr)
	{
		tokens.insert(tokens.begin(), lastToken);
		lastToken = lastToken->previous.get();
	}
	auto selector = [](Token* t) {return t->puzzel->ExtractPuzzelAndRotateEdgeToUp((t->pozzelRotation + 1) % 4, 20); };
	Mat mosaic = ComposePuzzels<Vec3b, Token>(tokens, selector, [](Token* t) {return to_string(t->puzzel->id); }, Scalar(0, 255, 0), puzzelsPerRow);
	putText(mosaic, buffer, Point(5, mosaic.rows - 12), HersheyFonts::FONT_HERSHEY_PLAIN, 1.5, Scalar(0, 0, 255), 2);
	return mosaic;
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

#if ENABLE_SOLVER
	auto solver = PuzzelSolver();
	int columnsToSolve = 4;
	long long int time = clock();
	solver.Solve(puzzels, columnsToSolve, 3);
	time = clock() - time;
	cout << "TIME: " << time << endl;
	solver.RemoveDuplicateds();

	cout << "SOLUTIONS: " << solver.Size() << endl;
	for (int i = 0; i < min(5, solver.Size()); i++)
	{
		string label = string("mosaic - solved ") + to_string(i);
		Presenter::ShowScaledImage(label.c_str(), ComposeSelectedEdges(solver.GetBest(i), columnsToSolve));
	}

	cout << "\n1-ST:\n";
	solver.PrintHistory(0);
	cout << "\n2-ND:\n";
	solver.PrintHistory(1);
	cout << "\n3-RD:\n";
	solver.PrintHistory(2);
	waitKey(1);
#endif

	Presenter::ShowScaledImage("corners", src);
#if MATCH_PUZZEL
	int puzzelNr = 11;
	puzzels[puzzelNr]->FindNeighbour(puzzels, 0, "w0");
	puzzels[puzzelNr]->FindNeighbour(puzzels, 1, "w1");
	puzzels[puzzelNr]->FindNeighbour(puzzels, 2, "w2");
	puzzels[puzzelNr]->FindNeighbour(puzzels, 3, "w3");
#endif
	if (puzzels.size() > 0)
	{
		Presenter::ShowScaledImage("mosaic - puzzels", ComposePuzzels(puzzels));
		Presenter::ShowScaledImage("mosaic - background edges", ComposeBackgroundEdges(puzzels));
		Presenter::ShowScaledImage("mosaic - edges", ComposeEdges(puzzels));
	}
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
	Presenter::ShowScaledImage(source_window, src);
	run();
	cout << "FINISHED";
	waitKey();
	return 0;
}
