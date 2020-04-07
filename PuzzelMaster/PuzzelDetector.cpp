#include "PuzzelDetector.h"
#include "IntrestingArea.h"
#include "LineProcessor.h"
#include "DebugFlags.h"
#include "Presenter.h"

PuzzelDetector::PuzzelDetector(Mat& input, int expectedPuzzelSize) : 
	image(input), 
	knn(input.cols, input.rows, MIN_PUZZEL_SIZE),
	expectedPuzzelSize(expectedPuzzelSize)
{
	cvtColor(input, image_gray, COLOR_BGR2GRAY);
	blur(image_gray, image_gray, Size(3, 3));
}

PuzzelDetector* PuzzelDetector::Create(char* input, int expectedPuzzelSize, int cannEdgeThresh)
{
	Mat img = imread(input);
	PuzzelDetector* detector = new PuzzelDetector(img, expectedPuzzelSize);
	detector->cannEdgeThresh = cannEdgeThresh;
	return detector;
}

Mat PuzzelDetector::ComputeEdgeMap(vector<vector<Point>> &contours)
{
	Mat edgeMap = Mat::zeros(image.size(), CV_8U);
	Mat edgeMap2 = Mat::zeros(image.size(), CV_8U);

	for (vector<vector<Point> >::iterator contour = contours.begin(); contour != contours.end(); contour++)
	{
		for (vector<Point>::iterator point = contour->begin(); point != contour->end(); point++)
		{
			edgeMap.at<uchar>(*point) = 255;
		}
	}

	Mat element = getStructuringElement(MORPH_RECT,
		Size(2 * dilation_size + 1, 2 * dilation_size + 1),
		Point(dilation_size, dilation_size));
	dilate(edgeMap, edgeMap2, element);
	return edgeMap2;
}

void PuzzelDetector::DrawContours(vector<vector<Point> > &contours, vector<Vec4i> &hierarchy)
{
	contourDrawing = Mat::zeros(image.size(), CV_8UC3);
	for (size_t i = 0; i < contours.size(); i++)
	{
		if (contours[i].size() > 50)
		{
			Scalar color = Scalar(220, 40, 180);
			drawContours(contourDrawing, contours, (int)i, color, 1, LINE_8, hierarchy, 0);
		}
		else
		{
			Scalar color = Scalar(20, 140, 20);
			drawContours(contourDrawing, contours, (int)i, color, 1, LINE_8, hierarchy, 0);
		}
	}

	Presenter::ShowScaledImage("Contours", contourDrawing);
	waitKey(1);
}

vector<PuzzelRectange*> PuzzelDetector::DetectPuzzels()
{
	Mat canny_output;
	Canny(image_gray, canny_output, cannEdgeThresh, cannEdgeThresh * 2);

	Mat canny2;
	Mat element = getStructuringElement(MORPH_RECT,
		Size(2 * dilation_size + 1, 2 * dilation_size + 1),
		Point(dilation_size, dilation_size));
	dilate(canny_output, canny2, element);

	RemoveTooLongLines(canny_output);

	//TODO: delete
	vector<vector<Point> > *contours = new vector<vector<Point>>(); 
	vector<Vec4i> hierarchy;
	findContours(canny_output, *contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);

	DrawContours(*contours, hierarchy);

	Mat edgeMap = ComputeEdgeMap(*contours);

	int change = 1;
	Mat contoursWithAreas;
	while (change > 0)
	{
		contoursWithAreas = contourDrawing.clone();
		change = knn.Pass(contours);
		knn.DrawBoxes(contoursWithAreas);

		Scalar color = Scalar(120, 140, 180);
		for (int i = 0; i < knn.centres.size(); i++)
		{
			drawMarker(contoursWithAreas, knn.centres[i]->Point, color, 0, 30, 2);
		}

		imshow("Contours", contoursWithAreas);
		waitKey(1);
	}
	Presenter::ShowScaledImage("Contours", contoursWithAreas);

	vector<IntrestingArea> puzzelAreas = knn.GetPuzzels(image, edgeMap);

	std::vector<Rect> anotherMyObjectList;

	for (IntrestingArea& ia : puzzelAreas)
	{
		anotherMyObjectList.push_back(ia.OriginRectange);
	}

	//TODO: delete
	BackgroundSeparator* separator = new BackgroundSeparator(image, anotherMyObjectList, edgeMap);

	return AddPuzzelsToList(puzzelAreas, separator);
}

vector<PuzzelRectange*> PuzzelDetector::AddPuzzelsToList(std::vector<IntrestingArea>& puzzelAreas, BackgroundSeparator*& separator)
{
	vector<PuzzelRectange*> puzzels;
	long long int time = clock();
	unsigned int idSequence = 0;

#if USE_MULTITHREADING_IN_PUZZEL_DETECTION
	auto it = puzzelAreas.begin();
	while (it != puzzelAreas.end())
	{
		PuzzelRectange* puzzel1 = nullptr;
		PuzzelRectange* puzzel3 = nullptr;
		std::thread t1(&IntrestingArea::findPuzzel2, it._Ptr, separator, idSequence, MIN_PUZZEL_SIZE, &puzzel1);
		std::thread* t2 = nullptr;
		if (++it != puzzelAreas.end())
		{
			idSequence += 100000000;
			t2 = new std::thread(&IntrestingArea::findPuzzel2, it._Ptr, separator, idSequence, MIN_PUZZEL_SIZE, &puzzel3);
		}
		if (++it != puzzelAreas.end())
		{
			idSequence += 100000000;
			PuzzelRectange* puzzel2 = it->findPuzzel(separator, idSequence, MIN_PUZZEL_SIZE);
			if (puzzel2 != nullptr)
				puzzels.push_back(puzzel2);
			it++;
		}

		t1.join();
		if (puzzel1 != nullptr)
			puzzels.push_back(puzzel1);

		if (t2 != nullptr)
		{
			t2->join();
			if (puzzel3 != nullptr)
				puzzels.push_back(puzzel3);
		}

	}
#else
	for (IntrestingArea& ia : puzzelAreas)
	{
		PuzzelRectange* puzzel = ia.findPuzzel(separator, idSequence, MIN_PUZZEL_SIZE);
		if (puzzel != nullptr)
			puzzels.push_back(puzzel);
	}
#endif
	time = (clock() - time) * 1000 / CLOCKS_PER_SEC;
	cout << "Puzzel detection time: " << time << " [ms]" << endl;
	return puzzels;
}

void PuzzelDetector::RemoveTooLongLines(cv::Mat& canny_output)
{
	vector<Vec4i> lines;
	Mat edges = Mat::zeros(canny_output.rows, canny_output.cols, CV_8UC1);
	HoughLinesP(canny_output, lines, 1, 0.01, MIN_LINE_LEN_TO_REMOVE, MIN_LINE_LEN_TO_REMOVE, 5);
	for (size_t i = 0; i < lines.size(); i++)
	{
		Vec4i l = lines[i];
		line(canny_output, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0), 2, LINE_AA);
		line(edges, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255), 2, LINE_AA);
	}

#if DRAW_TOO_LONG_LINES
	Presenter::ShowScaledImage("too long lines", edges);
#endif
}
