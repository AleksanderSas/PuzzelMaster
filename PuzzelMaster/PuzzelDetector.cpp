#include "PuzzelDetector.h"
#include "IntrestingArea.h"

PuzzelDetector::PuzzelDetector(Mat& input) : image(input), knn(input.cols, input.rows, MIN_SQUARE_DISTANCE, INIT_GRID_DENSITY)
{
	cvtColor(input, image_gray, COLOR_BGR2GRAY);
	blur(image_gray, image_gray, Size(3, 3));
}

struct JointCandidate
{
	int hits, miss;
};

int minJointLen = 6;
int P(Mat& edges, Point2f& p1, Point2f& p2, Point2f& current)
{
	int nonePuzzelLen = 0;
	int puzzelPoints = 0;

	ProcessLine(p1, p2, [&](int x, int y)
		{
			if (edges.at<char>(y, x) == 0) //NOT puzzel part
			{
				puzzelPoints = 0;
				if (++nonePuzzelLen > 1)
				{
					if (nonePuzzelLen > minJointLen)
					{
						return false;
					}
				}
				else
				{
					current.x = x;
					current.y = y;
				}
			}
			else  //puzzel part
			{
				if (puzzelPoints++ > 5)
				{
					nonePuzzelLen = 0;
				}
			}
			return true;
		});
	return nonePuzzelLen;
}

void DetectJoint(Mat &puzzel, Mat &edges, Point2f &p1, Point2f &p2)
{
	int nonePuzzelLen = 0;
	Point2f j1, j2;
	Point2f& current = j1;;
	
	nonePuzzelLen = P(edges, p1, p2, j1);
	if (nonePuzzelLen >= minJointLen)
	{
		drawMarker(puzzel, j1, Scalar(150, 10, 150));
	}

	nonePuzzelLen = P(edges, p2, p1, j2);
	if (nonePuzzelLen >= minJointLen)
	{
		drawMarker(puzzel, j2, Scalar(150, 10, 150));
	}
}

bool vComparer(PuzzelRectange *i, PuzzelRectange *j) { return (j->score < i->score); };

void ProcessOnePuzzel(IntrestingArea &ia, const char* name, KMeans& km, vector<PuzzelRectange> &returnBuffer)
{
	Mat greyMat, xDeriv, yDeriv, score;
	cv::cvtColor(ia.AreaImage, greyMat, CV_BGR2GRAY);

	int ddepth = CV_16S;
	/// Gradient X
	Sobel(greyMat, xDeriv, ddepth, 1, 0, 3);
	/// Gradient Y
	Sobel(greyMat, yDeriv, ddepth, 0, 1, 3);

	double k = 0.04;

	int maxCorners =  35;
	vector<Point2f> corners;
	double qualityLevel = 0.004;
	double minDistance = 8;
	int blockSize = 3, gradientSize = 3;
	bool useHarrisDetector = false;

	goodFeaturesToTrack(greyMat,
		corners,
		maxCorners,
		qualityLevel,
		minDistance,
		Mat(),
		blockSize,
		gradientSize,
		useHarrisDetector,
		k);
	cout << "** Number of corners detected: " << corners.size() << endl;

	/*for (auto it = corners.begin(); it != corners.end(); it++)
	{
		drawMarker(ia.AreaImage, *it, Scalar(123, 231, 90));
	}*/

	auto result = km.FindBestRectange(corners, xDeriv, yDeriv, ia);
	
	sort(result->begin(), result->end(), vComparer);
	
	if (result->size() > 0)
	{
		//for (auto p = corners.begin(); p != corners.end(); p++)
		//{
		//	drawMarker(puzzel, *p, Scalar(60, 160, 30));
		//}
		PuzzelRectange bestRectangle = *(result->at(0));
		/*DetectJoint(ia.AreaImage, ia.EdgeMap, bestRectangle.left, bestRectangle.upper);
		DetectJoint(ia.AreaImage, ia.EdgeMap, bestRectangle.left, bestRectangle.lower);
		DetectJoint(ia.AreaImage, ia.EdgeMap, bestRectangle.right, bestRectangle.upper);
		DetectJoint(ia.AreaImage, ia.EdgeMap, bestRectangle.right, bestRectangle.lower);
		imshow(name, ia.AreaImage);*/

		bestRectangle.puzzelArea = ia.AreaImage;
		bestRectangle.contours = ia.contours;
		returnBuffer.push_back(bestRectangle);
	}
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

	imshow("Contours", contourDrawing);
}

vector<PuzzelRectange> PuzzelDetector::DetectPuzzels()
{
	Mat canny_output;
	Canny(image_gray, canny_output, cannEdgeThresh, cannEdgeThresh * 2);

	Mat canny2;
	Mat element = getStructuringElement(MORPH_RECT,
		Size(2 * dilation_size + 1, 2 * dilation_size + 1),
		Point(dilation_size, dilation_size));
	dilate(canny_output, canny2, element);

	imshow("canny", canny_output);
	vector<vector<Point> > *contours = new vector<vector<Point> >(); //TODO <<<<<<<<<<<<<<<<<<<<<<<<
	vector<Vec4i> hierarchy;
	findContours(canny_output, *contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);

	DrawContours(*contours, hierarchy);
	waitKey(1);

	Mat edgeMap = ComputeEdgeMap(*contours);

	int change = 1;
	while (change > 0)
	{
		Mat contoursWithAreas = contourDrawing.clone();
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

	vector<IntrestingArea> puzzelAreas = knn.GetPuzzels(image, edgeMap);
	vector<PuzzelRectange> puzzels;
	
	for (int i = 0; i < puzzelAreas.size(); i++)
	{
		string name = string("P") + to_string(i);
		ProcessOnePuzzel(puzzelAreas[i], name.c_str(), knn, puzzels);
		//imshow(name.c_str(), puzzelAreas[i].first);
	}

	return puzzels;
}