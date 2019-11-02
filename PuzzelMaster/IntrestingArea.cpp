#include "IntrestingArea.h"
#include "LineProcessor.h"

IntrestingArea:: IntrestingArea(Mat areaImage, Mat edgeMap, Rect originRectange,int id):
	AreaImage(areaImage), 
	EdgeMap(edgeMap), 
	OriginRectange(originRectange),
	id(id)
{}

PuzzelRectange* IntrestingArea::findPuzzel(BackgroundSeparator* separator)
{
	Mat greyMat, xDeriv, yDeriv, score;
	cv::cvtColor(AreaImage, greyMat, CV_BGR2GRAY);

	int ddepth = CV_16S;
	/// Gradient X
	Sobel(greyMat, xDeriv, ddepth, 1, 0, 3);
	/// Gradient Y
	Sobel(greyMat, yDeriv, ddepth, 0, 1, 3);

	int maxCorners = 35;
	vector<Point2f> corners;
	double qualityLevel = 0.004;
	double minDistance = 8;

	goodFeaturesToTrack(greyMat,
		corners,
		maxCorners,
		qualityLevel,
		minDistance);
	cout << "** Number of corners detected: " << corners.size() << endl;

	/*for (auto it = corners.begin(); it != corners.end(); it++)
	{
		drawMarker(AreaImage, *it, Scalar(123, 231, 90));
	}*/

	auto result = FindBestRectange(corners, xDeriv, yDeriv, separator);
	if (result != nullptr)
	{
		//for (auto p = corners.begin(); p != corners.end(); p++)
		//{
		//	drawMarker(puzzel, *p, Scalar(60, 160, 30));
		//}

		result->puzzelArea = AreaImage;
		return result;
	}
	return nullptr;
}

static bool hComparer(Point2f i, Point2f j) { return (i.x < j.x); }

static float crossProduct(Point2f& a, Point2f& centre, Point2f& b)
{
	int x1 = centre.x = a.x;
	int y1 = centre.y = a.y;

	int x2 = centre.x = b.x;
	int y2 = centre.y = b.y;

	return abs(x1 * y2 - x2 * y1);
}

static double decide(Point2f& v1, Point2f& v2, double treshold = MIN_CROSS_PROD)
{
	double result = abs(v1.cross(v2));
	return result >= treshold ? result : 0.0;
}

static float GetHarrisScore(Point2f& p, Mat& xDeriv, Mat& yDeriv, float k = 0.04)
{
	float dx = abs(xDeriv.at<short>(p));
	float dy = abs(yDeriv.at<short>(p));
	return 1.0 * dx * dy - k * (dx - dy) * (dx - dy);
}

static Point2f GetVector(Point2f& a, Point2f& b)
{
	auto x = a.x - b.x;
	auto y = a.y - b.y;
	return Point2f(x, y);
}

static Point2f GetNormalizedVector(Point2f& a, Point2f& b)
{
	auto x = a.x - b.x;
	auto y = a.y - b.y;
	double len = hypot(x, y);
	y /= len;
	x /= len;
	return Point2f(x, y);
}

static float IsMoreOrLessRectange(PuzzelRectange* candidate, Mat& xDeriv, Mat& yDeriv)
{
	Point2f v1 = GetNormalizedVector(candidate->left, candidate->upper);
	Point2f v2 = GetNormalizedVector(candidate->upper, candidate->right);
	Point2f v3 = GetNormalizedVector(candidate->right, candidate->lower);
	Point2f v4 = GetNormalizedVector(candidate->lower, candidate->left);

	double rectangularityMeasure = decide(v1, v2) * decide(v2, v3) * decide(v3, v4) * decide(v4, v1);
	candidate->recScore = rectangularityMeasure;
	double harrisMeasure = GetHarrisScore(candidate->left, xDeriv, yDeriv)
		* GetHarrisScore(candidate->lower, xDeriv, yDeriv)
		* GetHarrisScore(candidate->right, xDeriv, yDeriv)
		* GetHarrisScore(candidate->upper, xDeriv, yDeriv);
	harrisMeasure = sqrt(sqrt(sqrt(harrisMeasure)));
	candidate->interestScore = harrisMeasure;

	if (rectangularityMeasure > 0.1)
	{
		Point2f v1 = GetVector(candidate->left, candidate->upper);
		Point2f v2 = GetVector(candidate->upper, candidate->right);

		int a = v1.x * v1.x + v1.y * v1.y;
		int b = v2.x * v2.x + v2.y * v2.y;

		int rectangleFactor = 2;
		if (1.0 * a / b > rectangleFactor || 1.0 * b / a > rectangleFactor) //it not rectange
			return 0.0;

		double area = sqrt(sqrt(a * b));
		candidate->areaScore = area;
		return area * rectangularityMeasure;
	}

	return 0.0;
}

static void UpdateHitMetric(Mat& edgeMap, int xPos, int yPos, int& hit, int& miss)
{
	if (edgeMap.at<uchar>(Point(xPos, yPos)))
	{
		hit++;
	}
	else
	{
		miss++;
	}
}

static void ComputeEdgeHits(Mat& edgeMap, PuzzelRectange* puzzel, int& hit, int& miss)
{
	auto counter = [&](int x, int y) {UpdateHitMetric(edgeMap, x, y, hit, miss); return true; };

	LineProcessor::Process(puzzel->left, puzzel->lower, counter);
	LineProcessor::Process(puzzel->left, puzzel->upper, counter);
	LineProcessor::Process(puzzel->right, puzzel->lower, counter);
	LineProcessor::Process(puzzel->right, puzzel->upper, counter);
}

PuzzelRectange* IntrestingArea::FindBestRectange(vector<Point2f>& corners, Mat& xDeriv, Mat& yDeriv, BackgroundSeparator *separator)
{
	auto hSorted = vector<Point2f>(corners);
	sort(hSorted.begin(), hSorted.end(), hComparer);

	int counter = 0;
	PuzzelRectange* bestRects = nullptr;

	for (vector<Point2f>::iterator left = hSorted.begin(); left != hSorted.end(); left++)
	{
		for (vector<Point2f>::iterator right = left + 1; right != hSorted.end(); right++)
		{
			int dx = left->x - right->x;
			int dy = left->y - right->y;
			if (dx * dx + dy + dy < MIN_RECT_DIAGONAL)
			{
				continue;
			}
			for (vector<Point2f>::iterator upper = left + 1; upper != right; upper++)
			{
				if (right->x == upper->x && right->y == upper->y)
				{
					continue;
				}
				Point2f v1 = GetNormalizedVector(*left, *upper);
				Point2f v2 = GetNormalizedVector(*upper, *right);

				double measure = decide(v1, v2);
				if (measure < MIN_CROSS_PROD)
				{
					continue;
				}

				for (vector<Point2f>::iterator lower = left + 1; lower != right; lower++)
				{
					if (lower == upper || lower->y > upper->y)
					{
						continue;
					}

					PuzzelRectange* candidate = new PuzzelRectange(*left, *right, *lower, *upper, counter++, separator);
					candidate->puzzelArea = AreaImage;

					double totalScore = IsMoreOrLessRectange(candidate, xDeriv, yDeriv);
					if (totalScore > 0.0)
					{
						float noneBackgroundScore = candidate->scoreArea(separator);
						//totalScore *= noneBackgroundScore;
						int hit = 0;
						int miss = 0;
						ComputeEdgeHits(EdgeMap, candidate, hit, miss);
						if (hit + miss == 0)
						{
							continue;
						}
						candidate->hitScore = 1.0 * hit / (hit + miss);
						//candidate.hitScore *= candidate.hitScore;
						if (candidate->hitScore < 0.5)
						{
							continue;
						}
						totalScore *= candidate->hitScore;
						candidate->score = totalScore;
						if (bestRects == nullptr || candidate->score > bestRects->score)
						{
							if (bestRects != nullptr)
							{
								delete bestRects;
							}
							bestRects = candidate;
						}
					}
				}
			}
		}
	}
	return bestRects;
}