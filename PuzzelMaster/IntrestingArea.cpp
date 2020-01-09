#include "IntrestingArea.h"
#include "LineProcessor.h"
#include "DebugFlags.h"
#include "Utils.h"
#include "MathUtils.h"

IntrestingArea:: IntrestingArea(Mat areaImage, Mat edgeMap, Rect originRectange,int id, RotatedRect box):
	AreaImage(areaImage), 
	EdgeMap(edgeMap), 
	OriginRectange(originRectange),
	id(id),
	box(box)
{}

static Mat getBackgroundMap(Mat& input, BackgroundSeparator* separator)
{
	Mat output = Mat::zeros(input.rows, input.cols, CV_8UC1);
	int count = 0;
	float score = 0.0;
	Vec3b* p_source;
	unsigned char* p_dest;
	for (int i = 0; i < input.rows; ++i)
	{
		p_source = input.ptr<Vec3b>(i);
		p_dest = output.ptr<unsigned char>(i);
		for (int j = 0; j < input.cols; ++j, ++p_source, ++p_dest)
		{
			float nondebackgroundProbability = (1 - separator->scorePoint(p_source));
			if (!isnan(nondebackgroundProbability))
			{
				*p_dest = nondebackgroundProbability * 255;
			}
		}
	}
	return output;
}

static vector<Point2f> Merge(vector<Point2f> set1, vector<Point2f> set2, int minDisatnce)
{
	vector<Point2f> result(set1);

	for (Point c2 : set2)
	{
		bool append = true;
		for (Point c1 : set1)
		{
			if (squareDist(c1, c2) < minDisatnce * minDisatnce)
			{
				append = false;
				continue;
			}
		}
		if(append)
			result.push_back(c2);
	}
	return result;
}

PuzzelRectange* IntrestingArea::findPuzzel(BackgroundSeparator* separator, unsigned int& idSequence, int minPuzzelSize)
{
	Mat greyMat, score;
	cv::cvtColor(AreaImage, greyMat, CV_BGR2GRAY);

	int maxCorners = 45;
	vector<Point2f> corners;
	double qualityLevel = 0.004;
	double minDistance = 8;

	goodFeaturesToTrack(greyMat,
		corners,
		maxCorners,
		qualityLevel,
		minDistance);

	Mat b = getBackgroundMap(AreaImage, separator);


#if DRAW_CORNERS
	for (auto c : corners)
	{
		drawMarker(AreaImage, c, Scalar(123, 231, 90));
	}
#endif

#if USE_INTRESTING_POINTS_FROM_BACKGROUND
	int maxCorners2 = 25;
	vector<Point2f> corners2;
	double qualityLevel2 = 0.004;

	goodFeaturesToTrack(b,
		corners2,
		maxCorners2,
		qualityLevel2,
		minDistance);
#if DRAW_CORNERS
	for (auto c : corners2)
	{
		circle(AreaImage, c, 2, Scalar(90, 30, 200), 3);
	}
#endif
	corners = Merge(corners, corners2, minDistance);
#endif

#if DRAW_CORNERS
	imshow(string("conrners_") + to_string(id), AreaImage);
	return nullptr;
#endif
	cout << "** nr: " << id << "  Number of corners detected: " << corners.size() << endl;

	auto result = FindBestRectange(corners, separator, idSequence, minPuzzelSize);
	if (result != nullptr)
	{
		result->puzzelArea = AreaImage;
		return result;
	}
	return nullptr;
}


static Mat getEdgeMapFromBackground(Mat& map)
{
	Mat image_gray, canny_output;
	int cannEdgeThresh = 300;
	Mat element = getStructuringElement(MORPH_RECT,
		Size(2, 2),
		Point(1, 1));
	blur(map, map, Size(3, 3));
	Canny(map, canny_output, cannEdgeThresh, cannEdgeThresh * 2);

	dilate(canny_output, canny_output, element);

	vector<Vec4i> lines;
	Mat edges = Mat::zeros(canny_output.rows, canny_output.cols, CV_8UC1);
	HoughLinesP(canny_output, lines, 1, 0.01, 15, 17, 4);
	for (size_t i = 0; i < lines.size(); i++)
	{
		Vec4i l = lines[i];
		line(map, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(125), 5, CV_AA);
		line(edges, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255), 5, CV_AA);
	}
	return edges;
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

static float IsMoreOrLessRectange(PuzzelRectange* candidate)
{
	Point2f v1 = GetNormalizedVector(candidate->left, candidate->upper);
	Point2f v2 = GetNormalizedVector(candidate->upper, candidate->right);
	Point2f v3 = GetNormalizedVector(candidate->right, candidate->lower);
	Point2f v4 = GetNormalizedVector(candidate->lower, candidate->left);

	double rectangularityMeasure = decide(v1, v2) * decide(v2, v3) * decide(v3, v4) * decide(v4, v1);
	candidate->recScore = rectangularityMeasure;

	if (rectangularityMeasure > 0.1)
	{
		Point2f v1 = GetVector(candidate->left, candidate->upper);
		Point2f v2 = GetVector(candidate->upper, candidate->right);

		int a = v1.x * v1.x + v1.y * v1.y;
		int b = v2.x * v2.x + v2.y * v2.y;

		int rectangleFactor = 2;
		if (1.0 * a / b > rectangleFactor || 1.0 * b / a > rectangleFactor) //it not rectange
			return 0.0;

		double area = sqrt(a * b);
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

static double ComputeHitScore(Mat& edges, PuzzelRectange* candidate)
{
	int hit = 0;
	int miss = 0;
	ComputeEdgeHits(edges, candidate, hit, miss);
	double score = 0.0;
	if (hit + miss > 0)
	{
		score = 1.0 * hit / (hit + miss);
	}
	return score;
}

PuzzelRectange* IntrestingArea::FindBestRectange(vector<Point2f>& corners, BackgroundSeparator *separator, unsigned int& idSequence, int minPuzzelSize)
{
	auto hSorted = vector<Point2f>(corners);
	sort(hSorted.begin(), hSorted.end(), hComparer);

	PuzzelRectange* bestRects = nullptr;
	Mat b = getBackgroundMap(AreaImage, separator);
	Mat edg = getEdgeMapFromBackground(b);
	int minRectDiagonal = minPuzzelSize * minPuzzelSize * 2;

	for (vector<Point2f>::iterator left = hSorted.begin(); left != hSorted.end(); left++)
	{
		for (vector<Point2f>::iterator right = left + 1; right != hSorted.end(); right++)
		{
			int dx = left->x - right->x;
			int dy = left->y - right->y;
			if (dx * dx + dy + dy < minRectDiagonal)
			{
				continue;
			}
			//TODO nie trzeba przegladac az do konca, tylko pewien obszar za right
			for (vector<Point2f>::iterator upper = left + 1; upper != hSorted.end(); upper++)
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

				//TODO nie trzeba przegladac az do konca, tylko pewien obszar za right
				for (vector<Point2f>::iterator lower = left + 1; lower != hSorted.end(); lower++)
				{
					if (lower == upper || lower->y > upper->y)
					{
						continue;
					}

					PuzzelRectange* candidate = new PuzzelRectange(*left, *right, *lower, *upper, idSequence++, separator, box);
					candidate->puzzelArea = AreaImage;
					candidate->backgroundEdges = edg;
					candidate->backgroundMap = b;

					double totalScore = IsMoreOrLessRectange(candidate);
					if (totalScore > 0.0)
					{
						float noneBackgroundScore = candidate->scoreArea(separator);
						//totalScore *= noneBackgroundScore;
						candidate->backgroundEdgeHitScore = ComputeHitScore(edg, candidate);
						candidate->imageEdgeHitScore = ComputeHitScore(EdgeMap, candidate);
						
						if (candidate->backgroundEdgeHitScore < 0.15)
						{
							continue;
						}
						totalScore *= pow(candidate->backgroundEdgeHitScore, 1.25);
						//totalScore *= candidate->backgroundEdgeHitScore;
#if USE_IMAG_EDGES_IN_PUZZEL_DETECTION
						totalScore *= candidate->imageEdgeHitScore;
#endif
						candidate->score = totalScore;
#if VERBOSE
						cout << "C  ";
						candidate->PrintScores(); 
#endif
						if (bestRects == nullptr || candidate->score > bestRects->score)
						{
							swap(bestRects, candidate);
						}
						if (candidate != nullptr)
						{
							delete candidate;
						}
					}
				}
			}
		}
	}
	return bestRects;
}