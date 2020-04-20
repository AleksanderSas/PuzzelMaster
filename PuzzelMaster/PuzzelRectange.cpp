#include "PuzzelRectange.h"
#include "math.h"
#include "KMeans.h"
#include "Utils.h"
#include "MathUtils.h"
#include "LineProcessor.h"
#include "DebugFlags.h"

#define M_PI 3.14159265358979323846


PuzzelRectange::PuzzelRectange(
	Point2f& left, 
	Point2f& right, 
	Point2f& lower, 
	Point2f& upper, 
	long long int id,
	long long int intrestingAreaId,
	BackgroundSeparator* _backgroundSeparator,
	RotatedRect box)
	: left(left), right(right), lower(lower), upper(upper), id(id), backgroundSeparator(_backgroundSeparator), box(box), intrestingAreaId(intrestingAreaId)
{
	lineParameters_left_upper = ParametrizeLine(left, upper);
	lineParameters_upper_rigth = ParametrizeLine(upper, right);
	lineParameters_right_lower = ParametrizeLine(right, lower);
	lineParameters_lower_left = ParametrizeLine(lower, left);

	hasBoundaryEdge = false;
}


void ComputeFeatureVector(Mat &m, Point2f startCorner, Point2f endCorner, Point2f startBackSide, Point2f endBackSide, Vec3i joint, EdgeFeatureVector *v, float factor)
{
	Point2f start = LinearComb(startCorner, startBackSide, factor);
	Point2f end = LinearComb(endCorner, endBackSide, factor);
	v->len = (int)hypot(start.x - end.x, start.y - end.y);

	int start2jointDist = hypot(joint[0] - start.x, joint[1] - start.y);
	int joint2endDist = hypot(joint[0] - end.x, joint[1] - end.y);
	int dist = start2jointDist + joint2endDist;

	Point2f start2joint = LinearComb(start, end,1.0 * (dist - start2jointDist + joint[2] )/ dist);
	Point2f joint2end = LinearComb(end, start, 1.0 * (dist - joint2endDist + joint[2]) / dist);

	LineProcessor::Process(start, start2joint, [&](int x, int y) {v->colors1.push_back(m.at<Vec3b>(y, x)); return true; });
	LineProcessor::Process(end, joint2end, [&](int x, int y) {v->colors2.push_back(m.at<Vec3b>(y, x)); return true; });
}

void ComputeFeatureVector(Mat& m, Point2f startCorner, Point2f endCorner, Point2f startBackSide, Point2f endBackSide, edgeFeature* e)
{
	ComputeFeatureVector(m, startCorner, endCorner, startBackSide, endBackSide, e->joint, e->colors, 0.99f);
	ComputeFeatureVector(m, startCorner, endCorner, startBackSide, endBackSide, e->joint, e->colors + 1, 0.96f);
	ComputeFeatureVector(m, startCorner, endCorner, startBackSide, endBackSide, e->joint, e->colors + 2, 0.93f);
	ComputeFeatureVector(m, startCorner, endCorner, startBackSide, endBackSide, e->joint, e->colors + 3, 0.90f);
}

void RemoveLines(Mat& edges)
{
	vector<Vec4i> lines;
	//HoughLines(edges, lines, 1, CV_PI / 180, 20, 0, 0);
	HoughLinesP(edges, lines, 1, 0.01, LINE_TRESHOLD, MIN_LINE_LEN, MAX_LINE_GAP);
	for (size_t i = 0; i < lines.size(); i++)
	{
		Vec4i l = lines[i];
		//line(sourceImg, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 1, LINE_AA);
		line(edges, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0), 2, LINE_AA);
	}
}

vector<Vec3f> PuzzelRectange::FindJointCandidates(Mat& image_gray, int circleTreshold)
{
	int d = hypot(left.x - upper.x, left.y - upper.y);
	int maxRadius = d / 5;
	int minRadius = d / 9;
	Mat canny_output ;
	int cannEdgeThresh = 70;

	Scalar color(255);
	Canny(image_gray, canny_output, cannEdgeThresh, cannEdgeThresh * 2);

	RemoveLines(canny_output);

	int thickness = 2;
	int minDist = d / 15;
	line(canny_output, left, upper, color, thickness);
	line(canny_output, left, lower, color, thickness);
	line(canny_output, right, upper, color, thickness);
	line(canny_output, right, lower, color, thickness);
	edges = canny_output;

	vector<Vec3f> circles;
	//HoughCircles(canny_output, circles, HOUGH_GRADIENT, 1.0,
	//	minDist,  // change this value to detect circles with different distances to each other
	//	13, 7, minRadius, maxRadius // change the last two parameters
	//// (min_radius & max_radius) to detect larger circles
	//);
	HoughCircles(canny_output, circles, HOUGH_GRADIENT, 1.3,
		minDist,  // change this value to detect circles with different distances to each other
		15, circleTreshold, minRadius, maxRadius // change the last two parameters
	// (min_radius & max_radius) to detect larger circles
	);

	return circles;
}

void PuzzelRectange::ComputeEdgeFeatures()
{
	Mat image_gray;
	cvtColor(puzzelArea, image_gray, COLOR_BGR2GRAY);
	vector<Vec3f> circles = FindJointCandidates(image_gray, 9);
	vector<Vec3f> circles2 = FindJointCandidates(backgroundMap, 10);
	edgeFeature* e = edgeFeatures;
	e->start = left;
	e->end = lower;
	FindBestCircleJoin(circles, circles2, left, lower, e);
	ComputeFeatureVector(puzzelArea, left, lower, upper, right, e++);
	
	e->start = lower;
	e->end = right;
	FindBestCircleJoin(circles, circles2, lower, right, e);
	ComputeFeatureVector(puzzelArea, lower, right, left, upper, e++);

	e->start = right;
	e->end = upper;
	FindBestCircleJoin(circles, circles2, right, upper, e);
	ComputeFeatureVector(puzzelArea, right, upper, lower, left, e++);

	e->start = upper;
	e->end = left;
	FindBestCircleJoin(circles, circles2, upper, left, e);
	ComputeFeatureVector(puzzelArea, upper, left, right, lower, e++);
	
	line(puzzelArea, lower, right, Scalar(200, 200, 40), 2);
	line(puzzelArea, left, upper, Scalar(200, 200, 40), 2);
}

double compare(uchar a, uchar b)
{
	unsigned int sum = a + b;
	return 1.0 * (a - b ) * (a - b) / sum;
}

unsigned int squareLen(Vec3b v)
{
	return(unsigned int)v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
}

double len(Vec3b v) 
{ 
	return sqrt(squareLen(v));
}

uchar substructAbs(uchar a, uchar b)
{
	return a >= b ? a - b : b - a;
}

double compare(Vec3b a, Vec3b b)
{
	Vec3b diff(substructAbs(a[0], b[0]), substructAbs(a[1], b[1]), substructAbs(a[2], b[2]));

	return 1.0 * squareLen(diff) / (len(a) + len(b));
}

static double feature(vector<Vec3b>& v1, vector<Vec3b>& v2, int s1, int s2)
{
	int c = 0;
	double diff = 0.0;

	vector<Vec3b>::iterator it1 = v1.begin() + s1;
	vector<Vec3b>::iterator it2 = v2.begin() + s2;

	for (; it1 != v1.end() && it2 != v2.end(); it1++, it2++)
	{
		//diff += compare((*it1)[0], (*it2)[0]) + compare((*it1)[1], (*it2)[1]) + compare((*it1)[2], (*it2)[2]);
		diff += compare(*it1, *it2);
		c++;
	}

	return diff / c;
	//int diffSize = v1.size() - v2.size();
	//int penalty = abs(diffSize);
	//return pair<double, int>(diff / c, penalty * 10);
}

static double featureWithShift(vector<Vec3b>& v1, vector<Vec3b>& v2)
{
	double best = DBL_MAX;
	for (int i = 0; i < 3; i++)
	{
		double current = feature(v1, v2, 0, i);
		if (current < best)
			best = current;
	}
	for (int i = 1; i < 3; i++)
	{
		double current = feature(v1, v2, i, 0);
		if (current < best)
			best = current;
	}
	return best;
}

static int Abs(int x) { return x >= 0 ? x : -x; }

pair<double, int> CompareFeatureVectors2(EdgeFeatureVector* e1, EdgeFeatureVector* e2)
{
#if USE_ADAPTIVE_FEATURE_SHIFT
		auto p1 = featureWithShift(e1->colors1, e2->colors2);
		auto p2 = featureWithShift(e1->colors2, e2->colors1);
#else
		auto p1 = feature(e1->colors1, e2->colors2, 0, 0);
		auto p2 = feature(e1->colors2, e2->colors1, 0, 0);
#endif
		
		int diffVectorSize = Abs(e1->colors1.size() + e1->colors2.size() - e2->colors1.size() - e2->colors2.size());
		int diffTotalSize = Abs(e1->len - e2->len);
		int diffScore = diffVectorSize * 2 + diffTotalSize * 4;
		double colorScore = (p1 + p2) * 4;

		return pair<double, int>(colorScore, diffScore);
}

pair<double, int> PuzzelRectange::CompareFeatureVectors(edgeFeature* e1, edgeFeature* e2)
{
	if (e1->isMaleJoint ^ e2->isMaleJoint && e1->hasJoint && e2->hasJoint)
	{

#if USE_ADAPTIVE_FEATURE_MARGINE
		pair<double, int> bestResult(0.0, INT_MAX);
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				pair<double, int> result = CompareFeatureVectors2(e1->colors + i, e2->colors + j);
				if (bestResult.first + bestResult.second > result.first + result.second)
				{
					bestResult = result;
				}
			}
		}
		return bestResult;
#else
		return CompareFeatureVectors2(e1->colors + 1, e2->colors + 1);
#endif

	}
	return pair<double, int>(DBL_MAX, INT_MAX);
}

static RNG rng(26358);
void PuzzelRectange::FindNeighbour(vector<PuzzelRectange*> &puzzels, int edgeNr, string name)
{
	double best = DBL_MAX;
	double bestSecond = DBL_MAX;
	int sideNr = 0;
	int sideNrSecond;
	PuzzelRectange* bestP = NULL;
	PuzzelRectange* bestPSecond = NULL;

	unsigned int colorIdx = rng.next() % 16;
	unsigned char* c = Utils::GetColorFromTable(colorIdx);
	Scalar color(c[2], c[1], c[0]);

	for (vector<PuzzelRectange*>::iterator p = puzzels.begin(); p != puzzels.end(); p++)
	{
		PuzzelRectange* puzzel = *p;
		if (puzzel != this)
		{
			for (int i = 0; i < 4; i++)
			{
				pair<double, int> score = CompareFeatureVectors(edgeFeatures + edgeNr, puzzel->edgeFeatures + i);
				double r = score.first + score.second;
				if (score.first > 9999999)
					continue;

				char buffer[255];
				sprintf_s(buffer, "edge [%d:%d]  : %g  =  %g  +  %d\n", puzzel->id, i, r, score.first, score.second);
				Utils::WriteColoredText(string(buffer), colorIdx);

				if (r < best)
				{
					bestSecond = best;
					bestPSecond = bestP;
					sideNrSecond = sideNr;

					sideNr = i;
					best = r;
					bestP = puzzel;
				}
				else if (r < bestSecond)
				{
					bestSecond = r;
					bestPSecond = puzzel;
					sideNrSecond = i;
				}
			}
		}
	}

	Utils::WriteColoredText("best score  [" + to_string(bestP->id) + ":"+ to_string(sideNr) + "]  : " + to_string(best) + "\n", colorIdx);

	drawMarker(puzzelArea, LinearComb(edgeFeatures[edgeNr].start, edgeFeatures[edgeNr].end, 0.5), color, 0, 20, 3);
	drawMarker(bestP->puzzelArea, LinearComb(bestP->edgeFeatures[sideNr].start, bestP->edgeFeatures[sideNr].end, 0.5), color, 0, 20, 3);
	//imshow(name, bestP->puzzelArea);
	if (bestPSecond != nullptr)
	{
		drawMarker(bestPSecond->puzzelArea, LinearComb(bestPSecond->edgeFeatures[sideNrSecond].start, bestPSecond->edgeFeatures[sideNrSecond].end, 0.5), color, 0, 15, 2);
		Utils::WriteColoredText("second score [" + to_string(bestPSecond->id) + ":" + to_string(sideNrSecond) + "] : " + to_string(bestSecond) + "\n", colorIdx);
		//imshow(name + "_second", bestPSecond->puzzelArea);
	}
	else
	{
		Utils::WriteColoredText("no second score\n", colorIdx);
	}
	waitKey(1);
}

static vector<Point>* FindClosestContour(vector<vector<Point>*> contours, Point p)
{
	int dist = INT32_MAX;
	vector<Point>* best = nullptr;

	for(vector<Point>* contour : contours)
	{
		Point cp = contour->at(0);
		int d = squareDist(cp, p);
		if (dist > d)
		{
			dist = d;
			best = contour;
		}

		Point cp2 = contour->at(contour->size() - 1);
		d = squareDist(cp2, p);
		if (dist > d)
		{
			dist = d;
			best = contour;
		}
	}

	return best;
}

static int distSign(Vec3f &line, int x, int y)
{
	return line[0] * x + line[1] * y + line[2];
}

float PuzzelRectange::computeBackgroundSimilarity(Vec3f circle, bool shouldBeInside)
{
	int circleX = circle[0];
	int circleY = circle[1];
	int circleR = circle[2];
	int circleR_2 = circleR * circleR;
	float score = 0;
	int count = 0;

	int yStart = circleY - circleR;
	int yStop = MIN(circleY + circleR, puzzelArea.rows);
	for (int y = MAX(0, yStart); y < yStop; y++)
	{
		int xStart = circleX - circleR;
		int xStop = MIN(circleX + circleR, puzzelArea.cols);
		for (int x = MAX(0, xStart); x < xStop; x++)
		{
			int dx = x - circleX;
			int dy = y - circleY;
			bool _isPointInside = isPointInside(x, y);
			if (dx * dx + dy * dy <= circleR_2)
			{
				if (shouldBeInside && _isPointInside || !shouldBeInside && !_isPointInside)
				{
					Vec3b pixel = puzzelArea.at<Vec3b>(y, x);
					float tmp = shouldBeInside ? isBackground(pixel) : isNotBackground(pixel);
					if (!isnan(tmp))
					{
						count++;
						score += tmp;
					}
				}
				else
				{
					count++;
				}
			}
		}
	}
	if (count == 0)
		return 0.0;

	return score / count;
}

float PuzzelRectange::scoreCircle(Vec3f circle)
{
	bool isInside = isPointInside(circle[0], circle[1]);
	return computeBackgroundSimilarity(circle, isInside);
}

static void SelectCirclesAlignedToEdge(vector<Vec3f>& circles, Point2f c1, Point2f c2, vector<Vec3f>& selectedCircles)
{
	Vec3f lineParams = ParametrizeLine(c1, c2);

	for (size_t i = 0; i < circles.size(); i++)
	{
		Vec3i _circle = circles[i];
		Point center = Point(_circle[0], _circle[1]);
		int radius = _circle[2];
		int squareDistValue = squareDist(lineParams, center.x, center.y);
		//allow only circles near to the line
		if (squareDistValue < 1.7 * radius * radius && squareDistValue > 0.4 * radius * radius)
		{
			int c1Dist = hypot(center.x - c1.x, center.y - c1.y);
			int c2Dist = hypot(center.x - c2.x, center.y - c2.y);
			int len = hypot(c1.x - c2.x, c1.y - c2.y);

			int shift = Abs(c1Dist - c2Dist) / 2;
			//allow only circles between c1 and c2
			if (c1Dist > len - radius || c2Dist > len - radius)
			{
				continue;
			}
			//allow only circles near to the line centre
			if (1.0 * shift / len < 0.2)
			{
				selectedCircles.push_back(_circle);
			}
		}
	}
}

void PuzzelRectange::FindBestCircleJoin(vector<Vec3f>& circles, vector<Vec3f>& circles2, Point2f c1, Point2f c2, edgeFeature* e)
{
	Vec3i candidate;
	float bestScore = -9999;
	float bestCircleScore1 = FindCircleJointCandidates(circles, c1, c2, bestScore, candidate, e, 0);
	float bestCircleScore2 = FindCircleJointCandidates(circles2, c1, c2, bestScore, candidate, e, circles.size() / 3);
	float bestCircleScore = MAX(bestCircleScore1, bestCircleScore2);
	e->hasJoint = bestCircleScore > 0.30;
	hasBoundaryEdge |= !e->hasJoint;

	cout << endl;

	if (e->hasJoint)
	{
		e->isMaleJoint = isPointInside(candidate[0], candidate[1]);
		e->joint = candidate;
	}

	Point center = Point(candidate[0], candidate[1]);
	circle(puzzelArea, center, 1, e->isMaleJoint? Scalar(0, 100, 100) : Scalar(100, 40, 200), 3, LINE_AA);
}

float PuzzelRectange::FindCircleJointCandidates(std::vector<cv::Vec3f>& circles, cv::Point2f& c1, cv::Point2f& c2, float& bestScore, cv::Vec3i& candidate, edgeFeature* e, int offset)
{
	vector<Vec3f> finalCircles;

	SelectCirclesAlignedToEdge(circles, c1, c2, finalCircles);
	float bestCircleScore = 0.0;
	for (size_t i = 0; i < finalCircles.size(); i++)
	{
		Vec3i _circle = finalCircles[i];
		float orderScore = 1.0 * (1.0 - (1.0 * i + offset) / (finalCircles.size() + offset));
		float coverScore = scoreCircle(_circle);
		float condidateScore = coverScore * orderScore;
		if (bestScore < condidateScore && coverScore > MIN_COVER_SCORE)
		{
			candidate = _circle;
			bestScore = condidateScore;
			bestCircleScore = coverScore;
		}

#if 0
		if (id != 200251157 || e - edgeFeatures != 1)
			continue;
		unsigned int colorIdx = i % 16;
		unsigned char* c = Utils::GetColorFromTable(colorIdx);
		Scalar color(c[2], c[1], c[0]);

		int radius = _circle[2];
		char buffer[255];
		sprintf_s(buffer, "joint score:  T %f    Ord %f   Cov %f  %s\n", condidateScore, orderScore, coverScore, coverScore > MIN_COVER_SCORE ? "" : "XXX");
		Utils::WriteColoredText(string(buffer), colorIdx);
		circle(puzzelArea, Point(_circle[0], _circle[1]), radius, color / (coverScore > MIN_COVER_SCORE ? 1 : 2), 1, LINE_AA);
#endif
	}
	return bestCircleScore;
}

bool PuzzelRectange::isPointInside(int x, int y)
{
	int testX = (lower.x + upper.x) / 2;
	int testY = (lower.y + upper.y) / 2;

	int f1 = distSign(lineParameters_left_upper, testX, testY) < 0 ? -1 : 1;
	int f2 = distSign(lineParameters_lower_left, testX, testY) < 0 ? -1 : 1;
	int f3 = distSign(lineParameters_right_lower, testX, testY) < 0 ? -1 : 1;
	int f4 = distSign(lineParameters_upper_rigth, testX, testY) < 0 ? -1 : 1;

	return distSign(lineParameters_left_upper, x, y) * f1 >= 0 &&
		   distSign(lineParameters_lower_left, x, y) * f2 >= 0 &&
		   distSign(lineParameters_right_lower, x, y) * f3 >= 0 &&
		   distSign(lineParameters_upper_rigth, x, y) * f4 >= 0;
}

Mat PuzzelRectange::GetContours()
{
	Mat image_gray, canny_output;
	cvtColor(puzzelArea, image_gray, COLOR_BGR2GRAY);
	int cannEdgeThresh = 70;
	Canny(image_gray, canny_output, cannEdgeThresh, cannEdgeThresh * 2);
	return canny_output;
}

float PuzzelRectange::isNotBackground(Vec3b pixel)
{
	return 1 - isBackground(pixel);
}

float PuzzelRectange::isBackground(Vec3b pixel)
{
	return backgroundSeparator->scorePoint(&pixel);
}

//TODO: to moze byc keszowane pomiedzy kandydatami korzystajacemi z tego samego obszaru
float PuzzelRectange::scoreArea(BackgroundSeparator* separator)
{
	int count = 0;
	float score = 0.0;
	Vec3b* p_source;
	for (int i = 0; i < puzzelArea.rows; ++i)
	{
		p_source = puzzelArea.ptr<Vec3b>(i);
		for (int j = 0; j < puzzelArea.cols; ++j, ++p_source)
		{
			if (isPointInside(j, i))
			{
				float BnondebackgroundProbability = (1 - separator->scorePoint(p_source));
				if (!isnan(BnondebackgroundProbability))
				{
					count++;
					score += BnondebackgroundProbability;
				}
			}
		}
	}
	if (count == 0)
		return 0;
	score /= count;
	noneBackgroundScore = score;

	return score;
}

void PuzzelRectange::PrintScores()
{
	cout << " n=" << id
		<< "\t total score=" << score
		<< "  background edge hit score=" << backgroundEdgeHitScore
		<< "  image hit score=" << imageEdgeHitScore
		<< "  rec score=" << recScore
		<< "  area score=" << areaScore
		<< "  bckg score=" << noneBackgroundScore
		<< endl;
}

static Point2f TransformPoint(Point2f p, RotatedRect rect)
{
	float angle = rect.angle;
	double radians = 2 * M_PI * angle / 360;
	float _sin = sin(radians);
	float _cos = cos(radians);

	p.x -= rect.size.width / 2 + AREA_PADDING;
	p.y -= rect.size.height / 2 + AREA_PADDING;

	Point2f rotatedPoint(
		_cos * p.x - _sin * p.y,
		_sin * p.x + _cos * p.y);

	rotatedPoint += rect.center;
	return rotatedPoint;
}

void PuzzelRectange::MarkEdgesOnOriginImage(Mat& image)
{
	Point2f p1 = TransformPoint(left, box);
	Point2f p2 = TransformPoint(right, box);
	Point2f p3 = TransformPoint(upper, box);
	Point2f p4 = TransformPoint(lower, box);

	drawMarker(image, p1, Scalar(123, 231, 90));
	drawMarker(image, p2, Scalar(123, 231, 90));
	drawMarker(image, p3, Scalar(123, 231, 90));
	drawMarker(image, p4, Scalar(123, 231, 90));
}

void PuzzelRectange::MarkJointsOnOriginImage(Mat& image)
{
	for (int i = 0; i < 4; i++)
	{
		auto joint = edgeFeatures[i];
		if (joint.hasJoint)
		{
			Point2f p1 = TransformPoint(Point2f(joint.joint[0], joint.joint[1]), box);
			circle(image, p1, 2, joint.isMaleJoint ? Scalar(250, 255, 50) : Scalar(90, 30, 255), 3, LINE_AA);
		}
		else
		{
			cout << "no edge: " << id << endl;
			Point2f centre = TransformPoint(LinearComb(joint.end, joint.start, 0.5), box);
			circle(image, centre, 3, Scalar(200, 40, 40), 4, LINE_AA);
		}
	}
}

static Point2f RotatePoint(Point2f p, float radians, Point2f rotatioPoint)
{
	float _sin = sin(radians);
	float _cos = cos(radians);

	p -= rotatioPoint;

	Point2f rotatedPoint(
		_cos * p.x - _sin * p.y,
		_sin * p.x + _cos * p.y);

	rotatedPoint += rotatioPoint;
	return rotatedPoint;
}

Mat PuzzelRectange::ExtractPuzzelAndRotateEdgeToUp(int edgeIdx,int padding)
{
	Mat M, rotated, cropped;
	Point2f rotationPoint = LinearComb(left, right, 0.5);
	Point2f p = edgeFeatures[edgeIdx].end - edgeFeatures[edgeIdx].start;
	float radius = atan2(p.y, p.x);
	float degree = radius * 180 / M_PI;

	Point2f p1 = RotatePoint(left, -radius, rotationPoint);
	Point2f p2 = RotatePoint(right, -radius, rotationPoint);

	M = getRotationMatrix2D(rotationPoint, degree, 1.0);
	warpAffine(puzzelArea, rotated, M, puzzelArea.size(), INTER_CUBIC);

	drawMarker(rotated, p1, Scalar(125, 200, 40));
	drawMarker(rotated, p2, Scalar(125, 200, 40));

	Size rect_size(abs(p1.x - p2.x), abs(p1.y - p2.y));
	rect_size.width += padding * 2;
	rect_size.height += padding * 2;

	getRectSubPix(rotated, rect_size, rotationPoint, cropped);
	return cropped;
}

bool PuzzelRectange::HasBoundaryEdge()
{
	return hasBoundaryEdge;
}

string PuzzelRectange::GetId()
{
	return to_string(id) + " : " + to_string(intrestingAreaId);
}