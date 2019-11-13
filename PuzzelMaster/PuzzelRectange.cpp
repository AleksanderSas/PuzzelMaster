#include "PuzzelRectange.h"
#include "math.h"
#include "KMeans.h"
#include "Utils.h"
#include "LineProcessor.h"

#define M_PI 3.14159265358979323846

// [A, B, C]
Vec3f ParametrizeLine(Point2f& c1, Point2f& c2)
{
	Vec3f p;
	int dx = c1.x - c2.x;
	int dy = c1.y - c2.y;

	if (abs(dx) < abs(dy) && c1.y != c2.y)
	{
		p[0] = 1.0;
		p[1] = -1.0 * dx / dy;
	}
	else
	{
		p[1] = 1.0;
		p[0] = -1.0 * dy / dx;
	}
	p[2] = -p[1] * c1.y - p[0] * c1.x;
	return p;
}

PuzzelRectange::PuzzelRectange(
	Point2f& left, 
	Point2f& right, 
	Point2f& lower, 
	Point2f& upper, 
	int id, 
	BackgroundSeparator* _backgroundSeparator,
	RotatedRect box)
	: left(left), right(right), lower(lower), upper(upper), id(id), backgroundSeparator(_backgroundSeparator), box(box)
{
	lineParameters_left_upper = ParametrizeLine(left, upper);
	lineParameters_upper_rigth = ParametrizeLine(upper, right);
	lineParameters_right_lower = ParametrizeLine(right, lower);
	lineParameters_lower_left = ParametrizeLine(lower, left);
}

Point2f TrasformPoint(Point2f &p, Mat &transformation)
{
	Mat_<float> pm(3, 1);
	pm << p.x, p.y, 1.0;
	Mat_<float> pr = transformation * pm;
	return Point2f(pr(0), pr(1));
}

Point2f LinearComb(Point2f p1, Point2f p2, float p1Weigth)
{
	return Point2f(p1.x * p1Weigth + p2.x * (1 - p1Weigth), p1.y * p1Weigth + p2.y * (1 - p1Weigth));
}

void ComputeFeatureVector(Mat &m, Point2f& startCorner, Point2f& endCorner, Point2f& startBeckSide, Point2f& endBeckSide, edgeFeature* e)
{
	float factor = 0.95f;
	Point2f start = LinearComb(startCorner, startBeckSide, factor);
	Point2f end = LinearComb(endCorner, endBeckSide, factor);
	e->len = (int)hypot(start.x - end.x, start.y - end.y);;

	int start2jointDist = hypot(e->joint[0] - start.x, e->joint[1] - start.y);
	int joint2endDist = hypot(e->joint[0] - end.x, e->joint[1] - end.y);
	int dist = start2jointDist + joint2endDist;

	Point2f start2joint = LinearComb(start, end, 1.0 * (dist - start2jointDist + e->joint[2] )/ dist);
	Point2f joint2end = LinearComb(end, start, 1.0 * (dist - joint2endDist + e->joint[2]) / dist);

	LineProcessor::Process(start, start2joint, [&](int x, int y) {e->colors1.push_back(m.at<Vec3b>(y, x)); return true; });
	LineProcessor::Process(joint2end, end, [&](int x, int y) {e->colors2.push_back(m.at<Vec3b>(y, x)); return true; });
}


int squareDist(Vec3f& line, int x, int y)
{
	int tmp = line[0] * x + line[1] * y + line[2];
	int tmp2 = line[0] * line[0] + line[1] * line[1];
	return tmp * tmp / tmp2;
}

void RemoveLines(Mat& edges, Mat& sourceImg)
{
	vector<Vec4i> lines;
	//HoughLines(edges, lines, 1, CV_PI / 180, 20, 0, 0);
	HoughLinesP(edges, lines, 1, 0.01, LINE_TRESHOLD, MIN_LINE_LEN, MAX_LINE_GAP);
	for (size_t i = 0; i < lines.size(); i++)
	{
		Vec4i l = lines[i];
		//line(sourceImg, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 1, CV_AA);
		line(edges, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0), 2, CV_AA);
	}
}

vector<Vec3f> PuzzelRectange::FindJointCandidates(Mat& puzzelArea)
{
	int d = hypot(left.x - upper.x, left.y - upper.y);
	int maxRadius = d / 5;
	int minRadius = d / 9;
	Mat image_gray, canny_output ;
	cvtColor(puzzelArea, image_gray, COLOR_BGR2GRAY);
	int cannEdgeThresh = 70;

	Mat element = getStructuringElement(MORPH_RECT,
		Size(3, 3),
		Point(1, 1));
	Scalar color(255);
	Canny(image_gray, canny_output, cannEdgeThresh, cannEdgeThresh * 2);

	RemoveLines(canny_output, puzzelArea);

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
		15, 9, minRadius, maxRadius // change the last two parameters
	// (min_radius & max_radius) to detect larger circles
	);

	return circles;
}

void PuzzelRectange::ComputeEdgeFeatures()
{
	vector<Vec3f> circles = FindJointCandidates(puzzelArea);
	edgeFeature* e = edgeFeatures;
	e->start = left;
	e->end = lower;
	FindBestCircleJoin(circles, left, lower, e);
	ComputeFeatureVector(puzzelArea, left, lower, upper, right, e++);
	
	e->start = lower;
	e->end = right;
	FindBestCircleJoin(circles, lower, right, e);
	ComputeFeatureVector(puzzelArea, lower, right, left, upper, e++);

	e->start = right;
	e->end = upper;
	FindBestCircleJoin(circles, right, upper, e);
	ComputeFeatureVector(puzzelArea, right, upper, lower, left, e++);

	e->start = upper;
	e->end = left;
	FindBestCircleJoin(circles, upper, left, e);
	ComputeFeatureVector(puzzelArea, upper, left, right, lower, e++);
	
	line(puzzelArea, lower, right, Scalar(200, 200, 40));
	line(puzzelArea, left, upper, Scalar(200, 200, 40));
}

double compare(uchar a, uchar b)
{
	return 1.0 * (a - b) * (a - b) / (a + b);
}

float feature(vector<Vec3b>& v1, vector<Vec3b>& v2)
{
	int c = 0;
	double diff = 0.0;

	vector<Vec3b>::iterator it1 = v1.begin();
	vector<Vec3b>::reverse_iterator it2 = v2.rbegin();

	for (; it1 != v1.end() && it2 != v2.rend(); it1++, it2++)
	{
		diff += compare((*it1)[0], (*it2)[0]) + compare((*it1)[1], (*it2)[1]) + compare((*it1)[2], (*it2)[2]);
		c++;
	}

	int diffSize = v1.size() - v2.size();
	int penalty = abs(diffSize);

	return diff / c + penalty * 10;
}

double CompareFeatureVectors(edgeFeature* e1, edgeFeature* e2)
{
	if (e1->isMaleJoint ^ e2->isMaleJoint)
	{
		float match = feature(e1->colors1, e2->colors2);
		match += feature(e1->colors2, e2->colors1);
		return match;
		/*int c = e1->colors1.size() + e1->colors2.size() + e2->colors1.size() + e2->colors2.size();
		float rMatch = compare(e1->joint[2], e2->joint[2]) * c/4 / 5;
		return match + rMatch;*/

	}
	return DBL_MAX;
}

static RNG rng(13375);
void PuzzelRectange::FindNeighbour(vector<PuzzelRectange*> &puzzels, int edgeNr, string name)
{
	double best = DBL_MAX;
	double bestSecond = DBL_MAX;
	int sideNr = 0;
	int sideNrSecond;
	PuzzelRectange* bestP = NULL;
	PuzzelRectange* bestPSecond = NULL;

	unsigned int colorIdx = rng.next() % 16;
	unsigned char* c = Utils::GetCOlorFromTable(colorIdx);
	Scalar color(c[2], c[1], c[0]);

	for (vector<PuzzelRectange*>::iterator p = puzzels.begin(); p != puzzels.end(); p++)
	{
		PuzzelRectange* puzzel = *p;
		if (puzzel != this)
		{
			for (int i = 0; i < 4; i++)
			{
				double r = CompareFeatureVectors(edgeFeatures + edgeNr, puzzel->edgeFeatures + i);
				if(r < 9999999)
					Utils::WriteColoredText("edge [" + to_string(puzzel->id) + ":" + to_string(i) + "]  : " + to_string(r) + "\n", colorIdx);
				if (r < best)
				{
					bestSecond = best;
					bestPSecond = bestP;
					sideNrSecond = sideNr;

					sideNr = i;
					best = r;
					bestP = puzzel;
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
		drawMarker(bestPSecond->puzzelArea, LinearComb(bestPSecond->edgeFeatures[sideNrSecond].start, bestPSecond->edgeFeatures[sideNrSecond].end, 0.5), color, 0, 20, 1);
		Utils::WriteColoredText("second score [" + to_string(bestPSecond->id) + ":" + to_string(sideNrSecond) + "] : " + to_string(bestSecond) + "\n", colorIdx);
		//imshow(name + "_second", bestPSecond->puzzelArea);
	}
	else
	{
		Utils::WriteColoredText("no second score\n", colorIdx);
	}
	waitKey(1);
}

static int squareDist(Point& p1, Point& p2)
{
	int x = p1.x - p2.x;
	int y = p1.y - p2.y;
	return x * x + y * y;
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

float PuzzelRectange::computeBackgroundSimilarity(Vec3f circle, bool isinside)
{
	int circleX = circle[0];
	int circleY = circle[1];
	int circleR = circle[2];
	int circleR_2 = circleR * circleR;
	float score = 0;
	int count = 0;

	int yStart = circleY - circleR / 2;
	int yStop = MIN(circleY + circleR / 2, puzzelArea.rows);
	for (int y = MAX(0, yStart); y < yStop; y++)
	{
		int xStart = circleX - circleR / 2;
		int xStop = MIN(circleX + circleR / 2, puzzelArea.cols);
		for (int x = MAX(0, xStart); x < xStop; x++)
		{
			int dx = x - circleX;
			int dy = y - circleY;
			bool _isPointInside = isPointInside(x, y);
			if (dx * dx + dy * dy <= circleR_2 && (isinside && _isPointInside || !isinside && !_isPointInside))
			{
				Vec3b pixel = puzzelArea.at<Vec3b>(y, x);
				float tmp = isinside ? isBackground(pixel) : isNotBackground(pixel);
				if (!isnan(tmp))
				{
					count++;
					score += tmp;
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

void PuzzelRectange::FindBestCircleJoin(vector<Vec3f>& circles, Point2f c1, Point2f c2, edgeFeature* e)
{
	Vec3f lineParams = ParametrizeLine(c1, c2);
	Vec3i candidate;
	float bestScore = -9999;

	int c = 0;
	for (size_t i = 0; i < circles.size(); i++)
	{
		Vec3i _circle = circles[i];
		Point center = Point(_circle[0], _circle[1]);
		int radius = _circle[2];
		int squareDistValue = squareDist(lineParams, center.x, center.y);
		if (squareDistValue < 1.7 * radius * radius && squareDistValue > 0.4 * radius * radius)
		{
			c++;
		}
	}

	int t = 0;
	for (size_t i = 0; i < circles.size(); i++)
	{
		Vec3i _circle = circles[i];
		Point center = Point(_circle[0], _circle[1]);
		int radius = _circle[2];
		int squareDistValue = squareDist(lineParams, center.x, center.y);
		if (squareDistValue < 1.7 * radius * radius && squareDistValue > 0.4 * radius * radius)
		{
			int c1Dist = hypot(center.x - c1.x, center.y - c1.y);
			int c2Dist = hypot(center.x - c2.x, center.y - c2.y);
			int len = hypot(c1.x - c2.x, c1.y - c2.y);

			int shift = abs(c1Dist - c2Dist) / 2;
			if (c1Dist > len - radius || c2Dist > len - radius || 1.0 * shift / len > 1.25)
			{
				continue;
			}

			float orderScore = 2.0 - 2.0 * t++ / c;
			float condidateScore = scoreCircle(_circle) + orderScore;

			if (1.0 * abs(c1Dist - c2Dist) / (c1Dist + c2Dist) < 0.5)
			{
				if (bestScore < condidateScore)
				{
					candidate = _circle;
					bestScore = condidateScore;
				}
				
#if 0
				cout << "joint score:  " << condidateScore << "  " << condidateScore - orderScore << "   " << orderScore << endl;
				circle(puzzelArea, Point(candidate[0], candidate[1]), radius, Scalar(100, 40, 200), 1, LINE_AA);
#endif
			}
		}
	}
	//cout << endl;

	e->isMaleJoint = isPointInside(candidate[0], candidate[1]);
	e->joint = candidate;

	Point center = Point(candidate[0], candidate[1]);
	circle(puzzelArea, center, 1, e->isMaleJoint? Scalar(0, 100, 100) : Scalar(100, 40, 200), 3, LINE_AA);
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
		<< "  hit score=" << hitScore
		<< "  rec score=" << recScore
		<< "  har score=" << interestScore
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
		auto j = edgeFeatures[i];
		Point2f p1 = TransformPoint(Point2f(j.joint[0], j.joint[1]), box);
		circle(image, p1, 2, j.isMaleJoint? Scalar(250, 255, 50) : Scalar(90, 30, 255), 3, LINE_AA);
	}
}