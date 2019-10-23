#include "PuzzelRectange.h"
#include "math.h"
#include "KMeans.h"
#include "Utils.h"

// [A, B, C]
Vec3f ParametrizeLine(Point2f& c1, Point2f& c2)
{
	Vec3f p;
	p[0] = 1.0;
	p[1] = 1.0 * (c2.x - c1.x) / (c1.y - c2.y);
	p[2] = 1.0 * -p[1] * c1.y - c1.x;
	return p;
}

PuzzelRectange::PuzzelRectange(Point2f& left, Point2f& right, Point2f& lower, Point2f& upper, int id)
	: left(left), right(right), lower(lower), upper(upper), id(id)
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
	float factor = 0.95;
	Point2f start = LinearComb(startCorner, startBeckSide, factor);
	Point2f end = LinearComb(endCorner, endBeckSide, factor);
	e->len = hypot(start.x - end.x, start.y - end.y);;

	int start2jointDist = hypot(e->joint[0] - start.x, e->joint[1] - start.y);
	int joint2endDist = hypot(e->joint[0] - end.x, e->joint[1] - end.y);
	int dist = start2jointDist + joint2endDist;

	Point2f start2joint = LinearComb(start, end, 1.0 * (dist - start2jointDist + e->joint[2] )/ dist);
	Point2f joint2end = LinearComb(end, start, 1.0 * (dist - joint2endDist + e->joint[2]) / dist);

	ProcessLine(start, start2joint, [&](int x, int y) {e->colors1.push_back(m.at<Vec3b>(y, x)); return true; });
	ProcessLine(joint2end, end, [&](int x, int y) {e->colors2.push_back(m.at<Vec3b>(y, x)); return true; });
}


int squareDist(Vec3f& line, int x, int y)
{
	int tmp = line[0] * x + line[1] * y + line[2];
	int tmp2 = line[0] * line[0] + line[1] * line[1];
	return tmp * tmp / tmp2;
}


void clean(vector<Vec4i> lines, Mat& img)
{
	for (int y = 0; y < img.rows; y++) {
		for (int x = 0; x < img.cols; x++) {
			unsigned char edge = img.at<unsigned char>(y, x);
			if (edge > 0)
			{
				for (auto& line : lines)
				{
					Point2f p1(line[0], line[1]);
					Point2f p2(line[2], line[3]);
					if ((p1.x >= x && x >= p2.x || p2.x >= x && x >= p1.x) && p1.y >= y && y >= p2.y || p2.y >= y && y >= p1.y)
					{
						auto lineParams = ParametrizeLine(p1, p2);
						int squareDistValue = squareDist(lineParams, x, y);
						if (squareDistValue <= 2)
						{
							img.at<unsigned char>(y, x) = 0;
							break;
						}
					}
				}
			}
		}
	}
}

void RemoveLines(Mat& edges, Mat& sourceImg)
{
	vector<Vec4i> lines;
	//HoughLines(edges, lines, 1, CV_PI / 180, 20, 0, 0);
	HoughLinesP(edges, lines, 1, 0.01, LINE_TRESHOLD, MIN_LINE_LEN, MAX_LINE_GAP);
	//clean(lines, edges);
	for (size_t i = 0; i < lines.size(); i++)
	{
		Vec4i l = lines[i];
		//line(sourceImg, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 1, CV_AA);
		line(edges, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0), 2, CV_AA);
	}
}

vector<Vec3f> PuzzelRectange::FindJointCandidates(Mat& puzzelArea)
{
	int xx1 = hypot(left.x - upper.x, left.y - upper.y);
	int xx2 = hypot(left.x - lower.x, left.y - lower.y);

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
	int minDist = d / 12;
	line(canny_output, left, upper, color, thickness);
	line(canny_output, left, lower, color, thickness);
	line(canny_output, right, upper, color, thickness);
	line(canny_output, right, lower, color, thickness);
	//dilate(canny_output, xxx, element);
	imshow("img" + to_string(id), canny_output);

	vector<Vec3f> circles;
	//HoughCircles(canny_output, circles, HOUGH_GRADIENT, 1.0,
	//	minDist,  // change this value to detect circles with different distances to each other
	//	13, 7, minRadius, maxRadius // change the last two parameters
	//// (min_radius & max_radius) to detect larger circles
	//);
	HoughCircles(canny_output, circles, HOUGH_GRADIENT, 1.3,
		minDist,  // change this value to detect circles with different distances to each other
		13, 7, minRadius, maxRadius // change the last two parameters
	// (min_radius & max_radius) to detect larger circles
	);

	return circles;
}

void PuzzelRectange::ComputeEdgeFeatures(string name)
{
	computeBackgroundColor();
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
	//imshow(name, puzzelArea);
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
void PuzzelRectange::FindNeighbour(vector<PuzzelRectange> &puzzels, int edgeNr, string name)
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

	for (vector<PuzzelRectange>::iterator p = puzzels.begin(); p != puzzels.end(); p++)
	{
		if (p._Ptr != this)
		{
			for (int i = 0; i < 4; i++)
			{
				double r = CompareFeatureVectors(edgeFeatures + edgeNr, p->edgeFeatures + i);
				if(r < 9999999)
					Utils::WriteColoredText("edge [" + to_string(p->id) + ":" + to_string(i) + "]  : " + to_string(r) + "\n", colorIdx);
				if (r < best)
				{
					bestSecond = best;
					bestPSecond = bestP;
					sideNrSecond = sideNr;

					sideNr = i;
					best = r;
					bestP = p._Ptr;
				}
			}
		}
	}

	Utils::WriteColoredText("best score  [" + to_string(bestP->id) + ":"+ to_string(sideNr) + "]  : " + to_string(best) + "\n", colorIdx);

	drawMarker(puzzelArea, LinearComb(edgeFeatures[edgeNr].start, edgeFeatures[edgeNr].end, 0.5), color, 0, 20, 3);
	imshow("source winner", puzzelArea);

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

int squareDist(Point& p1, Point& p2)
{
	int x = p1.x - p2.x;
	int y = p1.y - p2.y;
	return x * x + y * y;
}

vector<Point>* FindClosestContour(vector<vector<Point>*> contours, Point p)
{
	int dist = 999999;
	vector<Point>* best = NULL;

	for (auto iter = contours.begin(); iter != contours.end(); iter++)
	{

		Point cp = (*iter)->at(0);
		int d = squareDist(cp, p);
		if (dist > d)
		{
			dist = d;
			best = *iter;
		}

		Point cp2 = (*iter)->at((*iter)->size() - 1);
		d = squareDist(cp2, p);
		if (dist > d)
		{
			dist = d;
			best = *iter;
		}
	}

	return best;
}

int distSign(Vec3f &line, int x, int y)
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
				score += isinside ? isBackground(pixel) : isNotBackground(pixel);
			}
		}
	}

	return score;
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

	for (size_t i = 0; i < circles.size(); i++)
	{
		Vec3i circle = circles[i];
		Point center = Point(circle[0], circle[1]);
		int radius = circle[2];
		int squareDistValue = squareDist(lineParams, center.x, center.y);
		if (squareDistValue < 1.2 * radius * radius && squareDistValue > 0.4 * radius * radius)
		{
			int c1Dist = hypot(center.x - c1.x, center.y - c1.y);
			int c2Dist = hypot(center.x - c2.x, center.y - c2.y);
			int len = hypot(c1.x - c2.x, c1.y - c2.y);

			int shift = abs(c1Dist - c2Dist) / 2;
			if (c1Dist > len - radius || c2Dist > len - radius || 1.0 * shift / len > 1.25)
			{
				continue;
			}

			//int match = abs(radius - sqrt(squareDistValue)) + abs(c1Dist - c2Dist) - level * 8;
			float condidateScore = scoreCircle(circle);

			if (1.0 * abs(c1Dist - c2Dist) / (c1Dist + c2Dist) < 0.5)
			{
				if (bestScore < condidateScore)
				{
					candidate = circle;
					bestScore = condidateScore;
				}
			}
		}
	}
	
	Point center = Point(candidate[0], candidate[1]);
	circle(puzzelArea, center, 1, Scalar(0, 100, 100), 3, LINE_AA);

	// circle outline
	e->isMaleJoint = isPointInside(candidate[0], candidate[1]);
	e->joint = candidate;

}

void PuzzelRectange::ReconstructBorder()
{
	background = computeBackgroundColor();
	vector<Vec3f> circles = FindJointCandidates(puzzelArea);

	edgeFeature* e = edgeFeatures;
	FindBestCircleJoin(circles, left, upper, e++);
	FindBestCircleJoin(circles, left, lower, e++);
	FindBestCircleJoin(circles, right, upper, e++);
	FindBestCircleJoin(circles, right, lower, e++);
}

bool PuzzelRectange::isPointInside(int x, int y)
{
	int testX = (lower.x + upper.x) / 2;
	int testY = (lower.y + upper.y) / 2;

	int f1 = distSign(lineParameters_left_upper, testX, testY) < 0 ? -1 : 1;
	int f2 = distSign(lineParameters_lower_left, testX, testY) < 0 ? -1 : 1;
	int f3 = distSign(lineParameters_right_lower, testX, testY) < 0 ? -1 : 1;
	int f4 = distSign(lineParameters_upper_rigth, testX, testY) < 0 ? -1 : 1;

	return distSign(lineParameters_left_upper, x, y) * f1>= 0 &&
		   distSign(lineParameters_lower_left, x, y) * f2>= 0 &&
		   distSign(lineParameters_right_lower, x, y) * f3>= 0 &&
		   distSign(lineParameters_upper_rigth, x, y) * f4>= 0;
}

Mat PuzzelRectange::GetContours()
{
	Mat image_gray, canny_output;
	cvtColor(puzzelArea, image_gray, COLOR_BGR2GRAY);
	int cannEdgeThresh = 70;
	Canny(image_gray, canny_output, cannEdgeThresh, cannEdgeThresh * 2);
	return canny_output;
}


Mat PuzzelRectange::computeBackgroundColor()
{
	memset(qubeHistogram, 0, sizeof(float) * QUBE_SIZE * QUBE_SIZE * QUBE_SIZE);
	memset(qubeBackgroundHistogram, 0, sizeof(float) * QUBE_SIZE * QUBE_SIZE * QUBE_SIZE);
	
	int count = 0;

	for (int y = 0; y < puzzelArea.rows; y++) {
		for (int x = 0; x < puzzelArea.cols; x++) {
			auto pixel = puzzelArea.at<Vec3b>(y,x);
			if (!isPointInside(x, y))
			{
				qubeBackgroundHistogram[pixel[0] / QUBE_BIN][pixel[1] / QUBE_BIN][pixel[2] / QUBE_BIN] += 1;
				count++;
			}

			qubeHistogram[pixel[0] / QUBE_BIN][pixel[1] / QUBE_BIN][pixel[2] / QUBE_BIN] += 1;
		}
	}

	int total = puzzelArea.rows * puzzelArea.cols;
	
	for (int i = 0; i < QUBE_SIZE; i++)
		for (int j = 0; j < QUBE_SIZE; j++)
			for (int k = 0; k < QUBE_SIZE; k++)
			{
				qubeHistogram[i][j][k] /= total;
				qubeBackgroundHistogram[i][j][k] /= count;
			}


	backgroundProbability = 1.0 * count / total;

	Mat xxx = Mat::zeros(puzzelArea.size(), CV_8UC1);
	for (int y = 0; y < puzzelArea.rows; y++) {
		for (int x = 0; x < puzzelArea.cols; x++) {
			xxx.at<unsigned char>(y, x) = isNotBackground(puzzelArea.at<Vec3b>(y, x)) * 255;
		}
	}

	//imshow("backg_" + to_string(id), xxx);
	return xxx;
}

float PuzzelRectange::isNotBackground(Vec3b pixel)
{
	return 1 - isBackground(pixel);
}

float PuzzelRectange::isBackground(Vec3b pixel)
{
	float colorUnderBackgroundProbability = qubeBackgroundHistogram[pixel[0] / QUBE_BIN][pixel[1] / QUBE_BIN][pixel[2] / QUBE_BIN];
	float colorProbability = qubeHistogram[pixel[0] / QUBE_BIN][pixel[1] / QUBE_BIN][pixel[2] / QUBE_BIN];
	return colorUnderBackgroundProbability / colorProbability * backgroundProbability;
}