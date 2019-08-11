#include "KMeans.h"
#include <set>
#include "math.h"

KMeans::KMeans(int heigth, int width, int MIN_SQUARE_DISTANCE, int INIT_GRID_DENSITY)
{
	this->width = width;
	this->heigth = heigth;
	this->MIN_SQUARE_DISTANCE = MIN_SQUARE_DISTANCE;
	this->INIT_GRID_DENSITY = INIT_GRID_DENSITY;

	int centriesPerRow = INIT_GRID_DENSITY;
	int stepX = (width - 100) / (centriesPerRow - 1);
	int stepY = (heigth - 100) / (centriesPerRow - 1);


	for (int x = 0; x < centriesPerRow; x++)
	{
		for (int y = 0; y < centriesPerRow; y++)
		{
			Centre* c2 = new Centre();
			c2->Point.x = 50 + stepX * x;
			c2->Point.y = 50 + stepY * y;

			centres.push_back(c2);

		}
	}

}

inline int FindNearestCentre(cv::Point p, vector<Centre*> centres)
{
	Centre* nearest = NULL;
	int distance = INT_MAX;
	int idx = 0;

	for (int i = 0; i < centres.size(); i++)
	{

	/*}
	for (vector<Centre*>::iterator point = centres.begin(); point != centres.end(); point++)
	{*/
		auto x = centres[i];
		int xDist = (p.x - x->Point.x);
		int yDist = (p.y - x->Point.y);
		int newDist = xDist * xDist + yDist * yDist;

		if (newDist < distance)
		{
			idx = i;
			distance = newDist;
			nearest = x;
		}
	}
	return idx;
}


int KMeans::Pass(vector<vector<cv::Point> > &contours)
{
	for (vector<Centre*>::iterator centre = centres.begin(); centre != centres.end(); centre++)
	{
		(*centre)->BelongingPoints.clear();
		(*centre)->CapturedContours.clear();
	}

	int* scores = new int[centres.size()];
	for (vector<vector<cv::Point>>::iterator contour = contours.begin(); contour != contours.end(); contour++)
	{
		memset(scores, 0, sizeof(int) * centres.size());

		for (vector<cv::Point>::iterator point = contour->begin(); point != contour->end(); point++)
		{
			auto nearestCentreIdx = FindNearestCentre(*point, centres);
			scores[nearestCentreIdx]++;
		}

		int idx = 0;
		for (int i = 0; i < centres.size(); i++)
		{
			if (scores[idx] < scores[i])
			{
				idx = i;
			}
		}

		//vector<cv::Point>* xxx = contour._Ptr;
		Centre* c = centres[idx];
		c->CapturedContours.push_back(contour._Ptr);
		for (vector<cv::Point>::iterator point = contour->begin(); point != contour->end(); point++)
		{
			cv::Point* p = &(*point);
			c->BelongingPoints.push_back(p);
		}
	}

	int change = 0;

	vector< vector<Centre*>::iterator> toRemove;
	for (vector<Centre*>::iterator centre = centres.begin(); centre != centres.end(); centre++)
	{
		int oldX = (*centre)->Point.x;
		int oldY = (*centre)->Point.y;

		for (vector<Point*>::iterator point = (*centre)->BelongingPoints.begin(); point != (*centre)->BelongingPoints.end(); point++)
		{
			(*centre)->Point.x += (*point)->x;
			(*centre)->Point.y += (*point)->y;
		}

		int size = (*centre)->BelongingPoints.size();
		if (size > 0)
		{
			(*centre)->Point.x /= size;
			(*centre)->Point.y /= size;

			int dy = ((*centre)->Point.y - oldY);
			int dx = ((*centre)->Point.x - oldX);
			int d = dy * dy + dx * dx;
			 change += d;
			 if (change < 0)
			 {
				 int t = 0;
			 }
		}
		else
		{
			toRemove.push_back(centre);
		}

	}

	for (auto it = toRemove.rbegin(); it != toRemove.rend(); it++)
	{
		centres.erase(*it);
	}
	toRemove.clear();

	vector<int> setToRemove;
	for (int centre1 = 0; centre1 + 1 != centres.size(); centre1++)
	{
		for (int centre2 = centre1 + 1; centre2 + 1 != centres.size(); centre2++)
		{
			auto p1 = centres[centre1]->Point;
			auto p2 = centres[centre2]->Point;
			int xDist = (p1.x - p2.x);
			int yDist = (p1.y - p2.y);
			
			int newDist = xDist * xDist + yDist * yDist;

			if (newDist < MIN_SQUARE_DISTANCE &&
				std::find(setToRemove.begin(), setToRemove.end(), centre2) == setToRemove.end()
				&& std::find(setToRemove.begin(), setToRemove.end(), centre1) == setToRemove.end())
			{
				setToRemove.push_back(centre2);
			}
		}
	}

	std::sort(setToRemove.begin(), setToRemove.end());

	for (auto it = setToRemove.rbegin(); it != setToRemove.rend(); it++)
	{
		centres.erase(centres.begin() + *it);
	}
	return change;
}

int KMeans::ComputeCentres(vector<vector<cv::Point> > &contours)
{	
	
	return Pass(contours);
	
}

void drawAxis(Mat& img, Point p, Point q, Scalar colour, const float scale = 0.2)
{
	double angle;
	double hypotenuse;
	angle = atan2((double)p.y - q.y, (double)p.x - q.x); // angle in radians
	hypotenuse = sqrt((double)(p.y - q.y) * (p.y - q.y) + (p.x - q.x) * (p.x - q.x));
	//    double degrees = angle * 180 / CV_PI; // convert radians to degrees (0-180 range)
	//    cout << "Degrees: " << abs(degrees - 180) << endl; // angle in 0-360 degrees range
		// Here we lengthen the arrow by a factor of scale
	q.x = (int)(p.x - scale * hypotenuse * cos(angle));
	q.y = (int)(p.y - scale * hypotenuse * sin(angle));
	line(img, p, q, colour, 1, LINE_AA);
	// create the arrow hooks
	p.x = (int)(q.x + 9 * cos(angle + CV_PI / 4));
	p.y = (int)(q.y + 9 * sin(angle + CV_PI / 4));
	line(img, p, q, colour, 1, LINE_AA);
	p.x = (int)(q.x + 9 * cos(angle - CV_PI / 4));
	p.y = (int)(q.y + 9 * sin(angle - CV_PI / 4));
	line(img, p, q, colour, 1, LINE_AA);
}


double getOrientation(vector<vector<cv::Point>*> &pts, Mat& img)
{

	int totalSize = 0;
	for (vector<vector<cv::Point>*>::iterator contour = pts.begin(); contour != pts.end(); contour++)
	{
		totalSize += (*contour)->size();
	}
	//Construct a buffer used by the pca analysis
	int sz = static_cast<int>(totalSize);
	Mat data_pts = Mat(sz, 2, CV_64FC1);
	int offset = 0;
	for (vector<vector<cv::Point>*>::iterator contour = pts.begin(); contour != pts.end(); contour++)
	{
		for (int i = 0; i < (*contour)->size(); ++i)
		{
			data_pts.at<double>(offset + i, 0) = (*contour)->at(i).x;
			data_pts.at<double>(offset + i, 1) = (*contour)->at(i).y;
		}
		offset += (*contour)->size();
	}
	//Perform PCA analysis
	PCA pca_analysis(data_pts, Mat(), PCA::DATA_AS_ROW);
	//Store the center of the object
	Point cntr = Point(static_cast<int>(pca_analysis.mean.at<double>(0, 0)),
		static_cast<int>(pca_analysis.mean.at<double>(0, 1)));
	//Store the eigenvalues and eigenvectors
	vector<Point2d> eigen_vecs(2);
	vector<double> eigen_val(2);
	for (int i = 0; i < 2; ++i)
	{
		eigen_vecs[i] = Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
			pca_analysis.eigenvectors.at<double>(i, 1));
		eigen_val[i] = pca_analysis.eigenvalues.at<double>(i);
	}
	// Draw the principal components
	circle(img, cntr, 3, Scalar(255, 0, 255), 2);
	Point p1 = cntr + 0.02 * Point(static_cast<int>(eigen_vecs[0].x * eigen_val[0]), static_cast<int>(eigen_vecs[0].y * eigen_val[0]));
	Point p2 = cntr - 0.02 * Point(static_cast<int>(eigen_vecs[1].x * eigen_val[1]), static_cast<int>(eigen_vecs[1].y * eigen_val[1]));
	drawAxis(img, cntr, p1, Scalar(0, 255, 0), 1);
	drawAxis(img, cntr, p2, Scalar(255, 255, 0), 5);
	double angle = atan2(eigen_vecs[0].y, eigen_vecs[0].x); // orientation in radians
	return angle;
}

void KMeans::DrawEigenVectors(Mat& img)
{
	for (vector<Centre*>::iterator centre = centres.begin(); centre != centres.end(); centre++)
	{
		
		getOrientation((*centre)->CapturedContours, img);
	}
}

unique_ptr< vector<cv::Point>> MergeContours(vector<vector<cv::Point>*> &contours)
{
	unique_ptr< vector<cv::Point>> ptr(new vector<cv::Point>());
	for (vector<vector<cv::Point>*>::iterator contour = contours.begin(); contour != contours.end(); contour++)
	{
		ptr->insert(ptr->end(), (*contour)->begin(), (*contour)->end());
	}
	return ptr;
}

void KMeans::drawApprovedContours(Mat& img)
{
	for (vector<Centre*>::iterator centre = centres.begin(); centre != centres.end(); centre++)
	{
		vector<vector<cv::Point>*> buffer;
		//GetCloseContours(*centre, buffer);
		for (int k = 0; k < buffer.size(); k++)
		{
			//drawContours(img, buffer, k, Scalar(50, 50, 250), 1, LINE_8, hierarchy, 0);
			//contours[i].push_back(contours[i][0]);
		}
	}
}

void GetCloseContours(Centre* centre, vector<vector<cv::Point>*> &buffer)
{
	int removed = 0;
	int count = 0;
	double dist = 0.0;
	double* dists = new double[centre->CapturedContours.size()];
	for (int i = 0; i < centre->CapturedContours.size(); i++)
	{
		int x = 0;
		int y = 0;

		auto contour = centre->CapturedContours[i];
		count += contour->size();
		for(vector<cv::Point>::iterator point = contour->begin(); point != contour->end(); point++)
		{
			/*int dx = point->x - centre->Point.x;
			int dy = point->y - centre->Point.y;*/

			x += point->x;
			y += point->y;
		}

		x /= contour->size();
		y /= contour->size();

		int dx = x - centre->Point.x;
		int dy = y - centre->Point.y;

		dist += hypot(dx, dy);
		dists[i] = hypot(dx, dy);
	}

	dist /= centre->CapturedContours.size();
	float var = 0;
	for (int i = 0; i < centre->CapturedContours.size(); i++)
	{
		var += (dist - dists[i]) * (dist - dists[i]);
	}
	var /= centre->CapturedContours.size();
	var = 1 * sqrt(var);

	for (int i = 0; i < centre->CapturedContours.size(); i++)
	{
		if (dists[i] < dist + var || centre->CapturedContours.size() < 7 )
		{
			buffer.push_back(centre->CapturedContours[i]);
		}
		else
		{
			removed++;
		}
	}
}

void KMeans::DrawBoxes(Mat& img)
{
	for (vector<Centre*>::iterator centre = centres.begin(); centre != centres.end(); centre++)
	{
		vector<vector<cv::Point>*> buffer;
		GetCloseContours(*centre, buffer);

		auto mergedContour = MergeContours(buffer);
		//auto box = boundingRect(*mergedContour);
		auto box = minAreaRect(*mergedContour);
		Scalar color = Scalar(80, 100, 120);
		//rectangle(img, box.tl(), box.br(), color, 2);

		Point2f rect_points[4];
		box.points(rect_points);
		for (int j = 0; j < 4; j++)
		{
			line(img, rect_points[j], rect_points[(j + 1) % 4], color);
		}
	}
}

bool vComparer(Point2f i, Point2f j) { return (i.y < j.y); }
bool hComparer(Point2f i, Point2f j) { return (i.x < j.x); }

Point2f GetVector(Point2f& a, Point2f& b)
{
	auto x = a.x - b.x;
	auto y = a.y - b.y;
	return Point2f(x, y);
}

Point2f GetNormalizedVector(Point2f& a, Point2f& b)
{
	auto x = a.x - b.x;
	auto y = a.y - b.y;
	double len = hypot(x, y);
	y /= len;
	x /= len;
	return Point2f(x, y);
}

float crossProduct(Point2f& a, Point2f& centre, Point2f& b)
{
	int x1 = centre.x = a.x;
	int y1 = centre.y = a.y;

	int x2 = centre.x = b.x;
	int y2 = centre.y = b.y;

	return abs(x1 * y2 - x2 * y1);
}

double decide(Point2f& v1, Point2f& v2, double treshold = 0.997)
{
	double result = abs(v1.cross(v2));
	return result >= treshold ? result : 0.0;
}

float GetHarrisScore(Point2f &p, Mat& xDeriv, Mat& yDeriv, float k = 0.04)
{
	float dx = abs(xDeriv.at<short>(p));
	float dy = abs(yDeriv.at<short>(p));
	return 1.0 * dx * dy - k * (dx - dy) * (dx - dy);
}

float IsMoreOrLessRectange(PuzzelRectange &candidate, Mat& xDeriv, Mat& yDeriv)
{
	Point2f v1 = GetNormalizedVector(candidate.left, candidate.upper);
	Point2f v2 = GetNormalizedVector(candidate.upper, candidate.right);
	Point2f v3 = GetNormalizedVector(candidate.right, candidate.lower);
	Point2f v4 = GetNormalizedVector(candidate.lower, candidate.left);

	double measure = decide(v1, v2) * decide(v2, v3) * decide(v3, v4) * decide(v4, v1);
	candidate.recScore = measure;
	double measure2 = GetHarrisScore(candidate.left, xDeriv, yDeriv) 
					* GetHarrisScore(candidate.lower, xDeriv, yDeriv)
					* GetHarrisScore(candidate.right, xDeriv, yDeriv) 
					* GetHarrisScore(candidate.upper, xDeriv, yDeriv);
	measure2 = sqrt(sqrt(sqrt(measure2)));
	candidate.interestScore = measure2;

	if (measure > 0.1)
	{
		//measure *= measure2;

		Point2f v1 = GetVector(candidate.left, candidate.upper);
		Point2f v2 = GetVector(candidate.upper, candidate.right);

		int a = v1.x * v1.x + v1.y * v1.y;
		int b = v2.x * v2.x + v2.y * v2.y;

		int rectangeFactor = 2;
		if (1.0 * a / b > rectangeFactor || 1.0 * b / a > rectangeFactor) //it not rectange
			return 0.0;
		
		double area = sqrt(sqrt(a * b));
		candidate.areaScore = area;
		return area * measure;
	}

	return 0.0;
}

void UpdateHitMetric(Mat& edgeMap, int xPos, int yPos, int& hit, int& miss)
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

void ComputeEdgeHits(Mat& edgeMap, Point2f p1, Point2f p2, int& hit, int& miss)
{
	int x = p1.x - p2.x;
	int y = p1.y - p2.y;

	if (abs(x) > abs(y))
	{
		if (p1.x > p2.x) swap(p1, p2);
		double dy = 1.0 * (p2.y - p1.y) / (p2.x - p1.x);
		double yPos = p1.y;
		for (int xPos = p1.x; xPos < p2.x; xPos++)
		{
			UpdateHitMetric(edgeMap, xPos, yPos, hit, miss);
			yPos += dy;
		}
	}
	else
	{
		if (p1.y > p2.y) swap(p1, p2);
		double dx = 1.0 * (p2.x - p1.x) / (p2.y - p1.y);
		double xPos = p1.x;
		for (int yPos = p1.y; yPos < p2.y; yPos++)
		{
			UpdateHitMetric(edgeMap, xPos, yPos, hit, miss);
			xPos += dx;
		}
	}
}

void ComputeEdgeHits(Mat& edgeMap, PuzzelRectange &puzzel, int& hit, int& miss)
{
	ComputeEdgeHits(edgeMap, puzzel.left, puzzel.lower, hit, miss);
	ComputeEdgeHits(edgeMap, puzzel.left, puzzel.upper, hit, miss);
	ComputeEdgeHits(edgeMap, puzzel.right, puzzel.lower, hit, miss);
	ComputeEdgeHits(edgeMap, puzzel.right, puzzel.upper, hit, miss);
}

vector<PuzzelRectange>* KMeans::FindBestRectange(vector<Point2f>& corners, Mat &xDeriv, Mat& yDeriv, Mat &edgeMap)
{
	auto vSorted = vector<Point2f>(corners);
	auto hSorted = vector<Point2f>(corners);

	sort(vSorted.begin(), vSorted.end(), vComparer);
	sort(hSorted.begin(), hSorted.end(), hComparer);

	PuzzelRectange candidate;
	PuzzelRectange best;
	vector<PuzzelRectange>* rects = new vector<PuzzelRectange>();

	for (vector<Point2f>::iterator left = hSorted.begin(); left != hSorted.end(); left++)
	{
		for (vector<Point2f>::iterator right = left + 1; right != hSorted.end(); right++)
		{
			for (vector<Point2f>::iterator upper = left + 1; upper != right; upper++)
			{
				if (right-> x == upper-> x && right->y == upper->y)
				{
					continue;
				}
				for (vector<Point2f>::iterator lower = left + 1; lower != right; lower++)
				{
					if (lower->x == upper->x && lower->y == upper->y)
					{
						continue;
					}

					if (lower->y > upper->y)
					{
						continue;
					}
					candidate.left = *left;
					candidate.right = *right;
					candidate.lower = *lower;
					candidate.upper = *upper;

					double r = IsMoreOrLessRectange(candidate, xDeriv, yDeriv);
					if (r > 0.0)
					{
						int hit = 0;
						int miss = 0;
						ComputeEdgeHits(edgeMap, candidate, hit, miss);
						if (hit + miss == 0)
						{
							continue;
						}
						candidate.hitScore = 1.0 * hit / (hit + miss);
						if (candidate.hitScore < 0.5)
						{
							continue;
						}
						r *= 1.0 * hit / (hit + miss);
						candidate.score = r;
						rects->push_back(candidate);
					}
					/*if (r > best.score)
					{
						best = candidate;
						best.score = r;
					}*/
				}
			}
		}
	}
	return rects;
}

Mat Extract(RotatedRect rect, Mat &src, int extendBySize = 0)
{
	Mat M, rotated, cropped;
	float angle = rect.angle;
	Size rect_size = rect.size;
	if (rect.angle < -45.) {
		angle += 90.0;
		swap(rect_size.width, rect_size.height);
	}
	M = getRotationMatrix2D(rect.center, angle, 1.0);
	warpAffine(src.clone(), rotated, M, src.size(), INTER_CUBIC);

	rect.center.x -= extendBySize;
	rect_size.width += extendBySize * 2;
	rect_size.height += extendBySize * 2;

	getRectSubPix(rotated, rect_size, rect.center, cropped);

	return cropped;
}

vector<pair<Mat, Mat>> KMeans::GetPuzzels(Mat &img, Mat& edgeMap)
{
	vector<pair<Mat, Mat>> puzzelAreas;
	for (vector<Centre*>::iterator centre = centres.begin(); centre != centres.end(); centre++)
	{
		vector<vector<cv::Point>*> buffer;
		GetCloseContours(*centre, buffer);

		auto mergedContour = MergeContours(buffer);
		//auto box = boundingRect(*mergedContour);
		auto box = minAreaRect(*mergedContour);
		puzzelAreas.push_back(pair<Mat, Mat>(Extract(box, img, 30), Extract(box, edgeMap, 30)));
	}
	return puzzelAreas;
}