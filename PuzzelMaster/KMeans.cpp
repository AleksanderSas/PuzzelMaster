#include "KMeans.h"
#include <set>
#include "math.h"
#include "IntrestingArea.h"
#include "BackgroundSeparator.h"
#include "LineProcessor.h"

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


int KMeans::Pass(vector<vector<cv::Point> > *contours)
{
	for (vector<Centre*>::iterator centre = centres.begin(); centre != centres.end(); centre++)
	{
		(*centre)->BelongingPoints.clear();
		(*centre)->CapturedContours.clear();
	}

	int* scores = new int[centres.size()];
	for (vector<vector<cv::Point>>::iterator contour = contours->begin(); contour != contours->end(); contour++)
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

unique_ptr< vector<cv::Point>> MergeContours(vector<vector<cv::Point>*> *contours)
{
	unique_ptr< vector<cv::Point>> ptr(new vector<cv::Point>());
	for (vector<vector<cv::Point>*>::iterator contour = contours->begin(); contour != contours->end(); contour++)
	{
		ptr->insert(ptr->end(), (*contour)->begin(), (*contour)->end());
	}
	return ptr;
}

void GetCloseContours(Centre* centre, vector<vector<cv::Point>*> *buffer)
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
			buffer->push_back(centre->CapturedContours[i]);
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
		GetCloseContours(*centre, &buffer);
		auto mergedContour = MergeContours(&buffer);
		auto box = minAreaRect(*mergedContour);
		Scalar color = Scalar(80, 100, 120);

		Point2f rect_points[4];
		box.points(rect_points);
		for (int j = 0; j < 4; j++)
		{
			line(img, rect_points[j], rect_points[(j + 1) % 4], color);
		}
	}
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

vector<IntrestingArea> KMeans::GetPuzzels(Mat &img, Mat& edgeMap)
{
	vector<IntrestingArea> puzzelAreas;
	int id = 0;
	for (vector<Centre*>::iterator centre = centres.begin(); centre != centres.end(); centre++)
	{
		vector<vector<cv::Point>*> *buffer = new vector<vector<cv::Point>*>();
		GetCloseContours(*centre, buffer);

		auto mergedContour = MergeContours(buffer);
		auto box = minAreaRect(*mergedContour);
		int padding = 10;
		puzzelAreas.push_back(IntrestingArea(Extract(box, img, padding), Extract(box, edgeMap, padding), buffer, boundingRect(*mergedContour), id++));
	}
	return puzzelAreas;
}