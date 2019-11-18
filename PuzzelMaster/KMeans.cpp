#include "KMeans.h"
#include <set>
#include "math.h"
#include "IntrestingArea.h"
#include "BackgroundSeparator.h"
#include "LineProcessor.h"

KMeans::KMeans(int heigth, int width)
{
	this->width = width;
	this->heigth = heigth;

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

KMeans::~KMeans()
{
	for (Centre* c: centres)
	{
		delete c;
	}
}

inline int FindNearestCentre(cv::Point p, vector<Centre*>& centres)
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
	ResetKMeansCentries();
	MatchContoursToCentries(contours);
	int change = UpdateCentries();
	MergeTooCloseCentries();

	return change;
}

void KMeans::MergeTooCloseCentries()
{
	vector<int> toRemove;
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
				std::find(toRemove.begin(), toRemove.end(), centre2) == toRemove.end()
				&& std::find(toRemove.begin(), toRemove.end(), centre1) == toRemove.end())
			{
				toRemove.push_back(centre2);
			}
		}
	}

	std::sort(toRemove.begin(), toRemove.end());

	for (auto it = toRemove.rbegin(); it != toRemove.rend(); it++)
	{
		auto centreToRemove = centres.begin() + *it;
		delete *centreToRemove;
		centres.erase(centreToRemove);
	}
}

int KMeans::UpdateCentries()
{
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

		unsigned int size = (*centre)->BelongingPoints.size();
		if (size > 0)
		{
			(*centre)->Point.x /= size;
			(*centre)->Point.y /= size;

			int dy = ((*centre)->Point.y - oldY);
			int dx = ((*centre)->Point.x - oldX);
			int d = dy * dy + dx * dx;
			change += d;
		}
		else
		{
			toRemove.push_back(centre);
		}
	}

	for (auto it = toRemove.rbegin(); it != toRemove.rend(); it++)
	{
		delete **it;
		centres.erase(*it);
	}
	return change;
}

void KMeans::MatchContoursToCentries(std::vector<std::vector<cv::Point>>* contours)
{
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

		Centre* c = centres[idx];
		c->CapturedContours.push_back(contour._Ptr);
		for (vector<cv::Point>::iterator point = contour->begin(); point != contour->end(); point++)
		{
			cv::Point* p = &(*point);
			c->BelongingPoints.push_back(p);
		}
	}
	delete [] scores;
}

void KMeans::ResetKMeansCentries()
{
	for (vector<Centre*>::iterator centre = centres.begin(); centre != centres.end(); centre++)
	{
		(*centre)->BelongingPoints.clear();
		(*centre)->CapturedContours.clear();
	}
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

static double ComputeContourToCentreDisatnce(Centre* centre, vector<cv::Point>* contour)
{
	int x = 0;
	int y = 0;

	for (vector<cv::Point>::iterator point = contour->begin(); point != contour->end(); point++)
	{
		x += point->x;
		y += point->y;
	}

	x /= contour->size();
	y /= contour->size();

	int dx = x - centre->Point.x;
	int dy = y - centre->Point.y;

	return hypot(dx, dy);
}

void GetCloseContours(Centre* centre, vector<vector<cv::Point>*> *buffer)
{
	double avgDist = 0.0;
	double* dists = new double[centre->CapturedContours.size()];
	for (unsigned int i = 0; i < centre->CapturedContours.size(); i++)
	{
		auto contour = centre->CapturedContours[i];
		double dist = ComputeContourToCentreDisatnce(centre, contour);
		avgDist += dist;
		dists[i] = dist;
	}

	avgDist /= centre->CapturedContours.size();
	float variance = 0;
	for (int i = 0; i < centre->CapturedContours.size(); i++)
	{
		variance += (avgDist - dists[i]) * (avgDist - dists[i]);
	}
	variance /= centre->CapturedContours.size();
	variance = sqrt(variance);

	for (int i = 0; i < centre->CapturedContours.size(); i++)
	{
		if (dists[i] < avgDist + variance || centre->CapturedContours.size() < 7 )
		{
			buffer->push_back(centre->CapturedContours[i]);
		}
	}
	delete [] dists;
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
	M = getRotationMatrix2D(rect.center, angle, 1.0);
	warpAffine(src.clone(), rotated, M, src.size(), INTER_CUBIC);

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
		//TODO: delete
		vector<vector<cv::Point>*> *buffer = new vector<vector<cv::Point>*>();
		GetCloseContours(*centre, buffer);

		auto mergedContour = MergeContours(buffer);
		auto box = minAreaRect(*mergedContour);
		if (box.size.height < MIN_RECT_SIZE || box.size.width < MIN_RECT_SIZE)
		{
			continue;
		}

		puzzelAreas.push_back(
							IntrestingArea(Extract(box, img, AREA_PADDING),
							Extract(box, edgeMap, AREA_PADDING),
							boundingRect(*mergedContour),
							id++,
							box)
		);
	}
	return puzzelAreas;
}