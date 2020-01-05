#include "MathUtils.h"

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

Point2f TrasformPoint(Point2f& p, Mat& transformation)
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

int squareDist(Vec3f& line, int x, int y)
{
	int tmp = line[0] * x + line[1] * y + line[2];
	int tmp2 = line[0] * line[0] + line[1] * line[1];
	return tmp * tmp / tmp2;
}

int squareDist(Point& p1, Point& p2)
{
	int x = p1.x - p2.x;
	int y = p1.y - p2.y;
	return x * x + y * y;
}
