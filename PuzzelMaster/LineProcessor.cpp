#include "LineProcessor.h"
#include "math.h"

void LineProcessor::Process(Point2i p1, Point2i p2, function<bool(int x, int y)> processor)
{
	int x = p1.x - p2.x;
	int y = p1.y - p2.y;

	if (abs(x) > abs(y))
	{
		int xStep = p1.x < p2.x ? 1 : -1;
		double dy = 1.0 * (p2.y - p1.y) / (p2.x - p1.x) * xStep;
		double yPos = p1.y;
		for (int xPos = p1.x; xPos != p2.x && processor(xPos, yPos); xPos += xStep)
		{
			yPos += dy;
		}
	}
	else
	{
		int yStep = p1.y < p2.y ? 1 : -1;
		double dx = 1.0 * (p2.x - p1.x) / (p2.y - p1.y) * yStep;
		double xPos = p1.x;
		for (int yPos = p1.y; yPos != p2.y && processor(xPos, yPos); yPos += yStep)
		{
			xPos += dx;
		}
	}
}
