#include "IntrestingArea.h"

//IntrestingArea::IntrestingArea() :
//	AreaImage(Mat()),
//	EdgeMap(Mat()),
//	contours(vector<vector<cv::Point>*>())
//{}
//
//
IntrestingArea:: IntrestingArea(Mat areaImage, Mat edgeMap, vector<vector<cv::Point>*> *contours):
	AreaImage(areaImage), 
	EdgeMap(edgeMap), 
	contours(contours)
{}
//
//IntrestingArea::IntrestingArea(IntrestingArea&& ia) : AreaImage(ia.AreaImage),
//EdgeMap(ia.EdgeMap),
//contours(ia.contours)
//{
//
//}
//
//IntrestingArea::IntrestingArea(IntrestingArea& ia) : AreaImage(ia.AreaImage),
//EdgeMap(ia.EdgeMap),
//contours(ia.contours)
//{
//
//}

