#include "IntrestingArea.h"

IntrestingArea:: IntrestingArea(Mat areaImage, Mat edgeMap, vector<vector<cv::Point>*> *contours, Rect originRectange):
	AreaImage(areaImage), 
	EdgeMap(edgeMap), 
	contours(contours),
	OriginRectange(originRectange)
{}
