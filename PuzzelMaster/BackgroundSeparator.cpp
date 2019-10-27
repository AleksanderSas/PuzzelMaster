#include "BackgroundSeparator.h"

BackgroundSeparator::BackgroundSeparator(Mat& img, vector<IntrestingArea> puzzelCandidateAreas)
	:image(img)
{
	memset(qubeHistogram, 0, sizeof(float) * TOTAL_QUBE_SIZE);
	memset(qubeBackgroundHistogram, 0, sizeof(float) * TOTAL_QUBE_SIZE);
	initializeHistogram(puzzelCandidateAreas);

	for (int i = 1; i <= 15; i++)
	{
		tune();
	}
	showImg("backgrounfMap_15");
	waitKey(1);
}

bool isNotInsidePuzzelCandidate(Point2i &p, vector<IntrestingArea>& puzzelCandidateAreas)
{
	 for (auto it = puzzelCandidateAreas.begin(); it != puzzelCandidateAreas.end(); it++)
	 {
		 if (it->OriginRectange.contains(p))
			 return false;
	 }
	 return true;
}

void BackgroundSeparator::initializeHistogram(vector<IntrestingArea>& puzzelCandidateArea)
{
	auto filter = [&](Point2i& p) {return isNotInsidePuzzelCandidate(p, puzzelCandidateArea); };
	buildHistogram(filter);
}

void BackgroundSeparator::buildHistogram(function<bool(Point2i&)> filter)
{
	int c = 0;
	Vec3b* p;
	for (int i = 0; i < image.rows; ++i)
	{
		p = image.ptr<Vec3b>(i);
		for (int j = 0; j < image.cols; ++j, p++)
		{
			Point2i point(j, i);
			if (filter(point))
			{
				getQubeBackgroundBin(p) += 1;
				c++;
			}
			getQubeColorBin(p) += 1;
		}
	}

	normalizeBackgroundHistograms(c);
	normalizeHistograms(&qubeHistogram[0][0][0], image.cols * image.rows);
}

void BackgroundSeparator::tune()
{
	Mat img = getMatchMap();
	blur(img, img, Size(3, 3));
	auto filter = [&](Point2i& p) 
	{
		unsigned char ppp = img.at<unsigned char>(p);
		return ppp > 110; 
	};
	buildHistogram(filter);
}

Mat BackgroundSeparator::getMatchMap()
{
	Mat img = Mat::zeros(image.rows, image.cols, CV_8UC1);
	Vec3b* p_source;
	unsigned char* p_dest;
	for (int i = 0; i < image.rows; ++i)
	{
		p_source = image.ptr<Vec3b>(i);
		p_dest = img.ptr<unsigned char>(i);
		for (int j = 0; j < image.cols; ++j, ++p_source, ++p_dest)
		{
			*p_dest = scorePoint(p_source) * 255;
		}
	}
	return img;
}

void BackgroundSeparator::showImg(const char* name)
{
	Mat img = getMatchMap();
	imshow(name, img);
}

float& BackgroundSeparator::getQubeBackgroundBin(Vec3b *p)
{
	int cmp1 = p->operator[](0) / QUBE_BIN;
	int cmp2 = p->operator[](1) / QUBE_BIN;
	int cmp3 = p->operator[](2) / QUBE_BIN;
	return qubeBackgroundHistogram[cmp1][cmp2][cmp3];
}

float& BackgroundSeparator::getQubeColorBin(Vec3b* p)
{
	int cmp1 = p->operator[](0) / QUBE_BIN;
	int cmp2 = p->operator[](1) / QUBE_BIN;
	int cmp3 = p->operator[](2) / QUBE_BIN;
	return qubeHistogram[cmp1][cmp2][cmp3];
}

void BackgroundSeparator::normalizeBackgroundHistograms(int pointCount)
{
	normalizeHistograms(&qubeBackgroundHistogram[0][0][0], pointCount);
	backgroundProbability = 1.0 * pointCount / (image.cols * image.rows);
}

void BackgroundSeparator::normalizeHistograms(float* start, int pointCount)
{
	for (int i = 0; i < QUBE_SIZE * QUBE_SIZE * QUBE_SIZE; i++, start++)
	{
		*start /= pointCount;
	}
}

float BackgroundSeparator::scorePoint(Vec3b* pixel)
{
	float bProp = getQubeBackgroundBin(pixel);
	float colorProb = getQubeColorBin(pixel);

	return (bProp / colorProb * backgroundProbability);
}