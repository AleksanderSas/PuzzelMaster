#include "BackgroundSeparator.h"

BackgroundSeparator::BackgroundSeparator(Mat& img, vector<Rect> puzzelCandidateAreas)
	:image(img)
{
	memset(qubeHistogram, 0, sizeof(float) * TOTAL_QUBE_SIZE);
	memset(qubeBackgroundHistogram, 0, sizeof(float) * TOTAL_QUBE_SIZE);
	initializeHistogram(puzzelCandidateAreas);

	for (int i = 1; i <= 15; i++)
	{
		tune();
		/*showImg(("backgrounfMap_" + to_string(i)).c_str());
		waitKey(1);*/
	}
	showImg("backgrounfMap");
	waitKey(1);
}

bool isNotInsidePuzzelCandidate(Point2i &p, vector<Rect>& puzzelCandidateAreas)
{
	 for (auto it = puzzelCandidateAreas.begin(); it != puzzelCandidateAreas.end(); it++)
	 {
		 if (it->contains(p))
			 return false;
	 }
	 return true;
}

void BackgroundSeparator::initializeHistogram(vector<Rect>& puzzelCandidateArea)
{
	auto filter = [&](Point2i& p) {return isNotInsidePuzzelCandidate(p, puzzelCandidateArea)? 1.0 : 0.0; };
	buildHistogram(filter);
}

void BackgroundSeparator::buildHistogram(function<float(Point2i&)> filter)
{
	float c = 0;
	Vec3b* p;
	for (int i = 0; i < image.rows; ++i)
	{
		p = image.ptr<Vec3b>(i);
		for (int j = 0; j < image.cols; ++j, p++)
		{
			Point2i point(j, i);
			float tmp = filter(point);
			//if (filter(point))
			{
				getQubeBackgroundBin(p) += tmp;
				c += tmp;
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
		if (ppp > 160) return 1.0;
		if (ppp < 80) return 0.0;
		return 0.4 + 0.35 * (ppp - 80) / 80;
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