#include "BackgroundSeparator.h"
#include "Presenter.h"
#include <iostream>

BackgroundSeparator::BackgroundSeparator(Mat& img, vector<Rect> puzzelCandidateAreas, Mat& edgeMap)
	:image(img), edgeMap(edgeMap)
{
	memset(qubeHistogram, 0, sizeof(float) * TOTAL_QUBE_SIZE);
	memset(qubeBackgroundHistogram, 0, sizeof(float) * TOTAL_QUBE_SIZE);
	initializeHistogram(puzzelCandidateAreas);

	for (int i = 1; i <= 15; i++)
	{
		tune();
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
			getQubeBackgroundBin(p) += tmp;
			c += tmp;
			getQubeColorBin(p) += 1;
		}
	}

	normalizeBackgroundHistograms(c);
	normalizeHistograms(&qubeHistogram[0][0][0], image.cols * image.rows);
}

void update(unsigned char* p_dest, int x)
{
	*(p_dest + x) = *(p_dest + x) * 0.5 + *p_dest * 0.5;
}

void update(cv::Mat& probablityMap, int i, int x, unsigned char& max, unsigned char& min, unsigned int& acc)
{
	auto tmp = probablityMap.at<unsigned char>(i, x);
	if (tmp > max)
	{
		max = tmp;
	}
	if (tmp < min)
	{
		min = tmp;
	}
	acc += tmp;
}

void BackgroundSeparator::MatchToEdges(Mat& probablityMap)
{
	Mat tmp = probablityMap.clone();

	for (int i = EDGE_MATCH_PADDING; i < image.rows - EDGE_MATCH_PADDING; ++i)
	{
		for (int j = EDGE_MATCH_PADDING; j < image.cols - EDGE_MATCH_PADDING; ++j)
		{
			bool processX = false;
			bool processY = false;
			
			int startX = ProcessXMinus(j, i, processX);
			int endX = ProcessXPlus(j, i, processX);
			int startY =  ProcessYMinus(i, j, processY);
			int endY = ProcessYPlus(i, j, processY);

			UpdateMap(processX, startX, endX, probablityMap, i, processY, startY, endY, j, tmp);
		}
	}
	probablityMap = tmp;
}

void BackgroundSeparator::UpdateMap(bool processX, int startX, int endX, cv::Mat& sourceMap, int i, bool processY, int startY, int endY, int j, cv::Mat& destinationMap)
{
	unsigned int acc = 0;
	int k = 0;
	unsigned char min = 255;
	unsigned char max = 0;

	if (processX)
	{
		for (int x = startX; x <= endX; x++)
		{
			update(sourceMap, i, x, max, min, acc);
		}
		k += endX - startX + 1;
	}
	if (processY)
	{
		for (int y = startY; y <= endY; y++)
		{
			update(sourceMap, y, j, max, min, acc);
		}
		k += endY - startY + 1;
	}
	if (processX || processY)
	{
		unsigned char avg = acc / k;
		destinationMap.at<unsigned char>(i, j) = max - avg > avg - min ? min : max;;
	}
}

int BackgroundSeparator::ProcessYPlus(int i, int j, bool& processY)
{
	int endY = i + EDGE_MATCH_PADDING;
	for (int y1 = i; y1 <= i + EDGE_MATCH_PADDING; y1++)
	{
		if (edgeMap.at<unsigned char>(y1, j) != 0)
		{
			endY = y1;
			processY = true;
			break;
		}
	}
	return endY;
}

int BackgroundSeparator::ProcessYMinus(int i, int j, bool& processY)
{
	int startY = i - EDGE_MATCH_PADDING;
	for (int y1 = i; y1 >= i - EDGE_MATCH_PADDING; y1--)
	{
		if (edgeMap.at<unsigned char>(y1, j) != 0)
		{
			startY = y1;
			processY = true;
			break;
		}
	}
	return startY;
}

int BackgroundSeparator::ProcessXPlus(int j, int i, bool& processX)
{
	int endX = j + EDGE_MATCH_PADDING;
	for (int x1 = j; x1 <= j + EDGE_MATCH_PADDING; x1++)
	{
		if (edgeMap.at<unsigned char>(i, x1) != 0)
		{
			endX = x1;
			processX = true;
			break;
		}
	}
	return endX;
}

int BackgroundSeparator::ProcessXMinus(int j, int i, bool& processX)
{
	int startX = j - EDGE_MATCH_PADDING;
	for (int x1 = j; x1 >= j - EDGE_MATCH_PADDING; x1--)
	{
		if (edgeMap.at<unsigned char>(i, x1) != 0)
		{
			startX = x1;
			processX = true;
			break;
		}
	}
	return startX;
}

void BackgroundSeparator::tune()
{
	Mat probablityMap = getMatchMap();
	MatchToEdges(probablityMap);
	blur(probablityMap, probablityMap, Size(3, 3));
	auto filter = [&](Point2i& p) 
	{
		unsigned char backgProbability = probablityMap.at<unsigned char>(p);
		if (backgProbability > 160) return 1.0;
		if (backgProbability < 80) return 0.0;
		return 0.4 + 0.35 * (backgProbability - 80) / 80;
	};
	buildHistogram(filter);
}

Mat BackgroundSeparator::getMatchMap()
{
	Mat probablityMap = Mat::zeros(image.rows, image.cols, CV_8UC1);
	Vec3b* p_source;
	unsigned char* p_dest;
	for (int i = 0; i < image.rows; ++i)
	{
		p_source = image.ptr<Vec3b>(i);
		p_dest = probablityMap.ptr<unsigned char>(i);
		for (int j = 0; j < image.cols; ++j, ++p_source, ++p_dest)
		{
			*p_dest = scorePoint(p_source) * 255;
		}
	}
	return probablityMap;
}

void BackgroundSeparator::showImg(const char* name)
{
	Mat probablityMap = getMatchMap();
	Presenter::ShowScaledImage(name, probablityMap);
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

Mat BackgroundSeparator::getBackgroundMap(Mat& input)
{
	Mat output = Mat::zeros(input.rows, input.cols, CV_8UC1);
	int count = 0;
	float score = 0.0;
	Vec3b* p_source;
	unsigned char* p_dest;
	for (int i = 0; i < input.rows; ++i)
	{
		p_source = input.ptr<Vec3b>(i);
		p_dest = output.ptr<unsigned char>(i);
		for (int j = 0; j < input.cols; ++j, ++p_source, ++p_dest)
		{
			float nondebackgroundProbability = (1 - scorePoint(p_source));
			if (!isnan(nondebackgroundProbability))
			{
				*p_dest = nondebackgroundProbability * 255;
			}
		}
	}
	return output;
}