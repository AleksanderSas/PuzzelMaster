#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/imgcodecs.hpp"
#include <iostream>
using namespace std;
using namespace cv;
const float inlier_threshold = 2.5f; // Distance threshold to identify inliers with homography check
const float nn_match_ratio = 0.8f;   // Nearest neighbor matching ratio
RNG rng2(12345);
int main2(int argc, char* argv[])
{
	/*CommandLineParser parser(argc, argv,
		"{@img1 | ../data/graf1.png | input image 1}"
		"{@img2 | ../data/graf3.png | input image 2}"
		"{@homography | ../data/H1to3p.xml | homography matrix}");*/
	Mat img1 = imread(argv[1], IMREAD_GRAYSCALE);
	Mat imgColor = imread(argv[1], IMREAD_COLOR);
	//Mat img2 = imread(parser.get<String>("@img2"), IMREAD_GRAYSCALE);
	Mat homography;
	//FileStorage fs(parser.get<String>("@homography"), FileStorage::READ);
	//fs.getFirstTopLevelNode() >> homography;
	vector<KeyPoint> kpts1, kpts2;
	Mat desc1;
	//Mat desc2;
	//Ptr<GFTTDetector> gftd = GFTTDetector::create(100, 0.0005, 20, 5, true, 0.04);
	//Ptr<BRISK> gftd = BRISK::create(100, 0, 0.0005, 20, 5, true, 0.04);
	Ptr<ORB> akaze = ORB::create();// 5, 32);
	//PYRAMI
	//cv::PyramidAdaptedFeatureDetector ppp(akaze, 2);

	//gftd->detect(img1, kpts1);
	for (int i = 0; i < kpts1.size(); i++)
	{
		kpts1[i].response = 0.01;
		kpts1[i].angle = 1.0;
		kpts1[i].class_id = 0;

	}

	akaze->compute(img1, kpts1, desc1);
	//gftd->compute(img1, kpts1, desc1);
	Mat output;

	Mat m;
	desc1.convertTo(m, CV_32FC1);
	//cvConvert(desc1, m);
	kmeans(m, 5, output, TermCriteria(cv::TermCriteria::Type::COUNT, 50, 0), 10, 0);

	//akaze->detectAndCompute(img1, noArray(), kpts1, desc1);
	//akaze->detectAndCompute(img2, noArray(), kpts2, desc2);
	BFMatcher matcher(NORM_HAMMING);
	vector< vector<DMatch> > nn_matches;
	Scalar colors[10];
	for (int i = 0; i < 10; i++)
	{
		colors[i] = Scalar(rng2.uniform(0, 255), rng2.uniform(0, 256), rng2.uniform(0, 256));
	}

	for (int i = 0; i < kpts1.size(); i++)
	{
		cv::Point p((int)kpts1[i].pt.x, (int)kpts1[i].pt.y);
		//cv::Scalar c(123, 234, 0);
		int idx = output.at<int>(i, 0);
		drawMarker(imgColor, p, colors[idx]);
	}
	//matcher.knnMatch(desc1, desc2, nn_matches, 2);
	vector<KeyPoint> matched1, matched2;
	/*for (size_t i = 0; i < nn_matches.size(); i++) {
		DMatch first = nn_matches[i][0];
		float dist1 = nn_matches[i][0].distance;
		float dist2 = nn_matches[i][1].distance;
		if (dist1 < nn_match_ratio * dist2) {
			matched1.push_back(kpts1[first.queryIdx]);
			matched2.push_back(kpts2[first.trainIdx]);
		}
	}*/
	vector<DMatch> good_matches;
	vector<KeyPoint> inliers1, inliers2;
	/*for (size_t i = 0; i < matched1.size(); i++) {
		Mat col = Mat::ones(3, 1, CV_64F);
		col.at<double>(0) = matched1[i].pt.x;
		col.at<double>(1) = matched1[i].pt.y;
		col = homography * col;
		col /= col.at<double>(2);
		double dist = sqrt(pow(col.at<double>(0) - matched2[i].pt.x, 2) +
			pow(col.at<double>(1) - matched2[i].pt.y, 2));
		if (dist < inlier_threshold) {
			int new_i = static_cast<int>(inliers1.size());
			inliers1.push_back(matched1[i]);
			inliers2.push_back(matched2[i]);
			good_matches.push_back(DMatch(new_i, new_i, 0));
		}
	}*/
	Mat res;
	//drawMatches(img1, inliers1, img2, inliers2, good_matches, res);
	imshow("akaze_result", imgColor);
	double inlier_ratio = inliers1.size() / (double)matched1.size();
	cout << "A-KAZE Matching Results" << endl;
	cout << "*******************************" << endl;
	cout << "# Keypoints 1:                        \t" << kpts1.size() << endl;
	cout << "# Keypoints 2:                        \t" << kpts2.size() << endl;
	cout << "# Matches:                            \t" << matched1.size() << endl;
	cout << "# Inliers:                            \t" << inliers1.size() << endl;
	cout << "# Inliers Ratio:                      \t" << inlier_ratio << endl;
	cout << endl;
	//imshow("result", res);
	waitKey();
	return 0;
}