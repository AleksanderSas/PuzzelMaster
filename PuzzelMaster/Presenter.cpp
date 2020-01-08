#include "Presenter.h"

void Presenter::ShowScaledImage(string& name, Mat image, int maxCols, int maxRows)
{
	PrepareImage(image, maxCols, maxRows);

	imshow(name, image);
	waitKey(1);
}

void Presenter::PrepareImage(cv::Mat& image, int maxCols, int maxRows)
{
	if (image.cols <= maxCols && image.rows <= maxRows)
		return;

	if (image.rows <= maxCols && image.cols <= maxRows)
	{
		Mat imageTmp = image.t();
		image = imageTmp;
		return;
	}

	float factor = min(1.0 * maxCols / image.cols, 1.0 * maxRows / image.rows);
	float FlippedFactor = min(1.0 * maxCols / image.rows, 1.0 * maxRows / image.cols);
	if (factor < FlippedFactor)
	{
		factor = FlippedFactor;
		Mat imageTmp = image.t();
		image = imageTmp;
	}
	Mat output;
	blur(image, output, Size(3, 3));
	resize(output, image, Size(image.cols * factor, image.rows * factor));
}

void Presenter::ShowScaledImage(const char* name, Mat image, int maxCols, int maxRows)
{
	string _name(name);
	ShowScaledImage(_name, image, maxCols, maxRows);
}