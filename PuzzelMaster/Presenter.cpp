#include "Presenter.h"

void Presenter::ShowScaledImage(string& name, Mat image, int maxCols, int maxRows)
{
	
	if (image.cols > maxCols || image.rows > maxRows)
	{
		Mat output;
		blur(image, output, Size(3, 3));
		float factor = min(1.0 * maxCols / image.cols, 1.0 * maxRows / image.rows);
		resize(output, image, Size(image.cols * factor, image.rows * factor));
	}

	imshow(name, image);
	waitKey(1);
}

void Presenter::ShowScaledImage(const char* name, Mat image, int maxCols, int maxRows)
{
	string _name(name);
	ShowScaledImage(_name, image);
}