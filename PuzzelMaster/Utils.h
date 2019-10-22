#pragma once
#include <string>


using namespace std;

union rgbExtractor
{
	unsigned int color;
	unsigned char bytes[4];
};

class Utils
{
public:
	static void WriteColoredText(string text, unsigned int colorIdx);
	static unsigned char* GetCOlorFromTable(unsigned int idx);
};

