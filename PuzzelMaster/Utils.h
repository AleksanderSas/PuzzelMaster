#pragma once
#include <string>

using namespace std;

class Utils
{
public:
	static void WriteColoredText(string text, unsigned int colorIdx);
	static unsigned char* GetColorFromTable(unsigned int idx);
};
