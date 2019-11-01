#include "Utils.h"
#include <Windows.h>
#include <iostream>

void Utils::WriteColoredText(string text, unsigned int colorIdx)
{
	HANDLE  hConsole;
	hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
	FlushConsoleInputBuffer(hConsole);
	SetConsoleTextAttribute(hConsole, colorIdx);

	cout << text;
}

static unsigned char availableColors[][3] = 
	{ 
		{12, 12, 12}, 
		{0, 55, 218},
		{19, 161, 14},
		{58, 150, 221},
		{197, 15, 31},
		{136, 23, 152},
		{193, 156, 0},
		{204, 204 , 204},
		{118, 118, 118},
		{59, 120, 255},
		{22, 198, 12},
		{97, 214, 214},
		{231, 72, 86},
		{180, 0, 158},
		{249, 141, 164},
		{242, 242, 242}
	};

unsigned char* Utils::GetCOlorFromTable(unsigned int idx)
{
	return availableColors[idx];
}
