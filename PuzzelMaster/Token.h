#pragma once
#include "PuzzelRectange.h"

class Token
{
public:
	Token();
	~Token();

	//edge idx of left edge
	int pozzelRotation;
	unsigned int score;
	double leftScore;
	double upperScore;
	int row;
	PuzzelRectange* puzzel;
	Token* left;
	Token* upper;
	shared_ptr<Token> previous;

	static int counter;
};
