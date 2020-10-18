#pragma once
#include "PuzzelRectange.h"

class Token
{
public:
	Token();
	~Token();

	shared_ptr<Token> previous;
	double leftScore;
	double upperScore;
	PuzzelRectange* puzzel;
	Token* left;
	Token* upper;

	//edge idx of left edge
	int pozzelRotation;
	unsigned int score;
	int row;

	static int counter;
};
