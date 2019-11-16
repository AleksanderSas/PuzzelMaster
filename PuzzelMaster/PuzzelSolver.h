#pragma once
#include "PuzzelRectange.h"
#include <vector>

typedef struct token
{
	//edge idx of left edge
	int pozzelRotation;
	unsigned int score;
	double leftScore;
	double upperScore;
	int row;
	PuzzelRectange* puzzel;
	struct token* left;
	struct token* upper;
	struct token* previous;
} Token;


#define MAX_HIPOTHESIS 100
class PuzzelSolver
{
public:
	void Solve(vector<PuzzelRectange*> &puzzels, int columns, int rows);
	Token* GetBest(int nth);
	void PrintHistory(int nth);

private:
	vector<Token*> CurrentHipothesis;
	vector<Token*> PreviousHipothesis;

	void Initialize(std::vector<PuzzelRectange*>& puzzels);
	void AddHipothesisForToken(Token* token, std::vector<PuzzelRectange*>& puzzels, int y, int columns);
	void TruncateHipothesis();
};

