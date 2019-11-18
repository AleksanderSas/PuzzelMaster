#pragma once
#include <vector>
#include "Token.h"

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

