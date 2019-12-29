#pragma once
#include <vector>
#include "Token.h"
#include "FeatureVectorCache.h"

#define MAX_HIPOTHESIS 5000
class PuzzelSolver
{
public:
	void Solve(vector<PuzzelRectange*> &puzzels, int columns, int rows);
	Token* GetBest(int nth);
	void PrintHistory(int nth);
	void RemoveDuplicateds();

private:
	FeatureVectorCache *cache;
	vector<Token*> CurrentHipothesis;
	vector<Token*> PreviousHipothesis;

	void Initialize(std::vector<PuzzelRectange*>& puzzels);
	void AddHipothesisForToken(Token* token, std::vector<PuzzelRectange*>& puzzels, int y, int columns);
	void ScoreRotations(PuzzelRectange* p, Token* left, Token* upper);
	void TruncateHipothesis();
};

