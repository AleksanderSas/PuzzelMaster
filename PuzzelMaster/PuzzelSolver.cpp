#include "PuzzelSolver.h"
#include "math.h"
#include <set>
#include "Utils.h"
#include <iostream>

//e1 is NOT null
static bool edgesMatch(edgeFeature* e1, edgeFeature* e2)
{
	return e2 == nullptr || e1->isMaleJoint ^ e2->isMaleJoint && e1->hasJoint && e2->hasJoint;
}

double leftScores[4];
double upperScores[4];
static void ScoreRotations(PuzzelRectange* p, Token* left, Token* upper)
{
	edgeFeature* e1_1 = left != nullptr? left->puzzel->edgeFeatures + ((left->pozzelRotation + 2) % 4) : nullptr;
	edgeFeature* e2_1 = upper!= nullptr? upper->puzzel->edgeFeatures + ((upper->pozzelRotation + 3) % 4) : nullptr;

	for (int i = 0; i < 4; i++)
	{
		edgeFeature* e1_2 = p->edgeFeatures + i;
		edgeFeature* e2_2 = p->edgeFeatures + ((i + 1) % 4);
		if (edgesMatch(e1_2, e1_1) && edgesMatch(e2_2, e2_1))
		{
			leftScores[i] = 0.0;
			upperScores[i] = 0.0;
			if (e1_1 != nullptr)
			{
				auto s = PuzzelRectange::CompareFeatureVectors(e1_2, e1_1);
				auto s2 = PuzzelRectange::CompareFeatureVectors(e1_1, e1_2);
				leftScores[i] = s.first + s.second;
			}
			if (e2_1 != nullptr)
			{
				auto s = PuzzelRectange::CompareFeatureVectors(e2_2, e2_1);
				upperScores[i] = s.first + s.second;
			}
		}
		else
		{
			leftScores[i] = DBL_MAX;
		}
	}
}

static set<int> getUsedPuzzels(Token* token)
{
	set<int> puzzelIds;
	while (token != nullptr)
	{
		puzzelIds.insert(token->puzzel->id);
		token = token->previous;
	}
	return puzzelIds;
}

static Token* GetLeftToken(Token* parent, int row)
{
	return parent->row == row ? parent : nullptr;
}

static Token* GetUpperToken(Token* parent, int columns)
{
	for (int i = 1; i < columns; i++)
	{
		if (parent == nullptr)
			break;
		parent = parent->previous;
	}
	return parent;
}

Token* PuzzelSolver::addToken(Token* parent, PuzzelRectange* p, int rotation, int row, int columns)
{
	Token* token = new Token();
	token->puzzel = p;
	token->pozzelRotation = rotation;
	token->score = parent->score;;
	token->upper = GetUpperToken(parent, columns);
	token->left = GetLeftToken(parent, row);
	token->previous = parent;
	token->row = row;

	return token;
}

static bool compareTokens(Token* t1, Token* t2)
{
	return t1->score < t2->score;
}

void PuzzelSolver::Solve(vector<PuzzelRectange*>& puzzels, int columns, int rows)
{
	//PuzzelRectange* puzzel = puzzels[1];
	for (PuzzelRectange* puzzel : puzzels)
	{
		for (int i = 0; i < 4; i++)
		{
			if (!puzzel->edgeFeatures[(i + 2) % 4].hasJoint)
				continue;

			Token* token = new Token();
			token->puzzel = puzzel;
			token->pozzelRotation = i;
			token->score = 0;
			token->row = 0;
			token->upper = nullptr;
			token->left = nullptr;
			token->previous = nullptr;

			PreviousHipothesis.push_back(token);
		}
	}

	//TODO: lepsza kolejunosc -> kaskadowo?
	for (int y = 0; y < rows; y++)
	{
		for (int x = (y == 0? 1 : 0); x < columns; x++)
		{
			for (Token* token : PreviousHipothesis)
			{
				auto ps = getUsedPuzzels(token);
				for (PuzzelRectange* puzzel : puzzels)
				{
					if (ps.find(puzzel->id) != ps.end())
					{
						continue;
					}
					Token* left = GetLeftToken(token, y);
					Token* upper = GetUpperToken(token, columns);
					ScoreRotations(puzzel, left, upper);
					for (int i = 0; i < 4; i++)
					{
						if (leftScores[i] > 99999)
							continue;
						Token* t = addToken(token, puzzel, i, y, columns);
						CurrentHipothesis.push_back(t);
						t->score += leftScores[i] + upperScores[i];
						t->leftScore = leftScores[i];
						t->upperScore = upperScores[i];
					}
				}
			}

			sort(CurrentHipothesis.begin(), CurrentHipothesis.end(), compareTokens);
			PreviousHipothesis.clear();
			int range = MIN(MAX_HIPOTHESIS, CurrentHipothesis.size());
			for (int i = 0; i < range; i++)
			{
				PreviousHipothesis.push_back(CurrentHipothesis[i]);
			}
			CurrentHipothesis.clear();
		}
	}
}

void mark(PuzzelRectange* p, int idx, Scalar& s)
{
	auto e = p->edgeFeatures[idx];
	auto ppp = (e.start + e.end) / 2;
	circle(p->puzzelArea, ppp, 3, s, 3);
}

Token* PuzzelSolver::GetBest(int nth)
{
	return PreviousHipothesis[nth];
}

void PuzzelSolver::PrintHistory(int nth)
{
	Token* current = PreviousHipothesis[nth];
	Token* previour = current->previous;
	string message;
	char buffer[128];
	while (previour != nullptr)
	{
		int score = current->score - previour->score;
		sprintf_s(buffer,
			"%d\t->  %d  \t L: %g (%d->%d)  U: %g   T: %d\n",
			previour->puzzel->id,
			current->puzzel->id,
			current->leftScore,
			(previour->pozzelRotation + 2) % 4,
			current->pozzelRotation,
			current->upperScore,
			score);
		message = string(buffer) + message;
		current = previour;
		previour = previour->previous;
	}
	cout << message;
}