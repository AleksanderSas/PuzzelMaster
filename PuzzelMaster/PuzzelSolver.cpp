#include "PuzzelSolver.h"
#include "math.h"
#include <set>
#include "Utils.h"
#include <iostream>
#include "DebugFlags.h"

//e1 is NOT null
static bool edgesMatch(edgeFeature* e1, edgeFeature* e2)
{
	return e2 == nullptr || e1->isMaleJoint ^ e2->isMaleJoint && e1->hasJoint && e2->hasJoint;
}

static edgeFeature* getEdge(Token* token, int offset)
{
	return token != nullptr ? (token->puzzel->edgeFeatures + ((token->pozzelRotation + offset) % 4)) : nullptr;
}

static bool edgestMismatch(edgeFeature* e1, edgeFeature* e2)
{
	return e2 != nullptr && e1->hasJoint ^ e2->hasJoint;
}

double leftScores[4];
double upperScores[4];
void PuzzelSolver::ScoreRotations(PuzzelRectange* p, Token* left, Token* upper)
{
	edgeFeature* e1_1 = getEdge(left, 2);
	edgeFeature* leftPuzzel_upperEdeg = getEdge(left, 1);
	edgeFeature* leftPuzzel_bottomEdge = getEdge(left, 3);

	edgeFeature* e2_1 = getEdge(upper, 3);
	edgeFeature* upperPuzzel_leftEdge= getEdge(upper, 0);
	edgeFeature* upperPuzzel_rigthEdge= getEdge(upper, 2);

	for (int i = 0; i < 4; i++)
	{
		edgeFeature* e1_2 = p->edgeFeatures + i;
		edgeFeature* e2_2 = p->edgeFeatures + ((i + 1) % 4);

		edgeFeature* rigth_edge = p->edgeFeatures + ((i + 2) % 4);
		edgeFeature* bottom_edge = p->edgeFeatures + ((i + 3) % 4);
		if (edgesMatch(e1_2, e1_1) && edgesMatch(e2_2, e2_1))
		{
			leftScores[i] = 0.0;
			upperScores[i] = 0.0;
			if (e1_1 != nullptr)
			{
#if USE_CACHE_IN_SOLVER
				auto s = cache->GetFromCashe(p->id, left->puzzel->id, i, (left->pozzelRotation + 2) % 4);
#else
				auto s = PuzzelRectange::CompareFeatureVectors(e1_2, e1_1);
#endif
				leftScores[i] = s.first + s.second;
#if TAKE_INTO_ACCOUNT_EDGES
				if(edgestMismatch(e2_2, leftPuzzel_upperEdeg) ||
				  edgestMismatch(bottom_edge, leftPuzzel_bottomEdge))
				{
					leftScores[i] *= 16;
				}
#endif
			}
			if (e2_1 != nullptr)
			{
#if USE_CACHE_IN_SOLVER
				auto s = cache->GetFromCashe(p->id, upper->puzzel->id, ((i + 1) % 4), (upper->pozzelRotation + 3) % 4);
#else
				auto s = PuzzelRectange::CompareFeatureVectors(e2_2, e2_1);
#endif
				upperScores[i] = s.first + s.second;

#if TAKE_INTO_ACCOUNT_EDGES
				if (edgestMismatch(e1_2, upperPuzzel_leftEdge) ||
					edgestMismatch(rigth_edge, upperPuzzel_rigthEdge))
				{
					upperScores[i] *= 16;
				}
#endif
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
		token = token->previous.get();
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
		parent = parent->previous.get();
	}
	return parent;
}

static bool compareTokens(Token* t1, Token* t2)
{
	return t1->score < t2->score;
}

void PuzzelSolver::Solve(vector<PuzzelRectange*>& puzzels, int columns, int rows)
{
	this->columns = columns;
	this->rows = rows;

	Initialize(puzzels);
	cache = new FeatureVectorCache(puzzels);

	//TODO: lepsza kolejnosc -> kaskadowo?
	for (int y = 0; y < rows; y++)
	{
		for (int x = (y == 0 ? 1 : 0); x < columns; x++)
	    {
		    for (Token* token : PreviousHipothesis)
		    {
		    	AddHipothesisForToken(token, puzzels, y, x);
		    }

		    TruncateHipothesis();
			cout << y * columns + x + 1 << "/" << columns * rows << endl;
	    }
	}

	delete cache;
	cout << endl << "Token remains: " << Token::counter <<  endl;
}

void PuzzelSolver::AddHipothesisForToken(Token* token, std::vector<PuzzelRectange*>& puzzels, int y, int x)
{
	shared_ptr<Token> parent(token);
	auto usedPuzzelIDs = getUsedPuzzels(token);
	for (PuzzelRectange* puzzel : puzzels)
	{
		//puzzel has boundary
		if (puzzel->HasBoundaryEdge() && y != 0 && x != 0 && x != columns - 1 && y != rows - 1)
			continue;

		if (usedPuzzelIDs.find(puzzel->id) != usedPuzzelIDs.end())
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

			Token* newToken = new Token();
			newToken->puzzel = puzzel;
			newToken->pozzelRotation = i;
			newToken->upper = upper;
			newToken->left = left;
			newToken->previous = parent;
			newToken->row = y;

			newToken->score = token->score + leftScores[i] + upperScores[i];
			newToken->leftScore = leftScores[i];
			newToken->upperScore = upperScores[i];

			CurrentHipothesis.push_back(newToken);
		}
	}
}

void PuzzelSolver::TruncateHipothesis()
{
	sort(CurrentHipothesis.begin(), CurrentHipothesis.end(), compareTokens);
	PreviousHipothesis.clear();
	int range = MIN(MAX_HIPOTHESIS, CurrentHipothesis.size());
	for (int i = 0; i < range; i++)
	{
		PreviousHipothesis.push_back(CurrentHipothesis[i]);
	}
	for (int i = range; i < CurrentHipothesis.size(); i++)
	{
		delete CurrentHipothesis[i];
	}
	CurrentHipothesis.clear();
}

void PuzzelSolver::Initialize(std::vector<PuzzelRectange*>& puzzels)
{
	//PuzzelRectange* puzzel = puzzels[7];
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
			token->leftScore = 0;
			token->upperScore = 0;
			token->row = 0;
			token->upper = nullptr;
			token->left = nullptr;

			PreviousHipothesis.push_back(token);
		}
	}
}

Token* PuzzelSolver::GetBest(int nth)
{
	return PreviousHipothesis[nth];
}

int PuzzelSolver::Size() const
{
	return PreviousHipothesis.size();
}

void PuzzelSolver::PrintHistory(int nth)
{
	if (PreviousHipothesis.size() <= nth)
	{
		cout << to_string(nth) + ". hypothesis not available, only " << to_string(PreviousHipothesis.size()) + " solutions" << endl;
		return;
	}
	Token* current = PreviousHipothesis[nth];
	Token* previour = current->previous.get();
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
		previour = previour->previous.get();
	}
	cout << message;
}

bool AreTheSame(Token* h1, Token* t2)
{
	set<int> ids;
	while (t2 != NULL)
	{
		ids.insert(t2->puzzel->id);
		t2 = t2->previous.get();
	}

	while (t2 != NULL)
	{
		if (ids.find(t2->puzzel->id) == ids.end())
			return false;
		t2 = t2->previous.get();
	}
	return true;
}

void PuzzelSolver::RemoveDuplicateds()
{
	vector<int> toRemove;
	int previousScore = -1;
	for (int i = 0; i < PreviousHipothesis.size(); i++)
	{
		if (PreviousHipothesis[i]->score == previousScore && AreTheSame(PreviousHipothesis[i], PreviousHipothesis[i-1]))
		{
			toRemove.push_back(i);
		}
		previousScore = PreviousHipothesis[i]->score;
	}

	for (auto it = toRemove.rbegin(); it != toRemove.rend(); it++)
	{
		auto hypothesisToRemove = PreviousHipothesis.begin() + *it;
		PreviousHipothesis.erase(hypothesisToRemove);
	}
}
