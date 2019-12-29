#pragma once
#include <map>
#include <vector>
#include "PuzzelRectange.h"

using namespace std;

class FeatureVectorCache
{
public:
	FeatureVectorCache(vector<PuzzelRectange*>& puzzels);
	~FeatureVectorCache();
	pair<double, int> GetFromCashe(int id1, int id2, int rotation1, int rotation2);

private:
	int puzzelNr;
	map<int, int> id2Nr;
	pair<double, int>* featureVectorCashe;
	pair<double, int>* Get(int i, int j);
};

