#include "FeatureVectorCache.h"

FeatureVectorCache::FeatureVectorCache(vector<PuzzelRectange*>& puzzels)
{
	puzzelNr = puzzels.size();
	int cacheSize = puzzelNr * puzzelNr * 16;
	featureVectorCashe = new pair<double, int>[cacheSize];

	for (int i = 0; i < puzzelNr; i++)
	{
		PuzzelRectange* p1 = puzzels[i];
		id2Nr[p1->id] = i;

		for (int j = i + 1; j < puzzelNr; j++)
		{
			auto buffer = Get(i, j);
			PuzzelRectange* p2 = puzzels[j];

			for (int k = 0; k < 4; k++)
			{
				auto e1 = p1->edgeFeatures + k;

				for (int m = 0; m < 4; m++)
				{
					auto e2 = p2->edgeFeatures + m;
					auto s = PuzzelRectange::CompareFeatureVectors(e1, e2);
					*(buffer + k * 4 + m) = s;
				}
			}
		}
	}
}

FeatureVectorCache::~FeatureVectorCache()
{
	delete[] featureVectorCashe;
}

pair<double, int> FeatureVectorCache::GetFromCashe(int id1, int id2, int rotation1, int rotation2)
{
	int nr1 = id2Nr[id1];
	int nr2 = id2Nr[id2];
	if (nr1 > nr2)
	{
		swap(nr1, nr2);
		swap(rotation1, rotation2);
	}
	return Get(nr1, nr2)[rotation1 * 4 + rotation2];
}

pair<double, int>* FeatureVectorCache::Get(int i, int j)
{
	return featureVectorCashe + i * puzzelNr * 16 + j * 16;
}
