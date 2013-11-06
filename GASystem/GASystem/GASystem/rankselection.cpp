#include "rankselection.h"

RankSelection::RankSelection(){}
RankSelection::~RankSelection(){}

vector<Chromosome*> RankSelection::execute(vector<Chromosome*> _selectionPool, uint _selectionCount, vector<Chromosome*>& _unselected){
    assert(_selectionPool.size() >= _selectionCount);

    mRankFitness.clear();
    mMaxFit = 0;

    calculateRankFitness(_selectionPool);
    calculateMaxFitness();

    
    vector<Chromosome*> output;

    while(output.size() < _selectionCount)
        output.push_back(selectSingleChromosome(_selectionPool));

    _unselected = _selectionPool;

    return output;        
}

Chromosome* RankSelection::selectSingleChromosome(vector<Chromosome*>& _selectionPool){
    boost::mt19937 rng(rand());
    boost::uniform_real<double> dist(0, mMaxFit);
    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> gen(rng, dist);

    double selection = gen();

    int k = -1;

    while(selection > 0)
        selection -= mRankFitness[++k];

    Chromosome* output = _selectionPool[k];
    mMaxFit -= mRankFitness[k];

    _selectionPool.erase(_selectionPool.begin() + k);
    mRankFitness.erase(mRankFitness.begin() + k);

    return output;
}

void RankSelection::calculateMaxFitness(){
    for(uint k = 0; k < mRankFitness.size(); ++k)
        mMaxFit += mRankFitness[k];
}