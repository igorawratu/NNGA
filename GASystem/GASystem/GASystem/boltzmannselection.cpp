#include "boltzmannselection.h"

BoltzmannSelection::BoltzmannSelection(){
    mMaxGenerations = 200;
    mTemperature = mInitTemp = 200;
}

BoltzmannSelection::~BoltzmannSelection(){}

vector<Chromosome*> BoltzmannSelection::execute(vector<Chromosome*> _selectionPool, uint _selectionCount, vector<Chromosome*>& _unselected){
    assert(_selectionPool.size() >= _selectionCount);

    mFitnessVals.clear();
    mMaxFit = 0;

    calculateFitness(_selectionPool);
    calculateMaxFitness();

    vector<Chromosome*> output;

    while(output.size() < _selectionCount)
        output.push_back(selectChromosome(_selectionPool));

    _unselected = _selectionPool;

    if(mTemperature > mInitTemp / mMaxGenerations)
        mTemperature -= mInitTemp / mMaxGenerations;

    return output;        
}

Chromosome* BoltzmannSelection::selectChromosome(vector<Chromosome*>& _selectionPool){
    boost::mt19937 rng(rand());
    boost::uniform_real<double> dist(0, mMaxFit);
    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> gen(rng, dist);

    double selection = gen();

    int k = -1;

    while(selection > 0)
        selection -= mFitnessVals[++k];

    Chromosome* output = _selectionPool[k];
    mMaxFit -= mFitnessVals[k];

    _selectionPool.erase(_selectionPool.begin() + k);
    mFitnessVals.erase(mFitnessVals.begin() + k);

    return output;
}

void BoltzmannSelection::calculateFitness(vector<Chromosome*> _selectionPool){
    for(uint k = 0; k < _selectionPool.size(); ++k){
        double fit = 1 / (1 + pow(e, (double)k/mTemperature));
        mFitnessVals.push_back(fit);
    }
}

void BoltzmannSelection::calculateMaxFitness(){
    for(uint k = 0; k < mFitnessVals.size(); ++k)
        mMaxFit += mFitnessVals[k];
}