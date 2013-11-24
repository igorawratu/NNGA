#include "espsubpopulation.h"

struct ESPParameters
{
    uint populationSize;
    uint maxGenerations;

    string nnFormatFilename;
    double stagnationThreshold;
    double fitnessEpsilonThreshold;

    string mutationAlgorithm;
    map<string, double> mutationParameters;
    string crossoverAlgorithm;
    map<string, double> crossoverParameters;
    string selectionAlgorithm;

    uint elitismCount;
};

ESPSubPopulation::ESPSubPopulation(ESPParameters _parameters){
    mParameters = _parameters;
    mCrossoverAlgorithm = CrossoverFactory::instance().create(mParameters.crossoverAlgorithm);
    SelectionAlgorithm = SelectionFactory::instance().create(mParameters.selectionAlgorithm);
    mEvaluationCompleted = false;

    //generate initial chromosomes here
}

ESPSubPopulation::~ESPSubPopulation(){
    for(uint k = 0; k < mUnevaluatedSubpopulation.size(); ++k)
        delete mUnevaluatedSubpopulation[k];
    for(uint k = 0; k < mSubpopulation.size(); ++k)
        delete mSubpopulation[k];

    delete mCrossoverAlgorithm;
    delete mSelectionAlgorithm;
}

ESPSubPopulation::ESPSubPopulation(const ESPSubPopulation& _other){
    for(uint k = 0; k < other.mUnevaluatedSubpopulation.size(); ++k)
        mUnevaluatedSubpopulation.push_back(new ESPChromosome(other.mUnevaluatedSubpopulation[k]);
    for(uint k = 0; k < other.mSubpopulation.size(); ++k)
        mSubpopulation.push_back(new ESPChromosome(other.mSubpopulation[k]));
    mDeltaCodes = other.mDeltaCodes;
    mEvaluationCounter = other.mEvaluationCounter;
    mDeltaEvaluationCounter = other.mDeltaEvaluationCounter;
    mParameters = other.mParameters;
    mEvaluationCompleted = other.mEvaluationCompleted;

    mCrossoverAlgorithm = CrossoverFactory::instance().create(mParameters.crossoverAlgorithm);
    SelectionAlgorithm = SelectionFactory::instance().create(mParameters.selectionAlgorithm);
}

ESPSubPopulation::ESPSubPopulation& operator = (const ESPSubPopulation& _other){
    for(uint k = 0; k < other.mUnevaluatedSubpopulation.size(); ++k)
        mUnevaluatedSubpopulation.push_back(new ESPChromosome(other.mUnevaluatedSubpopulation[k]);
    for(uint k = 0; k < other.mSubpopulation.size(); ++k)
        mSubpopulation.push_back(new ESPChromosome(other.mSubpopulation[k]));
    mDeltaCodes = other.mDeltaCodes;
    mEvaluationCounter = other.mEvaluationCounter;
    mDeltaEvaluationCounter = other.mDeltaEvaluationCounter;
    mParameters = other.mParameters;
    mEvaluationCompleted = other.mEvaluationCompleted;

    mCrossoverAlgorithm = CrossoverFactory::instance().create(mParameters.crossoverAlgorithm);
    SelectionAlgorithm = SelectionFactory::instance().create(mParameters.selectionAlgorithm);

    return *this;
}

void ESPSubPopulation::generateOffspring(){
    quicksort(mSubpopulation, 0, mSubpopulation.size() - 1);
    mUnevaluatedSubpopulation = crossoverAlgorithm->execute(mSubpopulation, mSubpopulation.size(), mParameters.crossoverParameters, mSelectionAlgorithm);

    mEvaluationCompleted = false;
}

void ESPSubPopulation::nextGeneration(){
    population.insert(mSubpopulation.end(), mUnevaluatedSubpopulation.begin(), mUnevaluatedSubpopulation.end());
    quicksort(mSubpopulation, 0, mSubpopulation.size() - 1);
    mUnevaluatedSubpopulation.clear();
    
    vector<Chromosome*> unselected;

    for(uint i = 0; i < mParameters.elitismCount; i++)
        newPopulation.push_back(mSubpopulation[i]);

    for(uint i = 0; i < mParameters.elitismCount; i++)
        population.erase(mSubpopulation.begin());

    mSubpopulation = selectionAlgorithm->execute(mSubpopulation, mParameters.populationSize - mParameters.elitismCount, unselected);

    for(uint k = 0; k < unselected.size(); ++k)
        delete unselected[k];
}

ESPChromosome* ESPSubPopulation::getUnevaluatedChromosome(){
    
}

ESPChromosome* ESPSubPopulation::getChromosome(uint _position){
    assert(_position < mSubpopulation.size());

    return mSubpopulation[_position];
}

void ESPSubPopulation::setChromosomeFitness(ESPChromosome* _chromosome, double _fitnessVal){

}

void ESPSubPopulation::generateDeltaCodes(){

}

vector<double> ESPSubPopulation::getDeltaCode(uint& _pos, bool& _complete){

}

void ESPSubPopulation::setDeltaCodeFitness(uint _pos, double _fitnessVal){

}

void ESPSubPopulation::integrateDeltaCodes(){

}

private:
    vector<ESPChromosome*> mUnevaluatedSubpopulation;
    vector<ESPChromosome*> mSubpopulation;
    vector<uint> mEvaluationCounter;
    vector<vector<double>> mDeltaCodes;
    vector<uint> mDeltaEvaluationCounter;