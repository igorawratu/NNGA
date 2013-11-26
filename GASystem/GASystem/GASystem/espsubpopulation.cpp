#include "espsubpopulation.h"

ESPSubPopulation::ESPSubPopulation(ESPParameters _parameters, pugi::xml_node* _root){
    mParameters = _parameters;
    mCrossoverAlgorithm = CrossoverFactory::instance().create(mParameters.crossoverAlgorithm);
    mSelectionAlgorithm = SelectionFactory::instance().create(mParameters.selectionAlgorithm);

    for(uint k = 0; k < mParameters.populationSize; ++k){
        mUnevaluatedSubpopulation.push_back(new ESPChromosome());
        if(!mUnevaluatedSubpopulation[k]->initialize(_root))
            cout << "Error initializing ESP Subpopulation" << endl;
        mEvaluationCounter.push_back(0);
    }
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
    for(uint k = 0; k < _other.mUnevaluatedSubpopulation.size(); ++k)
        mUnevaluatedSubpopulation.push_back(_other.mUnevaluatedSubpopulation[k]->clone());
    for(uint k = 0; k < _other.mSubpopulation.size(); ++k)
        mSubpopulation.push_back(_other.mSubpopulation[k]->clone());
    mEvaluationCounter = _other.mEvaluationCounter;
    mParameters = _other.mParameters;

    mCrossoverAlgorithm = CrossoverFactory::instance().create(mParameters.crossoverAlgorithm);
    mSelectionAlgorithm = SelectionFactory::instance().create(mParameters.selectionAlgorithm);
}

ESPSubPopulation& ESPSubPopulation::operator = (const ESPSubPopulation& _other){
    for(uint k = 0; k < _other.mUnevaluatedSubpopulation.size(); ++k)
        mUnevaluatedSubpopulation.push_back(_other.mUnevaluatedSubpopulation[k]->clone());
    for(uint k = 0; k < _other.mSubpopulation.size(); ++k)
        mSubpopulation.push_back(_other.mSubpopulation[k]->clone());
    mEvaluationCounter = _other.mEvaluationCounter;
    mParameters = _other.mParameters;

    mCrossoverAlgorithm = CrossoverFactory::instance().create(mParameters.crossoverAlgorithm);
    mSelectionAlgorithm = SelectionFactory::instance().create(mParameters.selectionAlgorithm);

    return *this;
}

void ESPSubPopulation::print(){
    for(uint k = 0; k < mSubpopulation.size(); ++k){
        cout << mSubpopulation[k]->fitness() << " | ";
    }
    cout << endl;
}

void ESPSubPopulation::generateOffspring(){
    quicksort(mSubpopulation, 0, mSubpopulation.size() - 1);
    mUnevaluatedSubpopulation = mCrossoverAlgorithm->execute(mSubpopulation, mSubpopulation.size(), mParameters.crossoverParameters, mSelectionAlgorithm);
    for(uint k = 0; k < mUnevaluatedSubpopulation.size(); ++k){
        mUnevaluatedSubpopulation[k]->mutate(mParameters.mutationAlgorithm, mParameters.mutationParameters);
        mEvaluationCounter.push_back(0);
    }
}

void ESPSubPopulation::nextGeneration(){
    assert(!mUnevaluatedSubpopulation.size());

    quicksort(mSubpopulation, 0, mSubpopulation.size() - 1);
    
    vector<Chromosome*> unselected, newPopulation;

    for(uint i = 0; i < mParameters.elitismCount; i++)
        newPopulation.push_back(mSubpopulation[i]);

    for(uint i = 0; i < mParameters.elitismCount; i++)
        mSubpopulation.erase(mSubpopulation.begin());

    mSubpopulation = mSelectionAlgorithm->execute(mSubpopulation, mParameters.populationSize - mParameters.elitismCount, unselected);
    mSubpopulation.insert(mSubpopulation.end(), newPopulation.begin(), newPopulation.end());
    
    quicksort(mSubpopulation, 0, mSubpopulation.size() - 1);

    for(uint k = 0; k < unselected.size(); ++k)
        delete unselected[k];
}

Chromosome* ESPSubPopulation::getUnevaluatedChromosome(){
    if(mUnevaluatedSubpopulation.size() == 0)
        return NULL;
    else if(mUnevaluatedSubpopulation.size() == 1)
        return mUnevaluatedSubpopulation[0];

    boost::mt19937 rng(rand());
    boost::uniform_int<> dist(0, mUnevaluatedSubpopulation.size() - 1);
    boost::variate_generator<boost::mt19937, boost::uniform_int<>> gen(rng, dist);

    return mUnevaluatedSubpopulation[gen()];
}

Chromosome* ESPSubPopulation::getChromosome(uint _position){
    assert(_position < mSubpopulation.size());

    return mSubpopulation[_position];
}

void ESPSubPopulation::setChromosomeFitness(Neuron* _chromosome, double _fitnessVal, double _realFitnessVal){
    for(uint k = 0; k < mUnevaluatedSubpopulation.size(); ++k){
        if(_chromosome == dynamic_cast<ESPChromosome*>(mUnevaluatedSubpopulation[k])->getNeuron()){
            mUnevaluatedSubpopulation[k]->fitness() += _fitnessVal;
            mUnevaluatedSubpopulation[k]->realFitness() += _realFitnessVal;
            ++mEvaluationCounter[k];
            if(mEvaluationCounter[k] == mParameters.sampleEvaluationsPerChromosome){
                mUnevaluatedSubpopulation[k]->fitness() /= mParameters.sampleEvaluationsPerChromosome;
                mSubpopulation.push_back(mUnevaluatedSubpopulation[k]);
                mUnevaluatedSubpopulation.erase(mUnevaluatedSubpopulation.begin() + k);
                mEvaluationCounter.erase(mEvaluationCounter.begin() + k);
            }
        }
    }
}

void ESPSubPopulation::quicksort(vector<Chromosome*>& elements, int left, int right)
{
	int i = left;
	int j = right;

	Chromosome* pivot = elements[(left+ right) / 2];
	do{
		while (elements[i]->fitness() < pivot->fitness())
			i++;
		while (elements[j]->fitness() > pivot->fitness())
			j--;

		if (i <= j){
			Chromosome* temp = elements[i]; elements[i] = elements[j]; elements[j] = temp;
			i++; j--;
		}
	}while (i <= j);

	if(left < j)
		quicksort(elements, left, j);
	if(i < right)
		quicksort(elements, i, right);
}

/*void ESPSubPopulation::generateDeltaCodes(){

}

ESPChromosome* ESPSubPopulation::getDeltaCode(){

}

void ESPSubPopulation::setDeltaCodeFitness(ESPChromosome* _deltaCode){

}

void ESPSubPopulation::integrateDeltaCodes(){

}*/