#include "espsubpopulation.h"

ESPSubPopulation::ESPSubPopulation(ESPParameters _parameters, pugi::xml_node* _root, uint _teamID){
	mTeamID = _teamID;
    mParameters = _parameters;
    mCrossoverAlgorithm = CrossoverFactory::instance().create(mParameters.crossoverAlgorithm);
    mSelectionAlgorithm = SelectionFactory::instance().create(mParameters.selectionAlgorithm);

    for(uint k = 0; k < mParameters.populationSize; ++k){
        mUnevaluatedSubpopulation.push_back(new ESPChromosome(mTeamID));
        if(!mUnevaluatedSubpopulation[k]->initialize(_root))
            cout << "Error initializing ESP Subpopulation" << endl;
        mEvaluationCounter.push_back(0);
    }

    mLastDCRadius = _parameters.deltaCodeRadius;
    mBestLastFitness = 99999999999;
    mStagnationCounter = 0;
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

    mLastDCRadius = _other.mLastDCRadius;
    mBestLastFitness = _other.mBestLastFitness;
    mStagnationCounter = _other.mStagnationCounter;
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

    mLastDCRadius = _other.mLastDCRadius;
    mBestLastFitness = _other.mBestLastFitness;
    mStagnationCounter = _other.mStagnationCounter;

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

    if(mSubpopulation[0]->fitness() < mBestLastFitness){
        mBestLastFitness = mSubpopulation[0]->fitness();
        mStagnationCounter = 0;
    }
    else mStagnationCounter++;

    if(mStagnationCounter > mParameters.stagnationThreshold){
        mStagnationCounter = 0;

        generateDeltaCodes(mLastDCRadius);        
    }
    else{
        mUnevaluatedSubpopulation = mCrossoverAlgorithm->execute(mSubpopulation, mSubpopulation.size() - mParameters.elitismCount, mParameters.crossoverParameters, mSelectionAlgorithm);
        for(uint k = 0; k < mUnevaluatedSubpopulation.size(); ++k){
            mUnevaluatedSubpopulation[k]->mutate(mParameters.mutationAlgorithm, mParameters.mutationParameters);
            mEvaluationCounter.push_back(0);
        }

        for(uint k = mParameters.elitismCount; k < mSubpopulation.size(); ++k)
            delete mSubpopulation[k];

        mSubpopulation.erase(mSubpopulation.begin() + mParameters.elitismCount, mSubpopulation.end());
    }
}

void ESPSubPopulation::nextGeneration(){
    assert(!mUnevaluatedSubpopulation.size());

    quicksort(mSubpopulation, 0, mSubpopulation.size() - 1);
}

void ESPSubPopulation::printBestWorstDistance(){
    vector<double> bestWeights = mSubpopulation[0]->getWeightData()[0][1];
    vector<double> worstWeights = mSubpopulation[mSubpopulation.size() - 1]->getWeightData()[0][1];

    double dSquared = 0;
    for(uint k = 0; k < bestWeights.size(); ++k){
        double temp = bestWeights[k] - worstWeights[k];
        dSquared += temp * temp;
    }

    dSquared = sqrt(dSquared);
    cout << dSquared << endl;
}


Chromosome* ESPSubPopulation::getUnevaluatedChromosome(){
    if(mUnevaluatedSubpopulation.size() == 0)
        return NULL;

    boost::mt19937 rng(rand());
    boost::uniform_int<> dist(0, mUnevaluatedSubpopulation.size() - 1);
    boost::variate_generator<boost::mt19937, boost::uniform_int<>> gen(rng, dist);

    int pos = gen();
    ++mEvaluationCounter[pos];

    Chromosome* out = mUnevaluatedSubpopulation[pos];

    if(mEvaluationCounter[pos] == mParameters.sampleEvaluationsPerChromosome){
        mSubpopulation.push_back(mUnevaluatedSubpopulation[pos]);
        mUnevaluatedSubpopulation.erase(mUnevaluatedSubpopulation.begin() + pos);
        mEvaluationCounter.erase(mEvaluationCounter.begin() + pos);
    }

    return out;
}

Chromosome* ESPSubPopulation::getChromosome(uint _position){
    assert(_position < mSubpopulation.size());

    return mSubpopulation[_position];
}

void ESPSubPopulation::setChromosomeFitness(Neuron* _chromosome, double _fitnessVal, double _realFitnessVal){
    for(uint k = 0; k < mUnevaluatedSubpopulation.size(); ++k){
        if(_chromosome == dynamic_cast<ESPChromosome*>(mUnevaluatedSubpopulation[k])->getNeuron()){
            mUnevaluatedSubpopulation[k]->fitness() += _fitnessVal / mParameters.sampleEvaluationsPerChromosome;
            mUnevaluatedSubpopulation[k]->realFitness() += _realFitnessVal / mParameters.sampleEvaluationsPerChromosome;
        }
    }

    for(uint k = 0; k < mSubpopulation.size(); ++k){
        if(_chromosome == dynamic_cast<ESPChromosome*>(mSubpopulation[k])->getNeuron()){
            mSubpopulation[k]->fitness() += _fitnessVal / mParameters.sampleEvaluationsPerChromosome;
            mSubpopulation[k]->realFitness() + _realFitnessVal / mParameters.sampleEvaluationsPerChromosome;
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

void ESPSubPopulation::setParameters(ESPParameters _params){
    mParameters = _params;
}

void ESPSubPopulation::generateDeltaCodes(double _deltaRange){
    mLastDCRadius = _deltaRange;
    vector<Chromosome*> newPopulation;
    mEvaluationCounter.clear();

    for(uint k = mParameters.elitismCount; k < mParameters.populationSize; ++k){
        newPopulation.push_back(mSubpopulation[0]->clone());
        
        if(_deltaRange < 0)
            _deltaRange = -_deltaRange;
        else if(_deltaRange == 0)
            _deltaRange += 0.001;

        vector<double> deltaVector;
        boost::mt19937 rng(rand());
        boost::cauchy_distribution<> dist(0, _deltaRange);
        boost::variate_generator<boost::mt19937, boost::cauchy_distribution<>> gen(rng, dist);

        vector<map<uint, vector<double>>> weightDims = mSubpopulation[0]->getWeightData();
        for(uint i = 0; i < weightDims[0][1].size(); ++i)
            deltaVector.push_back(gen());

        vector<map<uint, vector<double>>> delta;
        delta.push_back(map<uint, vector<double>>());
        delta[0][1] = deltaVector;

        newPopulation[k - mParameters.elitismCount]->addDelta(delta);
        mEvaluationCounter.push_back(0);
    }

    for(uint k = 0; k < mUnevaluatedSubpopulation.size(); ++k){
        delete mUnevaluatedSubpopulation[k];
        mUnevaluatedSubpopulation[k] = 0;
    }

    for(uint k = mParameters.elitismCount; k < mSubpopulation.size(); ++k){
        delete mSubpopulation[k];
        mSubpopulation[k] = 0;
    }

    mSubpopulation.erase(mSubpopulation.begin() + mParameters.elitismCount, mSubpopulation.end());
    mUnevaluatedSubpopulation.clear();
    mUnevaluatedSubpopulation = newPopulation;
}

uint ESPSubPopulation::getTeamID(){
    return mTeamID;
}

void ESPSubPopulation::setTeamID(uint _id){
    mTeamID = _id;
}
