#ifndef NEAT_H
#define NEAT_H

#include "geneticalgorithm.h"
#include "neatchromosome.h"
#include "simulationcontainer.h"
#include "pugixml.hpp"
#include "solution.h"
#include "common.h"
#include "workstatus.h"
#include "neatparameters.h"
#include "crossover.h"
#include "selection.h"
#include "selectionfactory.h"
#include "crossoverfactory.h"

#include <mpi.h>
#include <fstream>
#include <map>
#include <vector>
#include <windows.h>
#include <boost/math/special_functions/round.hpp>
#include <string>
#include <boost/lexical_cast.hpp>

class NEAT : public GeneticAlgorithm
{
public:
    NEAT(NEATParameters _parameters, string _fileName);
    virtual ~NEAT();

    virtual Solution train(SimulationContainer* _simulationContainer, string _outputFileName);

    virtual void stopSlaves();

private:
    void quicksort(vector<Chromosome*>& elements, int left, int right);
    void quicksortRanks(vector<pair<int, double>>& elements, int left, int right);
	void hostwork();
	void sendData(Solution* _solution, int _slave);
	void evaluatePopulation(vector<Chromosome*>& _population);
    void createOffspring(vector<Chromosome*>& _offspring, Crossover* _coAlg, Selection* _selAlg);

private:
    NEATParameters mParameters;

	int* mUpdateList;
    MPI_Request* mRequests;
    double* mRetrievedFitnesses, *mRetrievedCompetitiveFitnesses;
    int* mRetrievedTeamIDs;
    int mTotalSlaveProcs, mTotalRequests;
    volatile WorkStatus mWorkStatus;
    SimulationContainer* mSimulationContainer;
	uint mStages, mStage;
    Solution* mSavedSolution;
    map<pair<long, long>, long> mMutationList;
    vector<vector<Chromosome*>> mSpecies;
    vector<Chromosome*> mPopulation;
    long mInnovation;

private:
    NEAT(const NEAT& other){}
    NEAT& operator = (const NEAT& other){}
    NEAT(){}
};

#endif