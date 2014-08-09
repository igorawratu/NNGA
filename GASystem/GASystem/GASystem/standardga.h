#ifndef STANDARDGA_H
#define STANDARDGA_H

#include "geneticalgorithm.h"

#include "selectionfactory.h"
#include "crossoverfactory.h"
#include "crossover.h"
#include "selection.h"
#include "nnchromosome.h"
#include "solution.h"
#include "simulationcontainer.h"
#include "pugixml.hpp"
#include "common.h"
#include "standardgaparameters.h"
#include "workstatus.h"

#include <mpi.h>
#include <string>
#include <fstream>
#include <windows.h>

using namespace std;

class StandardGA : public GeneticAlgorithm
{
public:
    StandardGA(StandardGAParameters _parameters);
    StandardGA(const StandardGA& other);
    StandardGA& operator = (const StandardGA& other);
    virtual ~StandardGA();

    virtual Solution train(SimulationContainer* _simulationContainer, string _outputFileName);

private:
    void quicksort(vector<Chromosome*>& elements, int left, int right);
	void hostwork();
	void stopSlaves();
	void sendData(Solution& _solution, int _slave);
	void evaluatePopulation(vector<Chromosome*>& _population);

private:
    StandardGAParameters mParameters;

	int* mUpdateList;
    MPI_Request* mRequests;
    double* mRetrievedFitnesses, *mRetrievedCompetitiveFitnesses;
    int* mRetrievedTeamIDs;
    int mTotalSlaveProcs, mTotalRequests;
    volatile WorkStatus mWorkStatus;
    SimulationContainer* mSimulationContainer;
	uint mStages, mStage;
	vector<Chromosome*> mPopulation;

private:
    StandardGA(){}
};

#endif