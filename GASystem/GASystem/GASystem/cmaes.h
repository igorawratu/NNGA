#ifndef CMAES_H
#define CMAES_H

#include "geneticalgorithm.h"
#include "cmaesparameters.h"
#include "nnchromosome.h"

#include "common.h"
#include "workstatus.h"
#include <vector>
#include <map>
#include <mpi.h>

#include <boost/random.hpp>
#include <boost/generator_iterator.hpp>

class CMAES : public GeneticAlgorithm{
public:
    CMAES(CMAESParameters _parameters);
    virtual ~CMAES();

    virtual Solution train(SimulationContainer* _simulationContainer, string _outputFileName);

private:
    void evaluateFitness(vector<Chromosome*>& _population);
    void evaluateCompetitiveFitness();
    void runDeltaCodes();
    void stopSlaves();
    void hostwork();
    void sendCompData(Solution* _solution, int _slave);
    void sendData(Solution* _solution, int _slave);
    Solution* constructCompSolution(map<int, vector<pair<uint, uint>>>& _competitiveTracklist, vector<int>& _team, vector<uint>& _position);
    bool setup();

private:
    CMAESParameters mParameters;
    uint mNumTeams;
	int* mUpdateList;
    MPI_Request *mRequests, *mTeamRequests;
    double* mRetrievedFitnesses, *mRetrievedCompetitiveFitnesses;
    int* mRetrievedTeamIDs;
    int mTotalSlaveProcs, mTotalRequests;
    volatile WorkStatus mWorkStatus;
    SimulationContainer* mSimulationContainer;
	uint mStages, mStage;
    Solution* mSavedSlaveSolution;
    std::vector<Chromosome*> mPopulation;
    std::map<int, vector<Chromosome*>> mCompetitivePopulations;

private:
    CMAES(const CMAES& _other){}
    CMAES(){}
};

#endif