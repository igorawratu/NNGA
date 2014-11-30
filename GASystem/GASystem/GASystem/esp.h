#ifndef ESP_H
#define ESP_H

#include "espsubpopulation.h"
#include "geneticalgorithm.h"
#include "simulationcontainer.h"
#include "pugixml.hpp"
#include "solution.h"
#include "common.h"
#include "workstatus.h"

#include <mpi.h>
#include <fstream>
#include <map>
#include <vector>
#include <windows.h>

using namespace std;

class ESP : public GeneticAlgorithm
{
public:
    ESP(ESPParameters _parameters, string _fileName);
    virtual ~ESP();

    virtual Solution train(SimulationContainer* _simulationContainer, string _outputFileName);

    virtual void stopSlaves();

private:
    void setupSubpopulationStructure();
    //first neuron cache, then output
    bool createNeuralNetworkPrimitives(vector<pair<map<uint, Neuron*>, map<uint, Neuron*>>>& _output);
    void evaluateFitness(SimulationContainer* _simulationContainer);
    void evaluateCompetitiveFitness(SimulationContainer* _simulationContainer);
    void runDeltaCodes();

    void hostwork();

    void sendCompData(Solution& _solution, int _slave);
    void sendData(Solution& _solution, int _slave);
    void saveUpdateVec(vector<pair<map<uint, Neuron*>, map<uint, Neuron*>>>& _nnPrims, int _slave);
    Solution* constructSolution(vector<pair<map<uint, Neuron*>, map<uint, Neuron*>>> _nnPrims);
    void updateCompFitness(int _slave);
    void updateFitness(int _slave, bool& _improved);

private:
    ESPParameters mParameters;
    vector<map<uint, pair<ESPSubPopulation*, uint>>> mSubpopulations;
    double mBestFitness, mBestRealFitness;
    Solution mBestSolution;
    uint mNumTeams;

    map<int, vector<map<uint, Neuron*>>> mUpdateList;
    MPI_Request* mRequests, *mTeamRequests;
    double* mRetrievedFitnesses, *mRetrievedCompetitiveFitnesses;
    int* mRetrievedTeamIDs;
    int mTotalSlaveProcs, mTotalRequests;
    map<uint, Solution*> mSavedSolutions;
    volatile WorkStatus mWorkStatus;
    SimulationContainer* mSimulationContainer;
	uint mStages, mStage;

private:
    ESP(const ESP& other){}
    ESP& operator = (const ESP& other){}
    ESP(){}
};

#endif