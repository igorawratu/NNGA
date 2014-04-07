#ifndef ESP_H
#define ESP_H

#include "espsubpopulation.h"
#include "geneticalgorithm.h"
#include "simulationcontainer.h"
#include "pugixml.hpp"
#include "solution.h"
#include <mpi.h>

#include <fstream>
#include <map>
#include <vector>

using namespace std;

class ESP : public GeneticAlgorithm
{
public:
    ESP(ESPParameters _parameters);
    virtual ~ESP();

    virtual Solution train(SimulationContainer* _simulationContainer, string _outputFileName);

private:
    void setupSubpopulationStructure();
    //first neuron cache, then output
    bool createNeuralNetworkPrimitives(vector<pair<map<uint, Neuron*>, map<uint, Neuron*>>>& _output);
    bool createDeltaNeuralNetworkPrimitives(vector<pair<map<uint, Neuron*>, map<uint, Neuron*>>>& _output);
    void evaluateFitness(SimulationContainer* _simulationContainer);
    void runDeltaCodes(SimulationContainer* _simulationContainer);
    void stopSlaves();

private:
    ESPParameters mParameters;
    vector<map<uint, pair<ESPSubPopulation*, uint>>> mSubpopulations;
    double mBestFitness, mBestRealFitness;
    Solution mBestSolution;
    uint mStagnationCounter;

    map<int, vector<map<uint, Neuron*>>> mUpdateList;
    MPI_Request* mRequests;
    double* mRetrievedFitnesses;
    int mTotalSlaveProcs, mTotalRequests;
    map<uint, Solution*> mSavedSolutions;

private:
    ESP(const ESP& other){}
    ESP& operator = (const ESP& other){}
    ESP(){}
};

#endif