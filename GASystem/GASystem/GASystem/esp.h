#ifndef ESP_H
#define ESP_H

#include "espsubpopulation.h"
#include "geneticalgorithm.h"
#include "simulationcontainer.h"
#include "pugixml.hpp"
#include "solution.h"

#include <fstream>

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

private:
    ESPParameters mParameters;
    vector<map<uint, pair<ESPSubPopulation*, uint>>> mSubpopulations;
    double mBestFitness, mBestRealFitness;
    Solution mBestSolution;
    uint mStagnationCounter;

private:
    ESP(const ESP& other){}
    ESP& operator = (const ESP& other){}
    ESP(){}
};

#endif