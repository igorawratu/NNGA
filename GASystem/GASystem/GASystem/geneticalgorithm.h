#ifndef GENETICALGORITHM_H
#define GENETICALGORITHM_H

#include <vector>
#include <string>

#include "simulationcontainer.h"

using namespace std;

class GeneticAlgorithm
{
public:
    GeneticAlgorithm(){}
    virtual ~GeneticAlgorithm(){}

    virtual Solution train(SimulationContainer* _simulationContainer)=0;
};

#endif