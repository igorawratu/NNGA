#ifndef CMAES_H
#define CMAES_H

#include "geneticalgorithm.h"

class CMAES : public GeneticAlgorithm{
public:
    CMAES();
    virtual ~CMAES();

    virtual Solution train(SimulationContainer* _simulationContainer, string _outputFileName);

private:
    

private:


private:
    CMAES(const CMAES& _other){}
};

#endif