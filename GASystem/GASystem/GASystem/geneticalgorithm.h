#ifndef GENETICALGORITHM_H
#define GENETICALGORITHM_H

#include <vector>
#include <string>

#include "simulationcontainer.h"
#include "filewriter.h"

using namespace std;

class GeneticAlgorithm
{
public:
    GeneticAlgorithm(){
        pTotalFitnessEvals = 20000;
    }
    virtual ~GeneticAlgorithm(){}

    virtual Solution train(SimulationContainer* _simulationContainer, string _outputFileName)=0;

    virtual void stopSlaves()=0;

protected:
    double pBestOverallFit;
    uint pNumFitEval;
    uint pTotalFitnessEvals;
};

#endif