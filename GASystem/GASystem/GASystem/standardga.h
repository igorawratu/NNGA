#ifndef STANDARDGA_H
#define STANDARDGA_H

#include "algorithmcreator.h"
#include "crossover.h"
#include "selection.h"
#include "nnchromosome.h"
#include "solution.h"
#include "simulationcontainer.h"

#include <string>

using namespace std;

struct StandardGAParameters
{
    uint populationSize;
    uint maxGenerations;

    string nnFormatFilename;
    double stagnationMovementThreshold;
    double fitnessEpsilonThreshold;

    string mutationAlgorithm;
    map<string, double> mutationParameters;
    string crossoverAlgorithm;
    map<string, double> crossoverParameters;
    string selectionAlgorithm;

};

class StandardGA
{
public:
    StandardGA(StandardGAParameters _parameters);
    StandardGA(const StandardGA& other);
    StandardGA& operator = (const StandardGA& other);
    virtual ~StandardGA();

    virtual Solution train(SimulationContainer* _simulationContainer);

private:
    void quicksort(vector<Chromosome*>& elements, int left, int right)

private:
    StandardGAParameters mParameters;

private:
    StandardGA(){}
};

#endif