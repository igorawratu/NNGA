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

#include <string>
#include <fstream>

using namespace std;

struct StandardGAParameters
{
    uint populationSize;
    uint maxGenerations;

    string nnFormatFilename;
    double stagnationThreshold;
    double fitnessEpsilonThreshold;

    string mutationAlgorithm;
    map<string, double> mutationParameters;
    string crossoverAlgorithm;
    map<string, double> crossoverParameters;
    string selectionAlgorithm;

    uint elitismCount;

};

class StandardGA : public GeneticAlgorithm
{
public:
    StandardGA(StandardGAParameters _parameters);
    StandardGA(const StandardGA& other);
    StandardGA& operator = (const StandardGA& other);
    virtual ~StandardGA();

    virtual Solution train(SimulationContainer* _simulationContainer);

private:
    void quicksort(vector<Chromosome*>& elements, int left, int right);

private:
    StandardGAParameters mParameters;

private:
    StandardGA(){}
};

#endif