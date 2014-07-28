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

#include <string>
#include <fstream>
#include <omp.h>

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

private:
    StandardGAParameters mParameters;

private:
    StandardGA(){}
};

#endif