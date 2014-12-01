#ifndef GAENGINE_H
#define GAENGINE_H

#include <string>

#include "common.h"
#include "solution.h"
#include "geneticalgorithm.h"
#include "simulationcontainer.h"
#include "mutationfactory.h"
#include "crossoverfactory.h"
#include "selectionfactory.h"

using namespace std;

class GAEngine
{
public:
    GAEngine();
    ~GAEngine();

    Solution train(GeneticAlgorithm* _geneticAlgorithm, SimulationContainer* _simulation, string _filename);

private:
    GAEngine(const GAEngine& other){}
    GAEngine& operator = (const GAEngine& other){}
    

};

#endif