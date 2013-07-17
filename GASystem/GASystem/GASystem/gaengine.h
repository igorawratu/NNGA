#ifndef GAENGINE_H
#define GAENGINE_H

#include <string>

#include "common.h"
#include "solution.h"
#include "geneticalgorithm.h"
#include "simulationcontainer.h"
#include "algorithmcreator.h"

using namespace std;

class GAEngine
{
public:
    GAEngine();
    ~GAEngine();

    Solution train(GeneticAlgorithm* _geneticAlgorithm, SimulationContainer* _simulation);

private:
    GAEngine(const GAEngine& other){}
    GAEngine& operator = (const GAEngine& other){}
    

};

#endif