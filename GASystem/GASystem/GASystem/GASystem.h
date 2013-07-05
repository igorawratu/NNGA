#ifndef GASYSTEM
#define GASYSTEM

#include <iostream>

#include "solution.h"
#include "geneticalgorithm.h"
#include "simulation.h"

#include "common.h"

using namespace std;

class GASystem
{
public:
    GASystem();
    GASystem(const GASystem& _other);
    ~GASystem();
    GASystem& operator = (const GASystem& _other);

    template<class GAType, class SimulationType>
    bool train(GAParameters _gaParameters, SimParameters _simParameters, Solution& _solution);
    
    

private:

};

#endif