#ifndef SLAVE_H
#define SLAVE_H

#include "simulation.h"
#include "simulationcontainer.h"
#include <mpi.h>
#include "solution.h"

class Slave
{
public:
    Slave(Simulation* _sim);

    void run();

private:
    SimulationContainer mSimulationContainer;
};

#endif