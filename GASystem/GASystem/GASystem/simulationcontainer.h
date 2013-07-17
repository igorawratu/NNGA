#ifndef SIMULATIONCONTAINER_H
#define SIMULATIONCONTAINER_H

#include "solution.h"
#include "simulation.h"

class SimulationContainer
{
public:
    SimulationContainer(){}
    virtual ~SimulationContainer(){}

    virtual void resetSimulation()=0;

    void runFullSimulation(Solution* solution){
        mSim->runFullSimulation();
        solution->fitness() = mSim->fitness();
    }

    void iterate(){
        mSim->iterate();
    }

    void renderCurrentSimulationState(){
        mSim->render();
    }

protected:
    Simulation* mSim;
};

#endif