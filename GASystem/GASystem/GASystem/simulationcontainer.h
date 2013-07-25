#ifndef SIMULATIONCONTAINER_H
#define SIMULATIONCONTAINER_H

#include "solution.h"
#include "simulation.h"
#include "fitness.h"

class SimulationContainer
{
public:
    //the sim container does NOT delete the pointers passed here, whoever created these must delete them
    SimulationContainer(Simulation* _sim, vector<Fitness*> _fit){
        mFitnessFunctions = _fit;
        mSim = _sim;
    }
    virtual ~SimulationContainer(){}

    virtual void resetSimulation()=0;

    void runFullSimulation(Solution* solution){
        mSim->runFullSimulation();
        solution->fitness() = mSim->fitness(mFitnessFunctions);
    }

    void iterate(){
        mSim->iterate();
    }

    void renderCurrentSimulationState(){
        mSim->render();
    }

protected:
    Simulation* mSim;
    vector<Fitness*> mFitnessFunctions;

};

#endif