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
    ~SimulationContainer(){}

    void resetSimulation(){
        Simulation* temp = mSim->getNewCopy();
        delete mSim;
        mSim = temp;
    }

    void runFullSimulation(Solution* solution){
        mSim->runFullSimulation();
        solution->fitness() = mSim->fitness(mFitnessFunctions);
    }

    void iterate(){
        mSim->iterate();
    }

    bool initialise(ResourceManager* _rm){
        return mSim->initialise(_rm);
    }

    bool isInitialised(){
        return mSim->isInitialised();
    }

    const map<string, ObjectInfo>& getSimulationState(){
        return mSim->getSimulationState();
    }

    uint getCyclesPerSecond(){
        return mSim->getCyclesPerSecond();
    }

protected:
    Simulation* mSim;
    vector<Fitness*> mFitnessFunctions;

};

#endif