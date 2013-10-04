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
    ~SimulationContainer(){
        if(mSim){
            delete mSim;
            mSim = 0;
        }
    }

    void resetSimulation(){
        Simulation* temp = mSim->getNewCopy();
        delete mSim;
        mSim = temp;
    }

    void setSolution(Solution* _solution){
        mSim->setSolution(_solution);
    }

    void runFullSimulation(Solution* _solution){
        mSim->setSolution(_solution);
        mSim->runFullSimulation();
        _solution->fitness() = mSim->fitness(mFitnessFunctions);
    }
    

    void iterate(){
        mSim->iterate();
    }

    bool initialise(ResourceManager* _rm){
        return mSim->initialise();
    }

    bool isInitialised(){
        return mSim->isInitialised();
    }

    const map<string, Agent*>& getSimulationState(){
        return mSim->getSimulationState();
    }

    uint getCyclesPerSecond(){
        return mSim->getCyclesPerSecond();
    }

    SimulationContainer* clone(){
        Simulation* temp = mSim->getNewCopy();
        return new SimulationContainer(temp, mFitnessFunctions);
    }

protected:
    Simulation* mSim;
    vector<Fitness*> mFitnessFunctions;

};

#endif