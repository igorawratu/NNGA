#include "simulationcontainer.h"

SimulationContainer::SimulationContainer(Simulation* _sim){
    mSim = _sim;
}

SimulationContainer::~SimulationContainer(){
    if(mSim){
        delete mSim;
        mSim = 0;
    }
}

void SimulationContainer::resetSimulation(){
    Simulation* temp = mSim->getNewCopy();
    delete mSim;
    mSim = temp;
}

void SimulationContainer::setSolution(Solution* _solution){
    mSim->setSolution(_solution);
}

void SimulationContainer::runFullSimulation(Solution* _solution){
    mSim->setSolution(_solution);
    mSim->runFullSimulation();
    _solution->fitness() = mSim->fitness();
    _solution->realFitness() = mSim->realFitness();
}

void SimulationContainer::iterate(){
    mSim->iterate();
}

bool SimulationContainer::initialise(ResourceManager* _rm){
    return mSim->initialise();
}

bool SimulationContainer::isInitialised(){
    return mSim->isInitialised();
}

const map<string, Agent*>& SimulationContainer::getSimulationState(){
    return mSim->getSimulationState();
}

uint SimulationContainer::getCyclesPerSecond(){
    return mSim->getCyclesPerSecond();
}

SimulationContainer* SimulationContainer::clone(){
    Simulation* temp = mSim->getNewCopy();
    return new SimulationContainer(temp);
}

vector<string> SimulationContainer::getRemoveList(){
    return mSim->getRemoveList();
}

vector<Line> SimulationContainer::getLines(){
    return mSim->getLines();
}