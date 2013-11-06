#ifndef SIMULATIONCONTAINER_H
#define SIMULATIONCONTAINER_H

#include "solution.h"
#include "simulation.h"
#include "fitness.h"

class SimulationContainer
{
public:
    //the sim container does NOT delete the pointers passed here, whoever created these must delete them
    SimulationContainer(Simulation* _sim);
    ~SimulationContainer();

    void resetSimulation();

    void setSolution(Solution* _solution);

    void runFullSimulation(Solution* _solution);

    void iterate();

    bool initialise(ResourceManager* _rm);

    bool isInitialised();

    const map<string, Agent*>& getSimulationState();

    uint getCyclesPerSecond();

    SimulationContainer* clone();

    vector<string> getRemoveList();

    vector<Line> getLines();

protected:
    Simulation* mSim;
};

#endif