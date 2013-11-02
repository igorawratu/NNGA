#ifndef SFOBSTACLESIMULATION_H
#define SFOBSTACLESIMULATION_H

#include "sfsimulation.h"

class SFObstacleSimulation : public SFSimulation
{
public:
    SFObstacleSimulation(double _rangefinderRadius, uint _numAgents, uint _numCycles, uint _cyclesPerDecision, uint _cyclesPerSecond, Solution* _solution, ResourceManager* _resourceManager, int _seed);
    SFObstacleSimulation(const SFObstacleSimulation& other);
    virtual ~SFObstacleSimulation();
    virtual Simulation* getNewCopy();
    virtual bool initialise();
};

#endif