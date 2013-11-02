#ifndef SFOBSTACLEFIELDSIMULATION_H
#define SFOBSTACLEFIELDSIMULATION_H

#include "sfsimulation.h"

class SFObstaclefieldSimulation : public SFSimulation
{
public:
    SFObstaclefieldSimulation(double _rangefinderRadius, uint _numAgents, uint _numCycles, uint _cyclesPerDecision, uint _cyclesPerSecond, Solution* _solution, ResourceManager* _resourceManager, int _seed);
    SFObstaclefieldSimulation(const SFObstaclefieldSimulation& other);
    virtual ~SFObstaclefieldSimulation();
    virtual Simulation* getNewCopy();
    virtual bool initialise();
};

#endif