#ifndef SFTURNBACKSIMULATION_H
#define SFTURNBACKSIMULATION_H

#include "sfsimulation.h"

class SFTurnbackSimulation : public SFSimulation
{
public:
    SFTurnbackSimulation(double _rangefinderRadius, uint _numAgents, uint _numCycles, uint _cyclesPerDecision, uint _cyclesPerSecond, Solution* _solution, ResourceManager* _resourceManager, int _seed, TeamSetup _setup);
    SFTurnbackSimulation(const SFTurnbackSimulation& other);
    virtual ~SFTurnbackSimulation();
    virtual Simulation* getNewCopy();
    virtual bool initialise();
};

#endif