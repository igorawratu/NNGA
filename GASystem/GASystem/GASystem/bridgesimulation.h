#ifndef BRIDGESIMULATION_H
#define BRIDGESIMULATION_H

#include "simulation.h"

class BridgeSimulation : public Simulation
{
public:
    BridgeSimulation();
    
    virtual ~BridgeSimulation();

    virtual void iterate();
    virtual bool initialise();
    virtual double fitness(vector<Fitness*> _fit);
    virtual Simulation* getNewCopy();

private:

};

#endif