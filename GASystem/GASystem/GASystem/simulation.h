#ifndef SIMULATION_H
#define SIMULATION_H

#include "common.h"

class Simulation
{
public:
    Simulation(uint _numCycles, uint _cyclesPerDecision){
        mNumCycles = _numCycles;
        mCyclesPerDecision = _cyclesPerDecision;
        mCycleCounter = 0;
    }
    virtual ~Simulation(){}

    virtual void iterate()=0;
    virtual void render()=0;
    void runFullSimulation(){
        for(uint k = 0; k < mNumCycles; k++)
            iterate();
    }
    virtual double fitness()=0;
    

protected:
    uint mNumCycles;
    uint mCyclesPerDecision;
    uint mCycleCounter;
};

#endif