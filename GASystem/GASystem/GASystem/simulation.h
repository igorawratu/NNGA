#ifndef SIMULATION_H
#define SIMULATION_H

#include "common.h"
#include "fitness.h"
#include "resourcemanager.h"

#include <map>
#include <vector>
#include <string>

#include <btBulletDynamicsCommon.h>

using namespace std;

class Simulation
{
public:
    Simulation(uint _numCycles, uint _cyclesPerDecision){
        mNumCycles = _numCycles;
        mCyclesPerDecision = _cyclesPerDecision;
        mCycleCounter = 0;
        mInitialised = false;
    }
    virtual ~Simulation(){}

    virtual void iterate()=0;
    virtual void render()=0;
    virtual bool initialise(ResourceManager* _rm)=0;

    bool isInitialised(){
        return mInitialised;
    }

    void runFullSimulation(){
        for(uint k = 0; k < mNumCycles; k++)
            iterate();
    }

    virtual double fitness(vector<Fitness*> _fit)=0;
    virtual Simulation* getNewCopy()=0;

    const map<string, pair<btRigidBody*, string>>& getSimulationState(){
        return mWorldEntities;
    }

    
protected:
    bool mInitialised;
    uint mNumCycles;
    uint mCyclesPerDecision;
    uint mCycleCounter;
    map<string, pair<btRigidBody*, string>> mWorldEntities;
};

#endif