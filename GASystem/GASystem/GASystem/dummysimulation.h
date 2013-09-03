#ifndef DUMMYSIMULATION_H
#define DUMMYSIMULATION_H

#include "common.h"
#include "simulation.h"
#include "cubeagent.h"

#include <iostream>

using namespace std;

class DummySimulation : public Simulation
{
public:
    DummySimulation(uint _numCycles, uint _cyclesPerDecision, uint _cyclesPerSecond, ResourceManager* _resourceManager) : Simulation(_numCycles, _cyclesPerDecision, _cyclesPerSecond, 0, _resourceManager){}
    virtual ~DummySimulation(){}

    virtual void iterate(){
        mWorld->stepSimulation(1/24.f, 24, 1/24.f);
    }

    virtual double fitness(vector<Fitness*> _fit){
        double finalFitness = 0;
        map<string, double> dblAcc;
        map<string, long> intAcc;
        map<string, vector3> pos;

        for(uint k = 0; k < _fit.size(); k++)
            finalFitness += _fit[k]->evaluateFitness(pos, dblAcc, intAcc);

        return finalFitness;
    }

    virtual Simulation* getNewCopy(){
        return new DummySimulation(mNumCycles, mCyclesPerDecision, mCyclesPerSecond, mResourceManager);
    }

    virtual bool initialise(){
        if(mInitialised)
            return true;    
        
        mWorldEntities["Cube"] = new CubeAgent(vector3(10, 10, 10), vector3(10, 10, 10));
        if(!mWorldEntities["Cube"]->initialise("cube.mesh", vector3(1, 1, 1), btQuaternion(0, 0, 0, 1), mResourceManager, vector3(0, 0, 0), 0))
            return false;
        mWorld->addRigidBody(mWorldEntities["Cube"]->getRigidBody());
        
        mInitialised = true;
        
        return true;
    }

};

#endif