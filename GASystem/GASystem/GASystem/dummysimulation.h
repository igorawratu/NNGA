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
    DummySimulation(uint _numCycles, uint _cyclesPerDecision, uint _cyclesPerSecond, ResourceManager* _resourceManager) : Simulation(_numCycles, _cyclesPerDecision, _cyclesPerSecond, 0, _resourceManager, TeamSetup::HET){}
    virtual ~DummySimulation(){}

    virtual void iterate(){
        mWorld->stepSimulation(1/24.f, 24, 1/24.f);

        if(mWorldEntities["Human"]->getAnimationLoop()){
            if((mCycleCounter / 120) % 2 == 0)
                mWorldEntities["Human"]->setAnimationInfo("walk", true);
            else mWorldEntities["Human"]->setAnimationInfo("run", true);
        }

        mCycleCounter++;
    }

    virtual double fitness(){
        double finalFitness = 0;
        map<string, double> dblAcc;
        map<string, long> intAcc;
        map<string, vector3> pos;

        for(uint k = 0; k < mFitnessFunctions.size(); k++)
            finalFitness += mFitnessFunctions[k]->evaluateFitness(pos, dblAcc, intAcc);

        return finalFitness;
    }
    
    virtual double realFitness(){
        return 0;
    }

    virtual Simulation* getNewCopy(){
        return new DummySimulation(mNumCycles, mCyclesPerDecision, mCyclesPerSecond, mResourceManager);
    }

    virtual bool initialise(){
        if(mInitialised)
            return true;    
        
        mWorldEntities["Human"] = new CubeAgent(vector3(10, 10, 10), vector3(10, 10, 10));
        if(!mWorldEntities["Human"]->initialise("human.mesh", vector3(1, 1, 1), btQuaternion(0, 0, 0, 1), mResourceManager, vector3(0, 0, 0), 0, 0))
            return false;
        //mWorld->addRigidBody(mWorldEntities["Cube"]->getRigidBody());
        
        mWorldEntities["Human"]->setAnimationInfo("shove", false);

        mInitialised = true;
        
        return true;
    }
	virtual ESPParameters getESPParams(string _nnFormatFile){
		ESPParameters params;
		return params;
	}
	virtual StandardGAParameters getSGAParameters(string _nnFormatFile){
		StandardGAParameters params;
		return params;
	}
    virtual CMAESParameters getCMAESParameters(string _nnFormatFile){
        CMAESParameters params;
        return params;
    }

};

#endif