#ifndef SIMULATION_H
#define SIMULATION_H

#include "common.h"
#include "fitness.h"
#include "resourcemanager.h"
#include "solution.h"

#include <map>
#include <vector>
#include <string>

#include <btBulletDynamicsCommon.h>

using namespace std;

class Simulation
{
public:
    Simulation(uint _numCycles, uint _cyclesPerDecision, uint _cyclesPerSecond, Solution* _solution){
        mNumCycles = _numCycles;
        mCyclesPerDecision = _cyclesPerDecision;
        mCycleCounter = 0;
        mCyclesPerSecond = _cyclesPerSecond;
        mInitialised = false;

        mBroadphase = new btDbvtBroadphase();
        mCollisionConfig = new btDefaultCollisionConfiguration();
        mDispatcher = new btCollisionDispatcher(mCollisionConfig);
        mSolver = new btSequentialImpulseConstraintSolver();
        mWorld = new btDiscreteDynamicsWorld(mDispatcher, mBroadphase, mSolver, mCollisionConfig);

    }
    virtual ~Simulation(){
        for(map<string, pair<btRigidBody*, string>>::const_iterator iter = mWorldEntities.begin(); iter != mWorldEntities.end(); iter++){
            delete iter->second.first->getCollisionShape();
            
            mWorld->removeRigidBody(iter->second.first);
            delete iter->second.first->getMotionState();
            delete iter->second.first;
        }

        if(mWorld)
        {
            delete mWorld;
            mWorld = 0;
        }
        
        if(mSolver)
        {
            delete mSolver;
            mSolver = 0;
        }
        
        if(mDispatcher)
        {
            delete mDispatcher;
            mDispatcher = 0;
        }
        
        if(mCollisionConfig)
        {
            delete mCollisionConfig;
            mCollisionConfig = 0;
        }
        
        if(mBroadphase)
        {
            delete mBroadphase;
            mBroadphase = 0;
        }

    }

    void setSolution(Solution* _solution){
        mSolution = _solution;
    }

    virtual void iterate()=0;
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
    uint mCyclesPerSecond;
    map<string, pair<btRigidBody*, string>> mWorldEntities;

    btBroadphaseInterface* mBroadphase;
    btDefaultCollisionConfiguration* mCollisionConfig;
    btCollisionDispatcher* mDispatcher;
    btSequentialImpulseConstraintSolver* mSolver;
    btDiscreteDynamicsWorld* mWorld;
    Solution* mSolution;
};

#endif