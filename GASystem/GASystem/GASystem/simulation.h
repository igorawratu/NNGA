#ifndef SIMULATION_H
#define SIMULATION_H

#include "common.h"
#include "fitness.h"
#include "resourcemanager.h"
#include "solution.h"
#include "agent.h"

#include <map>
#include <vector>
#include <string>
#include <iostream>

#include <btBulletDynamicsCommon.h>
#include <boost/tuple/tuple.hpp>
#include <BulletCollision/CollisionShapes/btCollisionShape.h>

typedef boost::tuples::tuple<btRigidBody*, string, vector3> ObjectInfo;

using namespace std;

class Simulation
{
public:
    Simulation(uint _numCycles, uint _cyclesPerDecision, uint _cyclesPerSecond, Solution* _solution, ResourceManager* _resourceManager){
        mNumCycles = _numCycles;
        mCyclesPerDecision = _cyclesPerDecision;
        mCycleCounter = 0;
        mCyclesPerSecond = _cyclesPerSecond;
        mInitialised = false;
        mSolution = _solution;
        mResourceManager = _resourceManager;
    
        mBroadphase = new btDbvtBroadphase();
        mCollisionConfig = new btDefaultCollisionConfiguration();

        

        mDispatcher = new btCollisionDispatcher(mCollisionConfig);
        mSolver = new btSequentialImpulseConstraintSolver();
        mWorld = new btDiscreteDynamicsWorld(mDispatcher, mBroadphase, mSolver, mCollisionConfig);
    }

    Simulation(const Simulation& other){}

    virtual ~Simulation(){
        for(map<string, Agent*>::const_iterator iter = mWorldEntities.begin(); iter != mWorldEntities.end(); iter++){
            mWorld->removeRigidBody(iter->second->getRigidBody());
            delete iter->second;
        }

        for(uint k = 0; k < mFitnessFunctions.size(); k++)
            delete mFitnessFunctions[k];
        mFitnessFunctions.clear();

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

    uint getCyclesPerSecond(){
        return mCyclesPerSecond;
    }

    virtual double realFitness()=0;

    void setSolution(Solution* _solution){
        mSolution = _solution;
    }

    virtual void iterate()=0;
    virtual bool initialise()=0;

    bool isInitialised(){
        return mInitialised;
    }

    void runFullSimulation(){
        for(uint k = 0; k < mNumCycles; k++)
            iterate();
    }

    virtual double fitness()=0;
    virtual Simulation* getNewCopy()=0;

    const map<string, Agent*>& getSimulationState(){
        return mWorldEntities;
    }

    virtual vector<string> getRemoveList(){
        return vector<string>();
    }

    virtual vector<Line> getLines(){
        return vector<Line>();
    }

protected:
    vector3 getPositionInfo(string _entityName){
        btRigidBody* rb = mWorldEntities[_entityName]->getRigidBody();
        btTransform trans;
        rb->getMotionState()->getWorldTransform(trans);

        return vector3(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ());
    }


    double getRayCollisionDistance(string _agentName, const btVector3& _ray){
        double dist = 100;

        btCollisionWorld::ClosestRayResultCallback ray = calculateRay(_agentName, _ray);

        vector3 from = getPositionInfo(_agentName);
        if(ray.hasHit())
            dist = from.calcDistance(vector3(ray.m_hitPointWorld.getX(), ray.m_hitPointWorld.getY(), ray.m_hitPointWorld.getZ()));

        return dist;
    }

    btCollisionWorld::ClosestRayResultCallback calculateRay(string _agentName, const btVector3& _ray){
        btVector3 correctedRot = mWorldEntities[_agentName]->getRigidBody()->getWorldTransform().getBasis() * _ray;

        btTransform trans;
        mWorldEntities[_agentName]->getRigidBody()->getMotionState()->getWorldTransform(trans);

        btVector3 agentPosition = trans.getOrigin();

        btVector3 correctedRay(correctedRot.getX() + agentPosition.getX(), correctedRot.getY() + agentPosition.getY(), correctedRot.getZ() + agentPosition.getZ());

        btCollisionWorld::ClosestRayResultCallback ray(agentPosition, correctedRay);

        mWorld->rayTest(agentPosition, correctedRay, ray);

        return ray;
    }

    
protected:
    bool mInitialised;
    uint mNumCycles;
    uint mCyclesPerDecision;
    uint mCycleCounter;
    uint mCyclesPerSecond;
    map<string, Agent*> mWorldEntities;

    btBroadphaseInterface* mBroadphase;
    btDefaultCollisionConfiguration* mCollisionConfig;
    btCollisionDispatcher* mDispatcher;
    btSequentialImpulseConstraintSolver* mSolver;
    btDiscreteDynamicsWorld* mWorld;
    Solution* mSolution;
    ResourceManager* mResourceManager;

    vector<Fitness*> mFitnessFunctions;
};

#endif