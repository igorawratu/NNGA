#ifndef DUMMYSIMULATION_H
#define DUMMYSIMULATION_H

#include "common.h"
#include "simulation.h"

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
        
        vector3 scale(10, 10, 10);

        btCollisionShape* ogreheadColShape = mResourceManager->getBulletCollisionShape("cube.mesh", false, false, vector3(0, 0, 0), scale);
        btMotionState* motionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0)));
        btVector3 inertia(0, 0, 0);
        ogreheadColShape->calculateLocalInertia(0, inertia);

        btRigidBody::btRigidBodyConstructionInfo constructionInfo(0, motionState, ogreheadColShape, inertia);
        btRigidBody* ogreheadRigidBody = new btRigidBody(constructionInfo);
        
        mWorld->addRigidBody(ogreheadRigidBody);
        mWorldEntities["head"] = ObjectInfo(ogreheadRigidBody, "cube.mesh", scale);
        
        mInitialised = true;
        
        return true;
    }

};

#endif