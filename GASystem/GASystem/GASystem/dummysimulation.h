#ifndef DUMMYSIMULATION_H
#define DUMMYSIMULATION_H

#include "common.h"
#include "simulation.h"

#include <iostream>

using namespace std;

class DummySimulation : public Simulation
{
public:
    DummySimulation(uint _numCycles, uint _cyclesPerDecision, uint _cyclesPerSecond) : Simulation(_numCycles, _cyclesPerDecision, _cyclesPerSecond, 0){}
    virtual ~DummySimulation(){}

    virtual void iterate(){
        mWorld->stepSimulation(1/24.f, 24, 1/24.f);
    }

    virtual double fitness(vector<Fitness*> _fit){
        double finalFitness = 0;
        map<uint, double> dblAcc;
        map<uint, long> intAcc;
        vector<vector3> pos;

        for(uint k = 0; k < _fit.size(); k++)
            finalFitness += _fit[k]->evaluateFitness(pos, dblAcc, intAcc);

        return finalFitness;
    }

    virtual Simulation* getNewCopy(){
        return new DummySimulation(mNumCycles, mCyclesPerDecision, mCyclesPerSecond);
    }

    virtual bool initialise(ResourceManager* _rm){
        if(mInitialised)
            return true;

        btConvexShape* ogreheadColShape = _rm->getBulletCollisionShape("ogrehead.mesh");
        btMotionState* motionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 100)));
        btVector3 inertia(0, 0, 0);
        ogreheadColShape->calculateLocalInertia(0, inertia);

        btRigidBody::btRigidBodyConstructionInfo constructionInfo(0, motionState, ogreheadColShape, inertia);
        btRigidBody* ogreheadRigidBody = new btRigidBody(constructionInfo);
        
        mWorld->addRigidBody(ogreheadRigidBody);
        mWorldEntities["head"] = make_pair(ogreheadRigidBody, "ogrehead.mesh");
        
        mInitialised = true;
        
        return true;
    }

};

#endif