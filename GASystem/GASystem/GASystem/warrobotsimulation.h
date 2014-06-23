#ifndef WARROBOTSIMULATION_H
#define WARROBOTSIMULATION_H

#include <boost/lexical_cast.hpp>
#include <boost/random.hpp>
#include <boost/generator_iterator.hpp>
#include <btBulletDynamicsCommon.h>

#include "simulation.h"
#include "common.h"
#include "warrobotagent.h"
#include "staticworldagent.h"
#include "finishlinefitness.h"
#include "collisionfitness.h"
#include "winnerfitness.h"
#include "expectedvaluefitness.h"

#include <iostream>
#include <vector>

class WarRobotSimulation : public Simulation
{
public:
    WarRobotSimulation(double _rangefinderRadius, uint _numCycles, uint _cyclesPerDecision, uint _cyclesPerSecond, Solution* _solution, ResourceManager* _resourceManager, int _seed);
    virtual ~WarRobotSimulation();
    virtual void iterate();
    virtual double fitness();
    virtual Simulation* getNewCopy();
    virtual bool initialise();
    void tick();
    virtual double realFitness();
    static void tickCallBack(btDynamicsWorld* world, btScalar timeStep){
        WarRobotSimulation* sim = (WarRobotSimulation*)world->getWorldUserInfo();
        sim->tick();
    }

    virtual vector<string> getRemoveList(){
        return mObjectsToRemove;
    }

    virtual vector<Line> getLines(){
        return mRaysShot;
    }

private:
    void applyUpdateRules(string _agentName, uint _agentNum);
    void checkRayObject(int _groupNum, const btCollisionObject* _obj, int& _team, string& _entityName);

private:
    vector<string> mGroupOneAgents, mGroupTwoAgents; 
    long mCollisions;
    double mRangefinderVals, mRangefinderRadius;
    int mSeed;
    double mVelocityAcc;
    vector<string> mObjectsToRemove;
    vector<Line> mRaysShot;
};

#endif