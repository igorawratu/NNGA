#ifndef CARCRASHSIMULATION_H
#define CARCRASHSIMULATION_H

#include <boost/lexical_cast.hpp>
#include <boost/random.hpp>
#include <boost/generator_iterator.hpp>
#include <btBulletDynamicsCommon.h>

#include "simulation.h"
#include "common.h"
#include "caragent.h"
#include "staticworldagent.h"
#include "finishlinefitness.h"
#include "collisionfitness.h"

#include <iostream>
#include <vector>

using namespace std;

class CarCrashSimulation : public Simulation
{
public:
    CarCrashSimulation(double _rangefinderRadius, uint _agentsPerSide, uint _numCycles, uint _cyclesPerDecision, uint _cyclesPerSecond, Solution* _solution, ResourceManager* _resourceManager, int _seed);
    virtual ~CarCrashSimulation();
    virtual void iterate();
    virtual double fitness();
    virtual Simulation* getNewCopy();
    virtual bool initialise();
    void tick();
    virtual double realFitness();
    static void tickCallBack(btDynamicsWorld* world, btScalar timeStep){
        CarCrashSimulation* sim = (CarCrashSimulation*)world->getWorldUserInfo();
        sim->tick();
    }
    virtual vector<Line> getLines();

private:
    void applyUpdateRules(string _agentName, uint groupNum);
    double calcCrossVal(vector3 a, vector3 b, vector3 c){
        return (b.x - a.x)*(c.z - a.z) - (b.z - a.z)*(c.x - a.x);
    }

private:
    Line mGroupOneFinish, mGroupTwoFinish;
    vector<string> mGroupOneAgents, mGroupTwoAgents; 
    long mCollisions;
    double mRangefinderVals, mRangefinderRadius;
    int mSeed;
};

#endif