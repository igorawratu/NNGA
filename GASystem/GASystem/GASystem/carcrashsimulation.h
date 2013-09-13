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

#include <iostream>
#include <vector>

using namespace std;

class CarCrashSimulation : public Simulation
{
public:
    CarCrashSimulation(uint _agentsPerSide, Line _groupOneFinish, Line _groupTwoFinish, uint _numCycles, uint _cyclesPerDecision, uint _cyclesPerSecond, Solution* _solution, ResourceManager* _resourceManager, int _seed);
    virtual ~CarCrashSimulation();
    virtual void iterate();
    virtual double fitness(vector<Fitness*> _fit);
    virtual Simulation* getNewCopy();
    virtual bool initialise();
    void tick();
    static void tickCallBack(btDynamicsWorld* world, btScalar timeStep){
        CarCrashSimulation* sim = (CarCrashSimulation*)world->getWorldUserInfo();
        sim->tick();
    }

private:
    double getRayCollisionDistance(string _agentName, const btVector3& _ray);
    void applyUpdateRules(string _agentName, uint groupNum);
    double calcDistance(vector3 _from, vector3 _to);
    vector3 getPositionInfo(string _entityName);

private:
    Line mGroupOneFinish, mGroupTwoFinish;
    vector<string> mGroupOneAgents, mGroupTwoAgents; 
    long mCollisions;
    int mSeed;
};

#endif