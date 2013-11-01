#ifndef SFSIMULATION_H
#define SFSIMULATION_H

#include <vector>

#include <boost/lexical_cast.hpp>
#include <boost/random.hpp>
#include <boost/generator_iterator.hpp>
#include <btBulletDynamicsCommon.h>

#include "simulation.h"
#include "common.h"
#include "starfighteragent.h"
#include "staticworldagent.h"
#include "collisionfitness.h"
#include "expectedvaluefitness.h"
#include "goalpointfitness.h"

class SFSimulation : public Simulation
{
public:
    SFSimulation(double _rangefinderRadius, uint _numAgents, uint _numCycles, uint _cyclesPerDecision, uint _cyclesPerSecond, Solution* _solution, ResourceManager* _resourceManager, int _seed);
    SFSimulation(const SFSimulation& other);
    virtual ~SFSimulation();
    virtual void iterate();
    virtual double fitness();
    virtual double realFitness();
    virtual Simulation* getNewCopy() = 0;
    virtual bool initialise() = 0;
    void tick();
    static void tickCallBack(btDynamicsWorld* world, btScalar timeStep){
        SFSimulation* sim = (SFSimulation*)world->getWorldUserInfo();
        sim->tick();
    }

protected:
    double getRayCollisionDistance(string _agentName, const btVector3& _ray);
    void applyUpdateRules(string _agentName);
    vector3 getPositionInfo(string _entityName);
    bool reached(string _agentName);

protected:
    vector3 mGoalpoint;
    vector<string> mAgents, mReached; 
    long mCollisions;
    int mSeed;
    double mRangefinderRadius;
    double mRangefinderVals;
    double mGoalRadius;
};

#endif