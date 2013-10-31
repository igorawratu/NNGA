#ifndef MOUSESCATTERSIMULATION_H
#define MOUSESCATTERSIMULATION_H

#include <vector>

#include <boost/lexical_cast.hpp>
#include <boost/random.hpp>
#include <boost/generator_iterator.hpp>
#include <btBulletDynamicsCommon.h>
#include <math.h>

#include "simulation.h"
#include "common.h"
#include "caragent.h"
#include "mouseagent.h"
#include "staticworldagent.h"
#include "mouseagent.h"
#include "collisionfitness.h"
#include "expectedvaluefitness.h"

enum AgentType{MOUSE, CAR};

class MouseScatterSimulation : public Simulation
{
public:
    MouseScatterSimulation(double _rangerfinderRadius, uint _numAgents, uint _numCycles, uint _cyclesPerDecision, uint _cyclesPerSecond, Solution* _solution, ResourceManager* _resourceManager, int _seed);
    MouseScatterSimulation(const MouseScatterSimulation& other);
    virtual ~MouseScatterSimulation();
    virtual void iterate();
    virtual double fitness();
    virtual double realFitness();
    virtual Simulation* getNewCopy();
    virtual bool initialise();
    void tick();
    static void tickCallBack(btDynamicsWorld* world, btScalar timeStep){
        MouseScatterSimulation* sim = (MouseScatterSimulation*)world->getWorldUserInfo();
        sim->tick();
    }

private:
    double getRayCollisionDistance(string _agentName, const btVector3& _ray);
    void applyUpdateRules(string _agentName);
    vector3 getPositionInfo(string _entityName);

private:
    vector3 mCenterPoint;
    vector<string> mAgents; 
    long mCollisions;
    int mSeed;
    double mRangefinderRadius;
    double mRangefinderVals;
};

#endif